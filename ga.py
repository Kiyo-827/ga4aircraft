import os
import csv
import numpy as np
import matplotlib.pyplot as plt

from pymoo.core.problem import ElementwiseProblem
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.optimize import minimize


# ============================================================
# 0. Output directories
# ============================================================

CSV_DIR = "./output/csv"
FIG_DIR = "./output/figure"

os.makedirs(CSV_DIR, exist_ok=True)
os.makedirs(FIG_DIR, exist_ok=True)


# ============================================================
# 1. Flight conditions
# ============================================================

RHO = 1.225          # air density [kg/m^3]
MU = 1.81e-5         # dynamic viscosity [Pa s]
G = 9.80665          # gravity [m/s^2]

MASS = 5.0           # aircraft mass [kg]
WEIGHT = MASS * G    # aircraft weight [N]

V_LOW = 12.0         # representative low-speed flight [m/s]
V_CRUISE = 25.0      # representative cruise flight [m/s]


# ============================================================
# 2. Constraints
# ============================================================

ALPHA_MAX = 10.0     # allowable angle of attack [deg]
MAX_SPAN = 3.0       # maximum wing span [m]
MIN_TIP_CHORD = 0.08 # minimum tip chord [m]


# ============================================================
# 3. Wing geometry
# ============================================================

def wing_geometry(S, AR, taper):
    span = np.sqrt(AR * S)
    c_root = 2.0 * S / (span * (1.0 + taper))
    c_tip = taper * c_root
    return span, c_root, c_tip


# ============================================================
# 4. Simplified Prandtl lifting-line method
# ============================================================

def solve_lifting_line(S, AR, taper, washout_deg, alpha_root_deg, V, N=10):
    span, c_root, c_tip = wing_geometry(S, AR, taper)

    theta = np.arange(1, N + 1) * np.pi / (2.0 * N)
    eta = np.cos(theta)

    chord = c_root - (c_root - c_tip) * eta

    alpha_geo = np.deg2rad(alpha_root_deg) - np.deg2rad(washout_deg) * eta

    alpha_L0 = np.deg2rad(-2.0)   # example fixed airfoil zero-lift angle
    a0 = 2.0 * np.pi              # 2D lift curve slope

    n = 2 * np.arange(1, N + 1) - 1

    matrix = np.zeros((N, N))
    rhs = alpha_geo - alpha_L0

    for i in range(N):
        matrix[i, :] = np.sin(n * theta[i]) * (
            4.0 * span / (a0 * chord[i]) + n / np.sin(theta[i])
        )

    A = np.linalg.solve(matrix, rhs)

    CL = np.pi * AR * A[0]
    CDi = np.pi * AR * np.sum(n * A**2)

    if CDi > 0.0:
        e = CL**2 / (np.pi * AR * CDi)
    else:
        e = np.nan

    return CL, CDi, e, A


# ============================================================
# 5. Find alpha for required lift
# ============================================================

def find_alpha_for_required_CL(S, AR, taper, washout_deg, V, CL_required):
    CL_0 = solve_lifting_line(S, AR, taper, washout_deg, 0.0, V)[0]
    CL_1 = solve_lifting_line(S, AR, taper, washout_deg, 1.0, V)[0]

    slope = CL_1 - CL_0
    if abs(slope) < 1e-12:
        return np.nan, None

    alpha_root = (CL_required - CL_0) / slope
    result = solve_lifting_line(S, AR, taper, washout_deg, alpha_root, V)
    return alpha_root, result


# ============================================================
# 6. Demonstration profile-drag model
# ============================================================

def section_drag_coefficient_demo(cl, Reynolds):
    Reynolds = np.maximum(Reynolds, 5.0e4)

    cd0 = 0.0105 * (Reynolds / 2.0e5) ** (-0.20)
    cd = cd0 + 0.008 * (cl - 0.4) ** 2
    return cd


# ============================================================
# 7. Profile drag
# ============================================================

def calculate_profile_drag(S, AR, taper, V, A):
    span, c_root, c_tip = wing_geometry(S, AR, taper)

    N = len(A)
    n = 2 * np.arange(1, N + 1) - 1

    theta = np.linspace(
        1.0e-5,
        np.pi / 2.0,
        400,
    )

    eta = np.cos(theta)

    chord = (
        c_root
        - (c_root - c_tip) * eta
    )

    # Fourier circulation distribution
    fourier_sum = np.sum(
        A[None, :]
        * np.sin(
            theta[:, None]
            * n[None, :]
        ),
        axis=1,
    )

    # Local section lift coefficient
    cl_local = (
        4.0
        * span
        / chord
        * fourier_sum
    )

    # Local Reynolds number
    Reynolds = (
        RHO
        * V
        * chord
        / MU
    )

    # Section drag coefficient
    cd_local = section_drag_coefficient_demo(
        cl_local,
        Reynolds,
    )

    # Integration term
    integrand = (
        chord
        * cd_local
        * (span / 2.0)
        * np.sin(theta)
    )

    # Numerical integration
    half_wing_drag = np.trapezoid(
        integrand,
        theta,
    )

    # Both sides of the wing
    CD_profile = (
        2.0
        * half_wing_drag
        / S
    )

    return CD_profile


# ============================================================
# 8. Evaluate one flight condition
# ============================================================

def evaluate_flight_condition(S, AR, taper, washout_deg, V):
    q = 0.5 * RHO * V**2
    CL_required = WEIGHT / (q * S)

    alpha_root, result = find_alpha_for_required_CL(
        S, AR, taper, washout_deg, V, CL_required
    )

    if result is None:
        return {
            "CL": np.nan,
            "CD": np.nan,
            "CDi": np.nan,
            "CDp": np.nan,
            "LD": 0.0,
            "alpha": 999.0,
            "e": np.nan,
        }

    CL, CD_induced, e, A = result
    CD_profile = calculate_profile_drag(S, AR, taper, V, A)
    CD_total = CD_induced + CD_profile

    LD = CL / CD_total

    return {
        "CL": CL,
        "CD": CD_total,
        "CDi": CD_induced,
        "CDp": CD_profile,
        "LD": LD,
        "alpha": alpha_root,
        "e": e,
    }


# ============================================================
# 9. Problem definition for pymoo
# ============================================================

class WingOptimizationProblem(ElementwiseProblem):
    def __init__(self):
        xl = np.array([
            0.45,   # S
            5.0,    # AR
            0.30,   # taper
            0.0,    # washout
        ])

        xu = np.array([
            1.20,   # S
            12.0,   # AR
            1.00,   # taper
            4.0,    # washout
        ])

        super().__init__(
            n_var=4,
            n_obj=2,
            n_ieq_constr=4,
            xl=xl,
            xu=xu,
        )

    def _evaluate(self, x, out, *args, **kwargs):
        S, AR, taper, washout = x

        span, c_root, c_tip = wing_geometry(S, AR, taper)

        low = evaluate_flight_condition(S, AR, taper, washout, V_LOW)
        cruise = evaluate_flight_condition(S, AR, taper, washout, V_CRUISE)

        # maximize L/D  -> minimize -L/D
        out["F"] = np.array([
            -low["LD"],
            -cruise["LD"],
        ])

        # g(x) <= 0
        out["G"] = np.array([
            low["alpha"] - ALPHA_MAX,
            cruise["alpha"] - ALPHA_MAX,
            span - MAX_SPAN,
            MIN_TIP_CHORD - c_tip,
        ])


# ============================================================
# 10. Run NSGA-II
# ============================================================

problem = WingOptimizationProblem()

algorithm = NSGA2(
    pop_size=120,
    eliminate_duplicates=True,
)

result = minimize(
    problem,
    algorithm,
    termination=("n_gen", 200),
    seed=42,
    verbose=True,
    save_history=True,
)

if result.X is None:
    print("No feasible solution was found.")
    raise SystemExit


# ============================================================
# 11. Extract final Pareto solutions
# ============================================================

X = result.X
F = result.F

LD_low = -F[:, 0]
LD_cruise = -F[:, 1]

# Representative solutions
index_low = np.argmax(LD_low)
index_cruise = np.argmax(LD_cruise)

low_norm = (LD_low - LD_low.min()) / (LD_low.max() - LD_low.min() + 1e-12)
cruise_norm = (LD_cruise - LD_cruise.min()) / (LD_cruise.max() - LD_cruise.min() + 1e-12)
distance_to_ideal = np.sqrt((1.0 - low_norm)**2 + (1.0 - cruise_norm)**2)
index_compromise = np.argmin(distance_to_ideal)


# ============================================================
# 12. Save final Pareto solutions to CSV
# ============================================================

pareto_csv = os.path.join(CSV_DIR, "pareto_solutions.csv")

with open(pareto_csv, "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow([
        "solution_id",
        "S_m2",
        "AR",
        "taper",
        "washout_deg",
        "span_m",
        "root_chord_m",
        "tip_chord_m",
        "LD_low",
        "LD_cruise",
    ])

    for i in range(len(X)):
        S, AR, taper, washout = X[i]
        span, c_root, c_tip = wing_geometry(S, AR, taper)

        writer.writerow([
            i,
            S,
            AR,
            taper,
            washout,
            span,
            c_root,
            c_tip,
            LD_low[i],
            LD_cruise[i],
        ])


# ============================================================
# 13. Save representative solutions to CSV
# ============================================================

rep_csv = os.path.join(CSV_DIR, "representative_solutions.csv")

representatives = [
    ("best_low_speed", index_low),
    ("best_cruise", index_cruise),
    ("compromise", index_compromise),
]

with open(rep_csv, "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow([
        "name",
        "solution_id",
        "S_m2",
        "AR",
        "taper",
        "washout_deg",
        "span_m",
        "root_chord_m",
        "tip_chord_m",
        "LD_low",
        "LD_cruise",
    ])

    for name, idx in representatives:
        S, AR, taper, washout = X[idx]
        span, c_root, c_tip = wing_geometry(S, AR, taper)

        writer.writerow([
            name,
            idx,
            S,
            AR,
            taper,
            washout,
            span,
            c_root,
            c_tip,
            LD_low[idx],
            LD_cruise[idx],
        ])


# ============================================================
# 14. Save convergence history to CSV
# ============================================================

history_csv = os.path.join(CSV_DIR, "convergence_history.csv")

history_rows = []

for gen_idx, algo in enumerate(result.history, start=1):
    pop = algo.pop
    F_pop = pop.get("F")
    CV_pop = pop.get("CV")

    feasible = CV_pop <= 0.0

    if np.any(feasible):
        F_feasible = F_pop[feasible.flatten()]
        ld_low_feasible = -F_feasible[:, 0]
        ld_cruise_feasible = -F_feasible[:, 1]

        best_low = np.max(ld_low_feasible)
        best_cruise = np.max(ld_cruise_feasible)
        mean_low = np.mean(ld_low_feasible)
        mean_cruise = np.mean(ld_cruise_feasible)

    else:
        best_low = np.nan
        best_cruise = np.nan
        mean_low = np.nan
        mean_cruise = np.nan

    history_rows.append([
        gen_idx,
        best_low,
        mean_low,
        best_cruise,
        mean_cruise,
    ])

with open(history_csv, "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow([
        "generation",
        "best_LD_low",
        "mean_LD_low",
        "best_LD_cruise",
        "mean_LD_cruise",
    ])
    writer.writerows(history_rows)


# ============================================================
# 15. Figure 1: Pareto front
# ============================================================

plt.figure(figsize=(7, 6))
plt.scatter(LD_low, LD_cruise, s=35, label="Pareto solutions")
plt.scatter(LD_low[index_low], LD_cruise[index_low], s=140, marker="*", label="Best low-speed")
plt.scatter(LD_low[index_cruise], LD_cruise[index_cruise], s=140, marker="*", label="Best cruise")
plt.scatter(LD_low[index_compromise], LD_cruise[index_compromise], s=140, marker="*", label="Compromise")

plt.xlabel("L/D at low speed")
plt.ylabel("L/D at cruise")
plt.title("Pareto Front of Wing Design")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(FIG_DIR, "pareto_front.png"), dpi=200)
plt.close()


# ============================================================
# 16. Figure 2: Convergence history (low-speed)
# ============================================================

gens = np.array([row[0] for row in history_rows])
best_low_hist = np.array([row[1] for row in history_rows])
mean_low_hist = np.array([row[2] for row in history_rows])

plt.figure(figsize=(7, 5))
plt.plot(gens, best_low_hist, label="Best feasible solution")
plt.plot(gens, mean_low_hist, label="Mean feasible solution")
plt.xlabel("Generation")
plt.ylabel("L/D at low speed")
plt.title("Convergence History: Low-speed Objective")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(FIG_DIR, "convergence_low_speed.png"), dpi=200)
plt.close()


# ============================================================
# 17. Figure 3: Convergence history (cruise)
# ============================================================

best_cruise_hist = np.array([row[3] for row in history_rows])
mean_cruise_hist = np.array([row[4] for row in history_rows])

plt.figure(figsize=(7, 5))
plt.plot(gens, best_cruise_hist, label="Best feasible solution")
plt.plot(gens, mean_cruise_hist, label="Mean feasible solution")
plt.xlabel("Generation")
plt.ylabel("L/D at cruise")
plt.title("Convergence History: Cruise Objective")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(FIG_DIR, "convergence_cruise.png"), dpi=200)
plt.close()


# ============================================================
# 18. Figure 4: Wing planform comparison
# ============================================================

def wing_polygon_points(S, AR, taper):
    span, c_root, c_tip = wing_geometry(S, AR, taper)
    half_span = span / 2.0

    # trapezoidal wing planform
    x = np.array([0.0, c_root, c_tip, 0.0, 0.0])
    y = np.array([0.0, 0.0, half_span, half_span, 0.0])

    return x, y, span, c_root, c_tip

plt.figure(figsize=(8, 6))

colors = ["tab:blue", "tab:orange", "tab:green"]
labels = ["Best low-speed", "Best cruise", "Compromise"]
indices = [index_low, index_cruise, index_compromise]

for color, label, idx in zip(colors, labels, indices):
    S, AR, taper, washout = X[idx]
    x_poly, y_poly, span, c_root, c_tip = wing_polygon_points(S, AR, taper)

    # right half
    plt.plot(x_poly, y_poly, color=color, label=label)
    # left half (mirror)
    plt.plot(x_poly, -y_poly, color=color)

plt.axis("equal")
plt.xlabel("Chordwise direction [m]")
plt.ylabel("Spanwise direction [m]")
plt.title("Representative Wing Planforms")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(FIG_DIR, "representative_wing_planforms.png"), dpi=200)
plt.close()


# ============================================================
# 19. Console output
# ============================================================

def print_solution(name, idx):
    S, AR, taper, washout = X[idx]
    span, c_root, c_tip = wing_geometry(S, AR, taper)

    print()
    print("--------------------------------------------------")
    print(name)
    print("--------------------------------------------------")
    print(f"S            = {S:.4f} m^2")
    print(f"AR           = {AR:.4f}")
    print(f"taper        = {taper:.4f}")
    print(f"washout      = {washout:.4f} deg")
    print(f"span         = {span:.4f} m")
    print(f"root chord   = {c_root:.4f} m")
    print(f"tip chord    = {c_tip:.4f} m")
    print(f"L/D low      = {LD_low[idx]:.4f}")
    print(f"L/D cruise   = {LD_cruise[idx]:.4f}")

print_solution("Best low-speed solution", index_low)
print_solution("Best cruise solution", index_cruise)
print_solution("Compromise solution", index_compromise)

print()
print("Saved files:")
print(f"  {pareto_csv}")
print(f"  {rep_csv}")
print(f"  {history_csv}")
print(f"  {os.path.join(FIG_DIR, 'pareto_front.png')}")
print(f"  {os.path.join(FIG_DIR, 'convergence_low_speed.png')}")
print(f"  {os.path.join(FIG_DIR, 'convergence_cruise.png')}")
print(f"  {os.path.join(FIG_DIR, 'representative_wing_planforms.png')}")