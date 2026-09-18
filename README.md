# ga4aircraft
NSGA-II-based multi-objective optimization of wing geometry for aerodynamic efficiency in low-speed and cruise flight.

# Wing Multi-Objective Optimization

低速飛行および巡航飛行における空力性能を考慮し、  
**NSGA-II** を用いて翼形状を多目的最適化する Python プログラムです。

## 研究目的

本研究では、

> 低速条件と巡航条件において効率的に揚力を得られる翼形状のパラメータは、  
> トレードオフの関係にあるか。  
> また、トレードオフが存在する場合、どのような翼形状がパレート最適となるか。

という問いを扱います。

低速条件と巡航条件のそれぞれについて、機体重量を支えるために必要な揚力を発生させた状態での揚抗比

```math
\frac{L}{D}
```

を評価し、両方を同時に最大化します。

---

## 最適化問題

### 目的関数

以下の2つの目的関数を同時に最大化します。

低速飛行時の揚抗比：

```math
\left(\frac{L}{D}\right)_{\mathrm{low}}
\rightarrow \max
```

巡航飛行時の揚抗比：

```math
\left(\frac{L}{D}\right)_{\mathrm{cruise}}
\rightarrow \max
```

`pymoo` では最小化問題として目的関数を定義するため、プログラム内部では

```math
f_1
=
-\left(\frac{L}{D}\right)_{\mathrm{low}}
```

```math
f_2
=
-\left(\frac{L}{D}\right)_{\mathrm{cruise}}
```

として計算しています。

---

## 制御（決定）変数

現在のモデルでは、翼形状を以下の4つのパラメータで表します。

| 変数 | 記号 | 内容 |
|---|---|---|
| 翼面積 | $S$ | 主翼全体の面積 |
| アスペクト比 | $AR$ | 翼の細長さ |
| テーパー比 | $\lambda$ | 翼端翼弦長 / 翼根翼弦長 |
| ねじり下げ角 | $\varepsilon$ | 翼端側の取付角の減少量 |

アスペクト比は

```math
AR=\frac{b^2}{S}
```

で定義されます。

ここで $b$ は翼幅です。

翼形状は左右対称な台形翼を仮定しています。

---

## 制約条件

現在のプログラムでは、以下のような制約条件を設定しています。

- 最大翼幅
- 最小翼端翼弦長
- 低速飛行時の最大迎角
- 巡航飛行時の最大迎角

各飛行条件では、

```math
L=W
```

となる迎角を求めます。

したがって、単純に大きな揚力を発生する翼を探すのではなく、

> 必要な揚力を発生した状態で、どれだけ小さな抗力で飛行できるか

を評価しています。

---

## 飛行条件

現在のサンプルでは、2つの代表的な飛行速度を設定しています。

```python
V_LOW = 12.0
V_CRUISE = 25.0
```

単位は `m/s` です。

- `V_LOW`  
  離着陸時を想定した低速定常飛行の代表速度

- `V_CRUISE`  
  巡航飛行の代表速度

これらの値は研究対象とする航空機に応じて変更します。

> [!NOTE]
> 本プログラムの「低速条件」は、実際の離陸・着陸運動そのものを
> シミュレーションするものではありません。  
> 低速での水平定常飛行を代表する設計点として扱っています。

---

## 空力モデル

現在のサンプルプログラムでは、

- Prandtl の揚力線理論
- 簡略化した翼型抗力モデル

を組み合わせて翼性能を計算しています。

全抗力係数は概念的に、

```math
C_D
=
C_{D,\mathrm{profile}}
+
C_{D,i}
```

として評価しています。

また、水平定常飛行では

```math
L=W
```

なので、必要揚力係数は

```math
C_{L,\mathrm{required}}
=
\frac{2W}{\rho V^2 S}
```

で求められます。

### 注意

現在使用している翼型抗力モデルは、NSGA-II を用いた最適化アルゴリズムの動作確認を目的とした**簡易モデル**です。

最終的な研究では、この部分を

- XFOIL
- XFLR5
- flow5

などによる翼型・翼の空力解析結果に置き換えることを想定しています。

---

## 最適化アルゴリズム

多目的最適化には **NSGA-II**  
（Non-dominated Sorting Genetic Algorithm II）を使用します。

Python の多目的最適化ライブラリ [`pymoo`](https://pymoo.org/) を利用しています。

NSGA-II は主に、

- 非優越ソート
- 混雑距離
- 選択
- 交叉
- 突然変異

を用いて、複数の目的関数に対するパレート最適解を探索します。

本研究では、

```math
\boldsymbol{x}
=
(S,\ AR,\ \lambda,\ \varepsilon)
```

の組合せを1つの個体として扱います。

---

## パレート最適解

多目的最適化では、必ずしも唯一の「最適な翼」が得られるとは限りません。

例えば、

- 低速性能に優れる翼
- 巡航性能に優れる翼
- 両者をバランスよく満たす翼

がそれぞれ存在する可能性があります。

どれか一方の目的関数を改善すると、もう一方が悪化してしまうような解の集合を**パレート最適解**と呼びます。

本研究では、

```math
\left(\frac{L}{D}\right)_{\mathrm{low}}
```

と

```math
\left(\frac{L}{D}\right)_{\mathrm{cruise}}
```

の関係から、低速性能と巡航性能の間にトレードオフが存在するかを調べます。

---

## 必要環境

- Python 3
- NumPy
- Matplotlib
- pymoo

必要なパッケージは以下のコマンドでインストールできます。

```bash
pip install numpy matplotlib pymoo
```

---

## 実行方法

リポジトリをクローンした後、プログラムを実行します。

```bash
python wing_optimization.py
```

実行すると、NSGA-II による最適化が開始されます。

---

## 出力

解析結果は以下のディレクトリに保存されます。

```text
output/
├── csv/
└── figure/
```

### CSV

```text
output/csv/
├── pareto_solutions.csv
├── representative_solutions.csv
└── convergence_history.csv
```

### `pareto_solutions.csv`

最終的に得られたパレート最適解を保存します。

主な項目：

- 翼面積 $S$
- アスペクト比 $AR$
- テーパー比 $\lambda$
- ねじり下げ角 $\varepsilon$
- 翼幅
- 翼根翼弦長
- 翼端翼弦長
- 低速時の揚抗比
- 巡航時の揚抗比

### `representative_solutions.csv`

以下の代表的な翼形状を保存します。

- 低速性能を重視した解
- 巡航性能を重視した解
- 両者の妥協解

### `convergence_history.csv`

世代ごとの最適化の進行状況を保存します。

---

## 出力される図

```text
output/figure/
├── pareto_front.png
├── convergence_low_speed.png
├── convergence_cruise.png
└── representative_wing_planforms.png
```

### Pareto front

`pareto_front.png`

横軸：

```math
\left(\frac{L}{D}\right)_{\mathrm{low}}
```

縦軸：

```math
\left(\frac{L}{D}\right)_{\mathrm{cruise}}
```

として、パレート最適解を表示します。

本研究において最も重要な図です。

この分布から、

> 低速性能と巡航性能の間にトレードオフが存在するか

を調べます。

---

### Convergence history

- `convergence_low_speed.png`
- `convergence_cruise.png`

各世代における

- Best feasible solution
- Mean feasible solution

の変化を示します。

ここで **feasible solution** とは、翼幅や迎角などの制約条件をすべて満たした実行可能解を意味します。

---

### Wing planforms

`representative_wing_planforms.png`

パレート最適解の中から選んだ

- 低速重視型
- 巡航重視型
- 妥協型

の翼平面形を比較します。

---

## 研究上の問い

本研究では、主に以下の問いに答えることを目的とします。

1. 低速飛行と巡航飛行における最適な翼形状は異なるか。
2. 両飛行条件の揚抗比にはトレードオフが存在するか。
3. パレート最適解では、翼面積 $S$、アスペクト比 $AR$、テーパー比 $\lambda$、ねじり下げ角 $\varepsilon$ がどのように変化するか。
4. 低速重視型、巡航重視型、妥協型の翼にはどのような形状上の特徴があるか。

---

## 今後の課題

---

## 参考文献

- Deb, K., Pratap, A., Agarwal, S., & Meyarivan, T. (2002).  
  *A Fast and Elitist Multiobjective Genetic Algorithm: NSGA-II*.  
  IEEE Transactions on Evolutionary Computation, 6(2), 182–197.

- Wei, X., Wang, X., & Chen, S. (2020).  
  *Research on Parameterization and Optimization Procedure of Low-Reynolds-Number Airfoils Based on Genetic Algorithm and Bezier Curve*.  
  Advances in Engineering Software, 149, 102864.

- Drela, M.  
  *XFOIL: An Analysis and Design System.*

- XFLR5 / flow5 Documentation

- pymoo Documentation  
  https://pymoo.org/
