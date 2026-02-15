# 1DOF Spring-Mass-Damper Example

## 目的

この例題では、1自由度ばね・ダンパ系をMuJoCoでシミュレーションし、理論解と比較します。  
以下の3つを同時に確認できます。

- 時系列シミュレーション
- アニメーションGIF
- 理論値とMuJoCo値の重ね描きプロット

## モデル設定

本例題のパラメータは以下です。

- `m = 1.0` kg
- `k = 20.0` N/m
- `c = 0.5` N·s/m
- 初期変位 `X0 = 1.0` m
- 初期速度 `V0 = 0.0` m/s

## モデルのジョイント関係

このモデルは、`world`に対して質点ボディ`mass`を1つだけ持つ最小構成です。  
`mass`ボディにはスライダジョイント`x`があり、x軸方向にのみ並進します。

- 親子関係: `world` -> `body(name="mass")`
- 関節: `joint(name="x", type="slide", axis="1 0 0")`
- 復元力と減衰: `stiffness="20"`, `damping="0.5"`
- 質量: `geom(type="sphere", mass="1")`

つまり、自由度は1つ（x方向変位）で、ばね・ダンパ付きの1自由度振動系として動作します。

## MJCF (oscillator.xml)

```xml
<mujoco model="1d_oscillator">
  <statistic center="0 0 0" extent="2"/>
  <option gravity="0 0 0" timestep="0.001"/>
  <worldbody>
    <body name="mass" pos="0 0 0">
      <joint name="x" type="slide" axis="1 0 0"
             stiffness="20" damping="0.5" springref="0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>
```

## 運動方程式

$$
m\ddot{x}(t) + c\dot{x}(t) + kx(t) = 0
$$

## 定義量

$$
\omega_n = \sqrt{\frac{k}{m}}
$$

$$
\zeta = \frac{c}{2\sqrt{mk}}
$$

$$
\omega_d = \omega_n\sqrt{1-\zeta^2}
$$

## 理論式（実装で比較に使う形）

以下は `calc_theory_displacement()` で用いる形です。

$$
x(0)=X_0,\quad \dot{x}(0)=V_0
$$

$$
n_v=\frac{V_0}{X_0\omega_n}
$$

### 不足減衰

$$
\zeta<1,\quad
x(t)=e^{-\sigma t}X\cos(\omega_d t-\phi)
$$

$$
X=\sqrt{X_0^2+\left(\frac{V_0+\sigma X_0}{\omega_d}\right)^2},\quad
\phi=\arctan\!\left(\frac{V_0+\sigma X_0}{X_0\omega_d}\right)
$$

### 臨界減衰

$$
\zeta=1,\quad
x(t)=X_0e^{-\omega_n t}\left((n_v+1)\omega_n t+1\right)
$$

### 過減衰

$$
\zeta>1,\quad
x(t)=X_0e^{-\zeta\omega_n t}\left[
\cosh\!\left(\omega_n t\sqrt{\zeta^2-1}\right)
+\frac{n_v+\zeta}{\sqrt{\zeta^2+1}}
\sinh\!\left(\omega_n t\sqrt{\zeta^2-1}\right)
\right]
$$

## 解の分類

### 不足減衰

$$
\zeta < 1
$$

### 臨界減衰

$$
\zeta = 1
$$

### 過減衰

$$
\zeta > 1
$$

## 実装との対応

理論解計算は `mujoco_1dof_sim.py` の `calc_theory_displacement()` で行います。

- `t_log`: シミュレーション時刻配列
- `x_log`: MuJoCoの変位ログ
- `theory`: 理論式から計算した変位

分岐は以下に対応します。

- `zeta < 1`: 不足減衰の式
- `np.isclose(zeta, 1.0)`: 臨界減衰の式
- それ以外: 過減衰の式

比較プロットは `save_comparison_plot()` で保存し、最後に表示します。

## 実行方法

```bash
cd src/examples/1dof_spring_mass
../../../venv/bin/python mujoco_1dof_sim.py
```

## 生成物

- `animation.gif`: MuJoCoレンダリングのアニメーション
- `theory_vs_mujoco.png`: 理論値とMuJoCo値の比較図
- `img/*.png`: GIF生成用の連番フレーム

## 出力例

### Animation

![animation](animation.gif)

### Theory vs MuJoCo

![theory-vs-mujoco](theory_vs_mujoco.png)

## 結果の見方

- 減衰によって時間とともに振幅が小さくなること
- MuJoCo計算値と理論値の波形が概ね一致すること
- 初期条件を変えると位相や振幅の推移が変わること

## 補足

- GitHub上の数式表示崩れを避けるため、数式は見出しや入れ子リストに入れず独立ブロックで記述しています。
- 環境によっては描画バックエンドの制約で実行時にレンダラ初期化エラーが出る場合があります。

---

戻る: [表紙 README](../../../README.md)
