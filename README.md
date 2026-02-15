# mujoco-examples

MuJoCoを使った例題を集めるリポジトリです。  
シミュレーション結果を可視化するだけでなく、理論式と比較して挙動を確認できる構成を目指しています。

## Quick Start

### 1. 仮想環境を有効化

```bash
source venv/bin/activate
```

### 2. 依存関係をインストール

```bash
pip install mujoco pillow numpy matplotlib
```

### 3. 例題を実行

```bash
cd src/examples/1dof_spring_mass
python mujoco_1dof_sim.py
```

## Repository Layout

本リポジトリの例題は以下の規則で配置します。

- `src/examples/<example_name>/`
- `<example_name>` は lowercase `snake_case`
- シミュレーションスクリプトとMJCFファイルを同一フォルダに配置

各例題の詳細は `src/examples/<example_name>/Example_*.md` を参照してください。

## Examples

### 1DOF Spring-Mass-Damper

- パス: `src/examples/1dof_spring_mass/`
- 詳細解説: [`src/examples/1dof_spring_mass/Example_1DOF_Spring-Mass-Damper.md`](src/examples/1dof_spring_mass/Example_1DOF_Spring-Mass-Damper.md)
- 実装内容:
  - MuJoCoシミュレーション
  - 理論解との比較
  - GIF出力
  - Matplotlibによる比較グラフ出力

詳細READMEへの直接リンク: [1DOF Spring-Mass-Damper 解説](src/examples/1dof_spring_mass/Example_1DOF_Spring-Mass-Damper.md)

## Outputs

### Animation (GIF)

![animation](src/examples/1dof_spring_mass/animation.gif)

### Theory vs MuJoCo

![theory-vs-mujoco](src/examples/1dof_spring_mass/theory_vs_mujoco.png)

## Notes

- 実行環境によってはOpenGL/GUI接続の制約でレンダラ初期化が失敗する場合があります。
- その場合はGUIが利用可能なローカル環境で実行してください。
