from pathlib import Path
import os
import glob
import mujoco
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

BASE_DIR = Path(__file__).resolve().parent

# 読み込むMJCF（XML）ファイルと、対象にするジョイント名
XML_PATH = BASE_DIR / "oscillator.xml"
JOINT_NAME = "x"

# 初期条件（初期変位と初期速度）
X0 = 1.0
V0 = 0.0

# シミュレーション秒数[s]
T_END = 20.0

# 画像出力フォルダ（なければ作成）
IMG_DIR = BASE_DIR / "img"
IMG_DIR.mkdir(exist_ok=True)

# 出力する画像サイズ
WIDTH = 640
HEIGHT = 480

# GIF動画の間引き設定[s]
FRAME_DT = 0.05
PLOT_PATH = BASE_DIR / "theory_vs_mujoco.png"

def create_gif(in_dir, out_filename, frame_dt_s, loop=0):
    """連番画像からGIFを作成"""

    path_list = sorted(glob.glob(os.path.join(in_dir, "*")))
    if len(path_list) == 0:
        raise FileNotFoundError(f"No images found in: {in_dir}")

    imgs = []
    for p in path_list:
        imgs.append(Image.open(p))

    # PILのdurationは「1フレームあたりの表示時間[ms]」
    duration_ms = max(1, int(round(frame_dt_s * 1000)))

    imgs[0].save(
        out_filename,
        save_all=True,
        append_images=imgs[1:],
        optimize=False,
        duration=duration_ms,
        loop=loop,
    )
    return

def calc_theory_displacement(t_log, x0, v0):
    """1自由度ばね・ダンパ系の理論解を計算"""

    # mass=1 [kg], stiffness=20 [N/m], damping=0.5 [N·s/m]
    m = 1.0
    k = 20.0
    c = 0.5

    t = t_log
    state0 = np.array([x0, v0], dtype=float)

    cc = 2 * np.sqrt(m * k)
    zeta = c / cc
    omega = np.sqrt(k / m)

    denom = state0[0] * omega
    n_order_v = state0[1] / denom if not np.isclose(denom, 0.0) else 0.0

    # 減衰振動の場合
    if zeta < 1:
        omega_d = omega * np.sqrt(1 - np.power(zeta, 2))
        sigma = omega_d * zeta
        x_amp = np.sqrt(
            np.power(state0[0], 2)
            + np.power((state0[1] + sigma * state0[0]) / omega_d, 2)
        )
        phi = np.arctan(
            (state0[1] + (sigma * state0[0])) / (state0[0] * omega_d)
        )
        theory = np.exp(- sigma * t) * x_amp * np.cos(omega_d * t - phi)
    # 臨界減衰の場合
    elif np.isclose(zeta, 1.0):
        theory = state0[0] * np.exp(- omega * t) * ((n_order_v + 1) * omega * t + 1)
    # 過減衰の場合
    else:
        root = np.sqrt(zeta ** 2 - 1)
        theory = state0[0] * np.exp(- zeta * omega * t) * (
            np.cosh(omega * t * root)
            + (n_order_v + zeta) / (np.sqrt(zeta ** 2 + 1))
            * np.sinh(omega * t * root)
        )

    return theory

def save_comparison_plot(t_log, sim_x, theory_x, out_path, show_plot=False):
    """MuJoCo計算値と理論値の比較グラフを保存"""

    # ユーザー指定フォーマット
    plt.rcParams["font.size"] = 14
    plt.rcParams["font.family"] = "Times New Roman"
    plt.rcParams["xtick.direction"] = "in"
    plt.rcParams["ytick.direction"] = "in"

    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.xaxis.set_ticks_position("both")
    ax1.yaxis.set_ticks_position("both")
    ax1.plot(t_log, sim_x, label="MuJoCo x(t)")
    ax1.plot(t_log, theory_x, label="Theory x(t)", linestyle="--")
    ax1.set_xlabel("Time[s]")
    ax1.set_ylabel("Disp.[m]")
    ax1.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    if show_plot:
        try:
            plt.show()
        except Exception as exc:
            print(f"Plot display skipped: {exc}")
    plt.close()

# 以前の画像が残っている場合、imgフォルダ内のpngを削除
for p in IMG_DIR.glob("*.png"):
    p.unlink()

# XMLファイルからモデルを読み込み、状態（時間・位置・速度など）を保持するdataを作成
model = mujoco.MjModel.from_xml_path(str(XML_PATH))
data = mujoco.MjData(model)

# ジョイント名から、qpos配列・qvel配列のどこに値が入っているかインデックスを探査
jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, JOINT_NAME)
qpos_adr = model.jnt_qposadr[jid]
qvel_adr = model.jnt_dofadr[jid]

# 状態を初期化してから、初期変位と初期速度を代入、内部計算を整合させるためにmj_forwardを実行
mujoco.mj_resetData(model, data)
data.qpos[qpos_adr] = X0
data.qvel[qvel_adr] = V0
mujoco.mj_forward(model, data)

# オフスクリーンレンダラを作成（ビューアは使わず、画像として取り出す）
renderer = mujoco.Renderer(model, width=WIDTH, height=HEIGHT)

# シミュレーション刻み幅（timestep）から、必要なステップ数と間引き間隔を決める
dt = model.opt.timestep
steps = int(T_END / dt) + 1

# FRAME_DTごとに保存したいので、何ステップごとに保存するかを計算する
save_every = max(1, int(round(FRAME_DT / dt)))

# 理論値比較用に時刻と変位をログする
t_log = []
x_log = []

# 連番画像を書き出し（t=0も含めて保存）
frame_idx = 0
for i in range(steps):
    if i > 0:
        mujoco.mj_step(model, data)

    t_log.append(float(data.time))
    x_log.append(float(data.qpos[qpos_adr]))

    if i % save_every == 0:
        renderer.update_scene(data)
        rgb = renderer.render()
        Image.fromarray(rgb).save(IMG_DIR / f"{frame_idx:06d}.png")
        frame_idx += 1

renderer.close()

# 保存した連番画像からGIFを作成（FRAME_DTをそのまま渡す）
create_gif(in_dir=str(IMG_DIR), out_filename=str(BASE_DIR / "animation.gif"), frame_dt_s=FRAME_DT)

# 理論値とMuJoCo結果を比較してグラフ画像を出力
t_log = np.array(t_log, dtype=float)
x_log = np.array(x_log, dtype=float)
theory = calc_theory_displacement(t_log=t_log, x0=X0, v0=V0)
save_comparison_plot(
    t_log=t_log,
    sim_x=x_log,
    theory_x=theory,
    out_path=PLOT_PATH,
    show_plot=True,
)
