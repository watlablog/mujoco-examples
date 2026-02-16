from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np
from PIL import Image, ImageDraw
from scipy.integrate import solve_ivp

BASE_DIR = Path(__file__).resolve().parent
XML_PATH = BASE_DIR / "double_pendulum.xml"
PLOT_PATH = BASE_DIR / "comparison_plot.png"
PENDULUM_GIF_PATH = BASE_DIR / "animation.gif"
OVERLAY_GIF_PATH = BASE_DIR / "comparison_overlay.gif"
IMG_DIR = BASE_DIR / "img"
OVERLAY_IMG_DIR = BASE_DIR / "img_overlay"
IMG_DIR.mkdir(exist_ok=True)
OVERLAY_IMG_DIR.mkdir(exist_ok=True)

JOINT1_NAME = "hinge1"
JOINT2_NAME = "hinge2"

# Point-mass double pendulum parameters (must match XML inertials)
M1 = 1.0
M2 = 1.0
L1 = 1.0
L2 = 1.0
G = 9.81

# Initial conditions (relative angles in MuJoCo generalized coordinates)
Q1_0 = 2.35
Q2_0 = 1.10
DQ1_0 = 0.80
DQ2_0 = -0.60

# Runtime settings
T_END = 16.0
SAMPLE_DT = 0.005
FRAME_DT = 0.03
OVERLAY_FRAME_DT = 0.06
SHOW_PLOT = True

THETA1_RMSE_TARGET_DEG = 0.8
THETA2_REL_RMSE_TARGET_DEG = 1.0

# Simple 2D pendulum animation canvas settings
PANEL_W = 340
PANEL_H = 420
MARGIN = 36


def clear_png_frames(in_dir):
    for p in in_dir.glob("*.png"):
        p.unlink()


def create_gif(in_dir, out_filename, frame_dt_s, loop=0):
    path_list = sorted(in_dir.glob("*.png"))
    if len(path_list) == 0:
        raise FileNotFoundError(f"No images found in: {in_dir}")

    imgs = [Image.open(p).convert("P", palette=Image.ADAPTIVE, colors=128) for p in path_list]
    duration_ms = max(1, int(round(frame_dt_s * 1000)))

    imgs[0].save(
        out_filename,
        save_all=True,
        append_images=imgs[1:],
        optimize=True,
        duration=duration_ms,
        loop=loop,
    )


def rhs(_t, y):
    # y = [q1, q2, dq1, dq2] where q2 is relative joint angle
    q1, q2, dq1, dq2 = y

    c2 = np.cos(q2)
    s2 = np.sin(q2)

    m11 = (M1 + M2) * L1**2 + M2 * L2**2 + 2.0 * M2 * L1 * L2 * c2
    m12 = M2 * L2**2 + M2 * L1 * L2 * c2
    m22 = M2 * L2**2

    h = -M2 * L1 * L2 * s2
    c1 = h * (2.0 * dq1 * dq2 + dq2**2)
    c2_term = -h * dq1**2

    g1 = (M1 + M2) * G * L1 * np.sin(q1) + M2 * G * L2 * np.sin(q1 + q2)
    g2 = M2 * G * L2 * np.sin(q1 + q2)

    m_mat = np.array([[m11, m12], [m12, m22]], dtype=float)
    rhs_vec = -np.array([c1 + g1, c2_term + g2], dtype=float)
    ddq = np.linalg.solve(m_mat, rhs_vec)

    return np.array([dq1, dq2, ddq[0], ddq[1]], dtype=float)


def run_ode(t_eval, y0):
    sol = solve_ivp(
        rhs,
        (float(t_eval[0]), float(t_eval[-1])),
        y0,
        t_eval=t_eval,
        rtol=1e-9,
        atol=1e-11,
        method="RK45",
    )
    if not sol.success:
        raise RuntimeError(f"solve_ivp failed: {sol.message}")

    q = sol.y[0:2, :].T
    dq = sol.y[2:4, :].T
    return q, dq


def run_mujoco_case(model, q0, dq0, t_eval):
    data = mujoco.MjData(model)

    j1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, JOINT1_NAME)
    j2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, JOINT2_NAME)
    qpos_adr1 = model.jnt_qposadr[j1]
    qpos_adr2 = model.jnt_qposadr[j2]
    qvel_adr1 = model.jnt_dofadr[j1]
    qvel_adr2 = model.jnt_dofadr[j2]

    mujoco.mj_resetData(model, data)
    data.qpos[qpos_adr1] = q0[0]
    data.qpos[qpos_adr2] = q0[1]
    data.qvel[qvel_adr1] = dq0[0]
    data.qvel[qvel_adr2] = dq0[1]
    mujoco.mj_forward(model, data)

    dt = model.opt.timestep
    sample_every = max(1, int(round(SAMPLE_DT / dt)))
    total_steps = int(round((t_eval[-1] - t_eval[0]) / dt))

    q_log = np.zeros((len(t_eval), 2), dtype=float)
    dq_log = np.zeros((len(t_eval), 2), dtype=float)

    k = 0
    for step in range(total_steps + 1):
        if step % sample_every == 0:
            if k >= len(t_eval):
                break
            q_log[k, 0] = data.qpos[qpos_adr1]
            q_log[k, 1] = data.qpos[qpos_adr2]
            dq_log[k, 0] = data.qvel[qvel_adr1]
            dq_log[k, 1] = data.qvel[qvel_adr2]
            k += 1

        if step < total_steps:
            mujoco.mj_step(model, data)

    if k < len(t_eval):
        q_log = q_log[:k]
        dq_log = dq_log[:k]

    return q_log, dq_log


def endpoint_positions(theta1, theta2_abs):
    x1 = L1 * np.sin(theta1)
    z1 = -L1 * np.cos(theta1)
    x2 = x1 + L2 * np.sin(theta2_abs)
    z2 = z1 - L2 * np.cos(theta2_abs)
    return x1, z1, x2, z2


def rmse_deg(a_rad, b_rad):
    return float(np.sqrt(np.mean(np.rad2deg(a_rad - b_rad) ** 2)))


def draw_single_pendulum_panel(theta1, theta2_abs, title, subtitle):
    image = Image.new("RGB", (PANEL_W, PANEL_H), color=(245, 247, 250))
    draw = ImageDraw.Draw(image)

    # Keep full motion in-frame and map world z-up to screen y-down.
    scale = min(
        (PANEL_W - 2 * MARGIN) / (2.3 * (L1 + L2)),
        (PANEL_H - 2 * MARGIN) / (2.3 * (L1 + L2)),
    )
    x0 = PANEL_W // 2
    y0 = PANEL_H // 2

    x1, z1, x2, z2 = endpoint_positions(theta1, theta2_abs)

    p0 = (x0, y0)
    p1 = (int(x0 + scale * x1), int(y0 - scale * z1))
    p2 = (int(x0 + scale * x2), int(y0 - scale * z2))

    draw.line([p0, p1], fill=(70, 120, 220), width=6)
    draw.line([p1, p2], fill=(220, 110, 60), width=6)

    r = 7
    draw.ellipse((p0[0] - r, p0[1] - r, p0[0] + r, p0[1] + r), fill=(30, 30, 30))
    draw.ellipse((p1[0] - r, p1[1] - r, p1[0] + r, p1[1] + r), fill=(50, 90, 180))
    draw.ellipse((p2[0] - r, p2[1] - r, p2[0] + r, p2[1] + r), fill=(180, 70, 30))

    draw.rectangle((8, 8, PANEL_W - 8, 56), fill=(255, 255, 255))
    draw.text((14, 14), title, fill=(25, 25, 25))
    draw.text((14, 34), subtitle, fill=(70, 70, 70))
    draw.text((PANEL_W - 94, 14), "g down", fill=(40, 40, 40))

    return image


def save_pendulum_animation(t, q_mj):
    clear_png_frames(IMG_DIR)
    if PENDULUM_GIF_PATH.exists():
        PENDULUM_GIF_PATH.unlink()

    t_frame = np.arange(0.0, float(t[-1]) + 0.5 * FRAME_DT, FRAME_DT)

    q1 = np.interp(t_frame, t, q_mj[:, 0])
    q2_rel = np.interp(t_frame, t, q_mj[:, 1])
    theta1 = q1
    theta2_abs = q1 + q2_rel

    for i, tt in enumerate(t_frame):
        panel = draw_single_pendulum_panel(
            theta1[i],
            theta2_abs[i],
            title="MuJoCo Double Pendulum",
            subtitle=f"t = {tt:5.2f} s",
        )
        panel.save(IMG_DIR / f"{i:06d}.png")

    create_gif(IMG_DIR, PENDULUM_GIF_PATH, FRAME_DT)


def save_comparison_plot(t, q_mj, q_ode, show_plot=False):
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    axes[0].plot(t, np.rad2deg(q_mj[:, 0]), label="MuJoCo", linewidth=1.6)
    axes[0].plot(t, np.rad2deg(q_ode[:, 0]), "--", label="solve_ivp", linewidth=1.6)
    axes[0].set_ylabel("q1 [deg]")
    axes[0].legend()

    axes[1].plot(t, np.rad2deg(q_mj[:, 1]), label="MuJoCo", linewidth=1.6)
    axes[1].plot(t, np.rad2deg(q_ode[:, 1]), "--", label="solve_ivp", linewidth=1.6)
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("q2 (relative) [deg]")
    axes[1].legend()

    fig.suptitle("Double Pendulum: MuJoCo vs solve_ivp")
    fig.tight_layout()
    fig.savefig(PLOT_PATH, dpi=160)

    if show_plot:
        plt.show()
    plt.close(fig)


def save_overlay_animation(t, q_mj, q_ode):
    clear_png_frames(OVERLAY_IMG_DIR)
    if OVERLAY_GIF_PATH.exists():
        OVERLAY_GIF_PATH.unlink()

    t_frame = np.arange(0.0, float(t[-1]) + 0.5 * OVERLAY_FRAME_DT, OVERLAY_FRAME_DT)
    q1_mj = np.interp(t_frame, t, q_mj[:, 0])
    q2_mj = np.interp(t_frame, t, q_mj[:, 1])
    q1_ode = np.interp(t_frame, t, q_ode[:, 0])
    q2_ode = np.interp(t_frame, t, q_ode[:, 1])

    th1_mj = q1_mj
    th2_mj = q1_mj + q2_mj
    th1_ode = q1_ode
    th2_ode = q1_ode + q2_ode

    x1_mj, z1_mj, x2_mj, z2_mj = endpoint_positions(th1_mj, th2_mj)
    x1_ode, z1_ode, x2_ode, z2_ode = endpoint_positions(th1_ode, th2_ode)

    limit = 2.15 * (L1 + L2)

    for i, tt in enumerate(t_frame):
        fig, ax = plt.subplots(figsize=(6.4, 6.0))
        ax.set_aspect("equal")
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit * 0.55)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("z [m]")
        ax.set_title(f"Double Pendulum Overlay (position space)  t = {tt:5.2f} s")

        # Pivot
        ax.plot(0.0, 0.0, "ko", ms=6)

        # Trajectory tails for second bob
        k0 = max(0, i - 80)
        ax.plot(x2_mj[k0:i + 1], z2_mj[k0:i + 1], color="tab:orange", alpha=0.35, lw=1.5)
        ax.plot(x2_ode[k0:i + 1], z2_ode[k0:i + 1], color="tab:red", alpha=0.35, lw=1.5, ls="--")

        # MuJoCo links and masses
        ax.plot([0, x1_mj[i]], [0, z1_mj[i]], color="tab:blue", lw=2.8, label="MuJoCo link1")
        ax.plot([x1_mj[i], x2_mj[i]], [z1_mj[i], z2_mj[i]], color="tab:orange", lw=2.8, label="MuJoCo link2")
        ax.plot(x1_mj[i], z1_mj[i], "o", color="tab:blue", ms=7)
        ax.plot(x2_mj[i], z2_mj[i], "o", color="tab:orange", ms=7)

        # ODE links and masses
        ax.plot([0, x1_ode[i]], [0, z1_ode[i]], color="tab:green", lw=2.0, ls="--", label="solve_ivp link1")
        ax.plot([x1_ode[i], x2_ode[i]], [z1_ode[i], z2_ode[i]], color="tab:red", lw=2.0, ls="--", label="solve_ivp link2")
        ax.plot(x1_ode[i], z1_ode[i], "s", color="tab:green", ms=5)
        ax.plot(x2_ode[i], z2_ode[i], "s", color="tab:red", ms=5)

        ax.text(0.03, 0.96, "g down", transform=ax.transAxes, va="top")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.2)

        frame_path = OVERLAY_IMG_DIR / f"{i:06d}.png"
        fig.tight_layout()
        fig.savefig(frame_path, dpi=120)
        plt.close(fig)

    create_gif(OVERLAY_IMG_DIR, OVERLAY_GIF_PATH, OVERLAY_FRAME_DT)


def main():
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))

    t = np.arange(0.0, T_END + 0.5 * SAMPLE_DT, SAMPLE_DT)
    q0 = np.array([Q1_0, Q2_0], dtype=float)
    dq0 = np.array([DQ1_0, DQ2_0], dtype=float)

    q_mj, _dq_mj = run_mujoco_case(model, q0, dq0, t)

    y0 = np.array([Q1_0, Q2_0, DQ1_0, DQ2_0], dtype=float)
    q_ode, _dq_ode = run_ode(t, y0)

    save_pendulum_animation(t, q_mj)
    save_overlay_animation(t, q_mj, q_ode)
    save_comparison_plot(t, q_mj, q_ode, show_plot=SHOW_PLOT)

    q1_rmse = rmse_deg(q_mj[:, 0], q_ode[:, 0])
    q2_rmse = rmse_deg(q_mj[:, 1], q_ode[:, 1])

    print(f"Saved: {PENDULUM_GIF_PATH}")
    print(f"Saved: {OVERLAY_GIF_PATH}")
    print(f"Saved: {PLOT_PATH}")
    print(f"RMSE q1 [deg]: {q1_rmse:.4f}")
    print(f"RMSE q2 [deg]: {q2_rmse:.4f}")

    if q1_rmse >= THETA1_RMSE_TARGET_DEG or q2_rmse >= THETA2_REL_RMSE_TARGET_DEG:
        print(
            "Warning: short-horizon RMSE target exceeded "
            f"(q1 < {THETA1_RMSE_TARGET_DEG:.1f} deg, q2 < {THETA2_REL_RMSE_TARGET_DEG:.1f} deg)."
        )


if __name__ == "__main__":
    main()
