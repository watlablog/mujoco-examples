from pathlib import Path
import mujoco
import numpy as np
from PIL import Image, ImageDraw
from scipy.integrate import solve_ivp

import matplotlib.pyplot as plt

BASE_DIR = Path(__file__).resolve().parent
XML_PATH = BASE_DIR / "gyroscopic_rotor_fixed_pivot.xml"
PLOT_PATH = BASE_DIR / "comparison_plot.png"
GIF_PATH = BASE_DIR / "comparison_animation.gif"
IMG_DIR = BASE_DIR / "img_compare"
IMG_DIR.mkdir(exist_ok=True)

JOINT_NAME = "pivot"
CAM_OBLIQUE = "cam_oblique"
CAM_TOP = "cam_top"

# ODE/MJCF shared parameters (must match gyroscopic_rotor_fixed_pivot.xml)
MASS = 1.0
GRAVITY = 9.81
COM_OFFSET = 0.20
I1_COM = 0.02
I3_COM = 0.01
# For fixed-pivot dynamics, transverse inertia must be about the pivot:
# I_pivot = I_com + m*l^2 (parallel-axis theorem).
I1_PIVOT = I1_COM + MASS * (COM_OFFSET**2)
I3_PIVOT = I3_COM
I_DIAG_PIVOT = np.array([I1_PIVOT, I1_PIVOT, I3_PIVOT], dtype=float)

# Initial condition
THETA0_DEG = 20.0
OMEGA_S0 = 200.0
OMEGA_P0 = 0.0

# Simulation settings
T_END = 10.0
SAMPLE_DT = 0.01
FRAME_DT = 0.04
VIEW_W = 240
VIEW_H = 360
SHOW_PLOT = True
THETA_RMSE_TARGET_DEG = 1.0
PHI_RMSE_TARGET_DEG = 5.0


def quat_normalize(q):
    n = np.linalg.norm(q)
    if n <= 0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def quat_from_x_rotation(theta_rad):
    half = 0.5 * theta_rad
    return np.array([np.cos(half), np.sin(half), 0.0, 0.0], dtype=float)


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )


def rotmat_from_quat(q):
    q = quat_normalize(q)
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rhs(_t, y):
    q = quat_normalize(y[0:4])
    omega_body = y[4:7]

    r_cm_body = np.array([0.0, 0.0, COM_OFFSET], dtype=float)
    r_world_from_body = rotmat_from_quat(q)

    r_cm_world = r_world_from_body @ r_cm_body
    f_gravity_world = np.array([0.0, 0.0, -MASS * GRAVITY], dtype=float)
    tau_world = np.cross(r_cm_world, f_gravity_world)
    tau_body = r_world_from_body.T @ tau_world

    iomega = I_DIAG_PIVOT * omega_body
    omega_dot = (tau_body - np.cross(omega_body, iomega)) / I_DIAG_PIVOT

    omega_quat = np.array([0.0, omega_body[0], omega_body[1], omega_body[2]], dtype=float)
    q_dot = 0.5 * quat_multiply(q, omega_quat)

    return np.concatenate((q_dot, omega_dot))


def extract_theta_phi_from_quats(q_array):
    axes = []
    for q in q_array:
        r = rotmat_from_quat(q)
        axes.append(r[:, 2])

    axes = np.array(axes, dtype=float)
    theta = np.arccos(np.clip(axes[:, 2], -1.0, 1.0))
    phi = np.unwrap(np.arctan2(axes[:, 1], axes[:, 0]))
    return theta, phi


def build_initial_omega_body(q0, omega_s0, omega_p0):
    # Base spin is defined about body symmetry axis.
    omega_body = np.array([0.0, 0.0, omega_s0], dtype=float)
    # If a world-z precession component is requested, map it to body coordinates
    # so MuJoCo and ODE share the exact same initial angular velocity definition.
    if omega_p0 != 0.0:
        r_world_from_body = rotmat_from_quat(q0)
        omega_body += r_world_from_body.T @ np.array([0.0, 0.0, omega_p0], dtype=float)
    return omega_body


def rmse_deg(x, y):
    return float(np.sqrt(np.mean(np.rad2deg(x - y) ** 2)))


def clear_png_frames(in_dir):
    for p in in_dir.glob("*.png"):
        p.unlink()


def annotate_frame(frame_rgb, left_title="Oblique", right_title="Top"):
    image = Image.fromarray(frame_rgb)
    draw = ImageDraw.Draw(image)

    draw.rectangle((8, 8, 130, 30), fill=(0, 0, 0))
    draw.text((14, 12), left_title, fill=(255, 255, 255))

    x_offset = VIEW_W
    draw.rectangle((x_offset + 8, 8, x_offset + 110, 30), fill=(0, 0, 0))
    draw.text((x_offset + 14, 12), right_title, fill=(255, 255, 255))

    return np.array(image)


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


def run_mujoco(t_eval, q0, omega_s0, omega_p0, save_animation=True):
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)

    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, JOINT_NAME)
    qpos_adr = model.jnt_qposadr[jid]
    qvel_adr = model.jnt_dofadr[jid]

    mujoco.mj_resetData(model, data)

    data.qpos[qpos_adr:qpos_adr + 4] = q0

    omega_body = build_initial_omega_body(q0, omega_s0, omega_p0)
    data.qvel[qvel_adr:qvel_adr + 3] = omega_body

    mujoco.mj_forward(model, data)

    dt = model.opt.timestep
    sample_every = max(1, int(round(SAMPLE_DT / dt)))
    frame_every = max(1, int(round(FRAME_DT / dt)))
    total_steps = int(round(T_END / dt))

    renderer = None
    frame_idx = 0
    if save_animation:
        clear_png_frames(IMG_DIR)
        if GIF_PATH.exists():
            GIF_PATH.unlink()
        try:
            renderer = mujoco.Renderer(model, width=VIEW_W, height=VIEW_H)
        except Exception as e:
            save_animation = False
            print(f"Warning: renderer unavailable, GIF export skipped ({e})")

    q_log = []
    for step in range(total_steps + 1):
        if step % sample_every == 0:
            q_now = quat_normalize(np.array(data.qpos[qpos_adr:qpos_adr + 4], dtype=float))
            q_log.append(q_now)

        if renderer is not None and step % frame_every == 0:
            renderer.update_scene(data, camera=CAM_OBLIQUE)
            rgb_left = renderer.render().copy()

            renderer.update_scene(data, camera=CAM_TOP)
            rgb_right = renderer.render().copy()

            combined = np.concatenate((rgb_left, rgb_right), axis=1)
            combined = annotate_frame(combined)
            Image.fromarray(combined).save(IMG_DIR / f"{frame_idx:06d}.png")
            frame_idx += 1

        if step < total_steps:
            mujoco.mj_step(model, data)

    if renderer is not None and save_animation:
        renderer.close()
        create_gif(IMG_DIR, GIF_PATH, FRAME_DT)

    q_log = np.array(q_log, dtype=float)
    theta, phi = extract_theta_phi_from_quats(q_log)
    return theta, phi


def run_ode(t_eval, q0, omega_s0, omega_p0):
    omega_body0 = build_initial_omega_body(q0, omega_s0, omega_p0)
    y0 = np.concatenate((q0, omega_body0))

    sol = solve_ivp(
        rhs,
        (float(t_eval[0]), float(t_eval[-1])),
        y0,
        t_eval=t_eval,
        rtol=1e-8,
        atol=1e-10,
        method="RK45",
    )
    if not sol.success:
        raise RuntimeError(f"solve_ivp failed: {sol.message}")

    q_array = sol.y[0:4, :].T
    q_array = np.array([quat_normalize(q) for q in q_array], dtype=float)

    theta, phi = extract_theta_phi_from_quats(q_array)
    return theta, phi


def save_comparison_plot(t, theta_mj, phi_mj, theta_ode, phi_ode, show_plot=False):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax1.plot(t, np.rad2deg(theta_mj), label="MuJoCo", linewidth=1.6)
    ax1.plot(t, np.rad2deg(theta_ode), label="solve_ivp", linestyle="--", linewidth=1.6)
    ax1.set_ylabel("theta [deg]")
    ax1.set_title("Fixed-Pivot Gyroscopic Top: MuJoCo vs Numerical ODE")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, np.rad2deg(phi_mj), label="MuJoCo", linewidth=1.6)
    ax2.plot(t, np.rad2deg(phi_ode), label="solve_ivp", linestyle="--", linewidth=1.6)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("phi [deg]")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(PLOT_PATH, dpi=150)
    if show_plot:
        plt.show()
    plt.close(fig)


def main():
    dt = SAMPLE_DT
    t_eval = np.arange(0.0, T_END + 0.5 * dt, dt)

    q0 = quat_from_x_rotation(np.deg2rad(THETA0_DEG))

    theta_mj, phi_mj = run_mujoco(t_eval, q0, OMEGA_S0, OMEGA_P0, save_animation=True)
    theta_ode, phi_ode = run_ode(t_eval, q0, OMEGA_S0, OMEGA_P0)

    save_comparison_plot(t_eval, theta_mj, phi_mj, theta_ode, phi_ode, show_plot=SHOW_PLOT)
    theta_err = rmse_deg(theta_mj, theta_ode)
    phi_err = rmse_deg(phi_mj, phi_ode)
    print(f"Saved: {PLOT_PATH}")
    if GIF_PATH.exists():
        print(f"Saved: {GIF_PATH}")
    print(f"RMSE theta [deg]: {theta_err:.4f}")
    print(f"RMSE phi   [deg]: {phi_err:.4f}")
    if theta_err >= THETA_RMSE_TARGET_DEG or phi_err >= PHI_RMSE_TARGET_DEG:
        print(
            "Warning: RMSE thresholds not met "
            f"(theta < {THETA_RMSE_TARGET_DEG:.1f} deg, phi < {PHI_RMSE_TARGET_DEG:.1f} deg)."
        )


if __name__ == "__main__":
    main()
