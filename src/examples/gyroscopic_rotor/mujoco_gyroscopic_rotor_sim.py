from pathlib import Path
import os
import glob
import mujoco
import numpy as np
from PIL import Image, ImageDraw

BASE_DIR = Path(__file__).resolve().parent

XML_PATH = BASE_DIR / "gyroscopic_rotor.xml"
JOINT_NAME = "free"
BODY_NAME = "rotor"
TIP_RADIUS = 0.014
TIP_CLEARANCE = 0.001

# Tuned for visible wobble (nutation) with constant spin hold.
TILT_DEG = 5.0
TARGET_SPIN_RAD_S = 80.0
HOLD_SPIN_CONSTANT = True

T_END = 10.0
FRAME_DT = 0.05

VIEW_W = 240
VIEW_H = 360
CAM_OBLIQUE = "cam_oblique"
CAM_TOP = "cam_top"

IMG_DIR = BASE_DIR / "img"
IMG_DIR.mkdir(exist_ok=True)
GIF_PATH = BASE_DIR / "animation.gif"


def create_gif(in_dir, out_filename, frame_dt_s, loop=0):
    """Create a GIF from sequential images."""

    path_list = sorted(glob.glob(os.path.join(in_dir, "*.png")))
    if len(path_list) == 0:
        raise FileNotFoundError(f"No images found in: {in_dir}")

    # Quantize frames to an adaptive palette to reduce GIF file size.
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


def quat_from_x_rotation(theta_rad):
    """Return quaternion [w, x, y, z] for rotation about x-axis."""

    half = 0.5 * theta_rad
    return np.array([np.cos(half), np.sin(half), 0.0, 0.0], dtype=float)


def rotmat_from_quat(q):
    """Convert quaternion [w, x, y, z] to 3x3 rotation matrix."""

    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def enforce_constant_body_spin(data, angvel_adr, body_id, target_spin):
    """Keep spin component about body z-axis constant."""

    rot = np.array(data.xmat[body_id]).reshape(3, 3)
    axis_world = rot[:, 2]

    w = np.array(data.qvel[angvel_adr:angvel_adr + 3], dtype=float)
    spin_now = float(np.dot(w, axis_world))
    w_perp = w - spin_now * axis_world
    data.qvel[angvel_adr:angvel_adr + 3] = w_perp + target_spin * axis_world


def annotate_frame(frame_rgb, left_title="Oblique", right_title="Top"):
    """Overlay simple labels on dual-view frame."""

    image = Image.fromarray(frame_rgb)
    draw = ImageDraw.Draw(image)

    draw.rectangle((8, 8, 130, 30), fill=(0, 0, 0))
    draw.text((14, 12), left_title, fill=(255, 255, 255))

    x_offset = VIEW_W
    draw.rectangle((x_offset + 8, 8, x_offset + 110, 30), fill=(0, 0, 0))
    draw.text((x_offset + 14, 12), right_title, fill=(255, 255, 255))

    return np.array(image)


for p in IMG_DIR.glob("*.png"):
    p.unlink()

model = mujoco.MjModel.from_xml_path(str(XML_PATH))
data = mujoco.MjData(model)

jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, JOINT_NAME)
qpos_adr = model.jnt_qposadr[jid]
qvel_adr = model.jnt_dofadr[jid]
bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BODY_NAME)
angvel_adr = qvel_adr + 3

mujoco.mj_resetData(model, data)

tilt_rad = np.deg2rad(TILT_DEG)
q_tilt = quat_from_x_rotation(tilt_rad)
data.qpos[qpos_adr:qpos_adr + 3] = np.array([0.0, 0.0, TIP_RADIUS + TIP_CLEARANCE], dtype=float)
data.qpos[qpos_adr + 3:qpos_adr + 7] = q_tilt

r_world_from_body = rotmat_from_quat(q_tilt)
spin_body = np.array([0.0, 0.0, TARGET_SPIN_RAD_S], dtype=float)
spin_world = r_world_from_body @ spin_body
data.qvel[angvel_adr:angvel_adr + 3] = spin_world

mujoco.mj_forward(model, data)
if HOLD_SPIN_CONSTANT:
    enforce_constant_body_spin(data, angvel_adr, bid, TARGET_SPIN_RAD_S)
    mujoco.mj_forward(model, data)

renderer = mujoco.Renderer(model, width=VIEW_W, height=VIEW_H)

dt = model.opt.timestep
steps = int(T_END / dt) + 1
save_every = max(1, int(round(FRAME_DT / dt)))

frame_idx = 0
for i in range(steps):
    if i > 0:
        mujoco.mj_step(model, data)
        if HOLD_SPIN_CONSTANT:
            enforce_constant_body_spin(data, angvel_adr, bid, TARGET_SPIN_RAD_S)
            mujoco.mj_forward(model, data)

    if i % save_every == 0:
        renderer.update_scene(data, camera=CAM_OBLIQUE)
        rgb_left = renderer.render().copy()

        renderer.update_scene(data, camera=CAM_TOP)
        rgb_right = renderer.render().copy()

        combined = np.concatenate((rgb_left, rgb_right), axis=1)
        combined = annotate_frame(combined)

        Image.fromarray(combined).save(IMG_DIR / f"{frame_idx:06d}.png")
        frame_idx += 1

renderer.close()

create_gif(in_dir=str(IMG_DIR), out_filename=str(GIF_PATH), frame_dt_s=FRAME_DT)
