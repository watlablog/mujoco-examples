# Double Pendulum Chaos Example

## Purpose

This example compares a 2-link pendulum simulated by MuJoCo and `solve_ivp`.

You can observe:

- MuJoCo single-run pendulum animation
- MuJoCo vs numerical ODE overlay in position space (`x-z`)
- Static time-series comparison plot (`q1`, `q2`)

## Model Summary

- Model file: `double_pendulum.xml`
- Joint structure:
  - `hinge1`: first link rotation
  - `hinge2`: second link relative rotation
- Gravity: `0 0 -9.81`
- Integration timestep (MuJoCo): `0.001 s`

The MJCF uses a point-mass approximation:

- `m1 = 1.0`, `m2 = 1.0`
- `l1 = 1.0`, `l2 = 1.0`
- Mass is concentrated near each link tip (visual rods are massless)

## MJCF (double_pendulum.xml)

```xml
<mujoco model="double_pendulum">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

  <worldbody>
    <body name="link1" pos="0 0 0">
      <joint name="hinge1" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -1" mass="1.0" diaginertia="1e-6 1e-6 1e-6"/>

      <body name="link2" pos="0 0 -1">
        <joint name="hinge2" type="hinge" axis="0 1 0" damping="0"/>
        <inertial pos="0 0 -1" mass="1.0" diaginertia="1e-6 1e-6 1e-6"/>
      </body>
    </body>
  </worldbody>
</mujoco>
```

## Equations of Motion

Let generalized coordinates be `q = [q1, q2]`, where `q2` is the relative joint angle.

$$
M(q)\ddot{q} + C(q,\dot{q}) + G(q) = 0
$$

with

$$
M_{11} = (m_1+m_2)l_1^2 + m_2 l_2^2 + 2m_2l_1l_2\cos q_2
$$

$$
M_{12} = M_{21} = m_2 l_2^2 + m_2l_1l_2\cos q_2
$$

$$
M_{22} = m_2 l_2^2
$$

$$
C_1 = -m_2l_1l_2\sin q_2\,(2\dot{q}_1\dot{q}_2 + \dot{q}_2^2)
$$

$$
C_2 = m_2l_1l_2\sin q_2\,\dot{q}_1^2
$$

$$
G_1 = (m_1+m_2)gl_1\sin q_1 + m_2gl_2\sin(q_1+q_2)
$$

$$
G_2 = m_2gl_2\sin(q_1+q_2)
$$

## Initial Conditions

Base case:

- `q1(0)=2.35 rad`
- `q2(0)=1.10 rad`
- `dq1(0)=0.80 rad/s`
- `dq2(0)=-0.60 rad/s`

These values are intentionally energetic so the pendulum motion is more dynamic.

## What Is Compared

- MuJoCo vs `solve_ivp` for `q1(t)`
- MuJoCo vs `solve_ivp` for `q2(t)` (relative angle)
- Position-space overlay animation using endpoint coordinates

## Run

```bash
cd src/examples/double_pendulum
../../../venv/bin/python mujoco_double_pendulum_compare.py
```

## Generated Files

- `animation.gif`: MuJoCo-only pendulum animation
- `comparison_overlay.gif`: animated overlay of MuJoCo and `solve_ivp` pendulum poses in `x-z` space
- `comparison_plot.png`: static overlay plot (`q1`, `q2`)
- `img/*.png`: frame sequence used to build `animation.gif`
- `img_overlay/*.png`: frame sequence used to build `comparison_overlay.gif`

Direct links:

- [MuJoCo animation (`animation.gif`)](animation.gif)
- [Position-space overlay (`comparison_overlay.gif`)](comparison_overlay.gif)
- [Time-series plot (`comparison_plot.png`)](comparison_plot.png)

## Output Preview

### Animation

![double-pendulum-animation](animation.gif)

### MuJoCo vs solve_ivp Overlay (GIF)

![double-pendulum-overlay](comparison_overlay.gif)

### MuJoCo vs solve_ivp Time-Series (PNG)

![double-pendulum-plot](comparison_plot.png)

## Why Results Drift Over Time

Even when the short-time match is good, MuJoCo and `solve_ivp` can separate at later times.
Main reasons are:

- chaotic sensitivity to tiny state differences (double pendulum is a nonlinear chaotic system)
- model mismatch (MuJoCo rigid-body model vs simplified ODE assumptions)
- integration differences (fixed-step RK4 in MuJoCo config vs adaptive-step RK45 in `solve_ivp`)

So this example should be read as:

- short horizon: waveform agreement check
- long horizon: trend/qualitative behavior check

## Notes

- Minor long-horizon mismatch can still appear from integration differences.
- Use short-horizon agreement and trend consistency as primary checks.

---

Back to: [Main README](../../../README.md)
