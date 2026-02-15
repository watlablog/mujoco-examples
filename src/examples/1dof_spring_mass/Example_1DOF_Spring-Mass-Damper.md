# 1DOF Spring-Mass-Damper Example

## Purpose

This example simulates a 1DOF spring-mass-damper system in MuJoCo and compares the result with an analytical solution.  
You can inspect all of the following at once:

- Time-series simulation
- Animation GIF
- Overlaid MuJoCo and theory displacement plot

## Model Parameters

The parameters in this example are:

- `m = 1.0` kg
- `k = 20.0` N/m
- `c = 0.5` N*s/m
- Initial displacement `X0 = 1.0` m
- Initial velocity `V0 = 0.0` m/s

## Joint Structure

This model uses a minimal setup with one body named `mass` attached to `world`.  
The `mass` body has a slider joint `x` and can translate only along the x-axis.

- Parent-child: `world` -> `body(name="mass")`
- Joint: `joint(name="x", type="slide", axis="1 0 0")`
- Restoring force and damping: `stiffness="20"`, `damping="0.5"`
- Mass: `geom(type="sphere", mass="1")`

So the system has exactly one degree of freedom (x displacement), behaving as a 1DOF spring-mass-damper oscillator.

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

## Equation of Motion

$$
m\ddot{x}(t) + c\dot{x}(t) + kx(t) = 0
$$

## Definitions

$$
\omega_n = \sqrt{\frac{k}{m}}
$$

$$
\zeta = \frac{c}{2\sqrt{mk}}
$$

$$
\omega_d = \omega_n\sqrt{1-\zeta^2}
$$

## Analytical Forms Used in Code

The following forms are used in `calc_theory_displacement()`.

$$
x(0)=X_0,\quad \dot{x}(0)=V_0
$$

$$
n_v=\frac{V_0}{X_0\omega_n}
$$

### Underdamped

$$
\zeta<1,\quad
x(t)=e^{-\sigma t}X\cos(\omega_d t-\phi)
$$

$$
X=\sqrt{X_0^2+\left(\frac{V_0+\sigma X_0}{\omega_d}\right)^2},\quad
\phi=\arctan\!\left(\frac{V_0+\sigma X_0}{X_0\omega_d}\right)
$$

### Critically Damped

$$
\zeta=1,\quad
x(t)=X_0e^{-\omega_n t}\left((n_v+1)\omega_n t+1\right)
$$

### Overdamped

$$
\zeta>1,\quad
x(t)=X_0e^{-\zeta\omega_n t}\left[
\cosh\!\left(\omega_n t\sqrt{\zeta^2-1}\right)
+\frac{n_v+\zeta}{\sqrt{\zeta^2+1}}
\sinh\!\left(\omega_n t\sqrt{\zeta^2-1}\right)
\right]
$$

## Damping Regimes

### Underdamped

$$
\zeta < 1
$$

### Critically Damped

$$
\zeta = 1
$$

### Overdamped

$$
\zeta > 1
$$

## Mapping to Implementation

The analytical solution is computed in `calc_theory_displacement()` inside `mujoco_1dof_sim.py`.

- `t_log`: simulation time array
- `x_log`: MuJoCo displacement log
- `theory`: displacement computed from analytical equations

The code branches match:

- `zeta < 1`: underdamped equation
- `np.isclose(zeta, 1.0)`: critically damped equation
- otherwise: overdamped equation

The comparison plot is saved in `save_comparison_plot()` and shown at the end.

## How to Run

```bash
cd src/examples/1dof_spring_mass
../../../venv/bin/python mujoco_1dof_sim.py
```

## Generated Files

- `animation.gif`: MuJoCo-rendered animation
- `theory_vs_mujoco.png`: theory vs MuJoCo comparison plot
- `img/*.png`: frame sequence used to build the GIF

## Output Preview

### Animation

![animation](animation.gif)

### Theory vs MuJoCo

![theory-vs-mujoco](theory_vs_mujoco.png)

## How to Read the Result

- The oscillation amplitude decays over time due to damping.
- MuJoCo and analytical waveforms should largely agree.
- Changing initial conditions shifts phase and amplitude evolution.

## Notes

- To avoid GitHub math rendering issues, equations are written as standalone blocks, not inside headings or nested lists.
- Depending on your environment, renderer initialization can fail due to graphics backend constraints.

---

Back to: [Main README](../../../README.md)
