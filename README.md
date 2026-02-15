# mujoco-examples

This repository collects MuJoCo examples.  
The goal is not only visualization, but also validating simulation behavior against analytical theory.

## Quick Start

### 1. Activate the virtual environment

```bash
source venv/bin/activate
```

### 2. Install dependencies

```bash
pip install mujoco pillow numpy matplotlib
```

### 3. Run the example

```bash
cd src/examples/1dof_spring_mass
python mujoco_1dof_sim.py
```

## Repository Layout

Examples in this repository follow these rules:

- `src/examples/<example_name>/`
- `<example_name>` uses lowercase `snake_case`
- Keep simulation scripts and MJCF files in the same folder

For details, see each example document under `src/examples/<example_name>/Example_*.md`.

## Examples

### 1DOF Spring-Mass-Damper

- Path: `src/examples/1dof_spring_mass/`
- Detailed doc: [`src/examples/1dof_spring_mass/Example_1DOF_Spring-Mass-Damper.md`](src/examples/1dof_spring_mass/Example_1DOF_Spring-Mass-Damper.md)
- Includes:
  - MuJoCo simulation
  - Theory comparison
  - GIF export
  - Matplotlib comparison plot export

Direct link: [1DOF Spring-Mass-Damper Guide](src/examples/1dof_spring_mass/Example_1DOF_Spring-Mass-Damper.md)

## Outputs

### Animation (GIF)

![animation](src/examples/1dof_spring_mass/animation.gif)

### Theory vs MuJoCo

![theory-vs-mujoco](src/examples/1dof_spring_mass/theory_vs_mujoco.png)

## Notes

- Depending on your environment, renderer initialization can fail due to OpenGL/GUI context constraints.
- If that happens, run the example on a local machine with a valid GUI session.
