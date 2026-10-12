# legacy/ — old code kept for reference only

Nothing in here is installed by `colcon build` and no node uses it.
It is the first version of the kinematics, before the closed-form solver
(`QuadPkg/kinematics/leg_kinematics.py`) replaced it.

| Folder / file | What it was |
|---|---|
| `pinocchio_ik/fk.py`, `ik.py` | Numeric FK/IK using Pinocchio (heavy, seed-dependent) |
| `pinocchio_ik/parser.py` | URDF → `robot_descriptor.yaml` (needs `pinocchio`) |
| `pinocchio_ik/robot_descriptor.yaml` | Output of the parser. **Its `urdf_path` points to the CAD-zero URDF on the old machine (`/home/nikunj/...`)** — edit it to `../../urdf/QuadURDFcad4_0_cadzero.urdf` before running anything here |
| `pinocchio_ik/test_fk.py`, `test_ik.py`, `verify_ik_sim.py` | Old manual checks of the above |
| `isaac/verify_fk.py` | Isaac Script Editor snippet that printed a foot position |
| `isaac/quad4_test_setup.usd` | Old scene. It references a robot USD from a previous folder layout, so it will open with a missing robot |

Safe to delete once you no longer need it.
