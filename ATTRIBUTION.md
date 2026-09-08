# Attribution and dependencies

- [SciPy 1.13.1](https://github.com/scipy/scipy/tree/v1.13.1), BSD-3-Clause:
  differential evolution optimizer. Installed as a dependency, not vendored.
- [Matplotlib 3.9.4](https://github.com/matplotlib/matplotlib/tree/v3.9.4),
  Matplotlib license (PSF-based): plotting. Installed as a dependency, not vendored.
- NumPy and other transitive dependencies are recorded in requirements-lock.txt;
  their upstream licenses apply to their packages.
- [University of Michigan CTMS motor speed model](https://ctms.engin.umich.edu/CTMS/?example=MotorSpeed&section=SystemModeling):
  reference for the standard physical model and educational parameter values.
  Equations are implemented independently; no tutorial code or images are included.
- [SciPy differential evolution documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.differential_evolution.html):
  optimizer API reference. The pinned version is selected for compatibility with
  the local Python environment; it is not represented as the newest release.

The MIT license in this repository applies to original project material only.
Preserve third-party licenses if dependencies or upstream code are later vendored.
