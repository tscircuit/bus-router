# Repository Instructions

## Tests and SVG snapshots

- Every test must assert a clear SVG snapshot of the behavior or output under test.
- Prefer a focused output visualization over a dump of unrelated solver search state.
- Use the `toMatchGraphicsSvg(import.meta.path)` matcher and commit the generated snapshot with the test.
- Expected-failure reproductions must remain active tests and snapshot the failure state clearly; do not skip them solely because the solver currently fails.
- Visually inspect every new or updated snapshot. Keep labels legible and snapshot files compact enough to review in a pull request.
