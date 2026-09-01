# SCONE procedural terrains

Terrains are generated as MuJoCo primitive geoms when `load_model()` runs.
They are not copied into `src/assets/model.xml`: the robot MJCF remains the
single robot description, while terrain parameters remain readable and
versionable Python data in this folder.

## Files

- `types.py`: public terrain names and validated stair/slope data classes.
- `presets.py`: all dimensions and difficulty values in SI units.
- `generator.py`: uneven-tile, variable-step stair, ramp, and mixed-course
  MJCF generation algorithms.
- `__init__.py`: stable terrain package exports.

## Presets

| Name | Definition |
| --- | --- |
| `flat` | Original infinite plane only |
| `uneven` | 1.8 × 1.2 m, 0.2 m tiles, 8–60 mm height, ±4° tilt |
| `stairs-1` | rises 35/45/55 mm; 135 mm total |
| `stairs-2` | rises 55/70/85 mm; 210 mm total |
| `stairs-3` | rises 80/100/120 mm; 300 mm total |
| `slope-1` | 8°, 1.4 m long, 0.9 m wide |
| `slope-2` | 15°, 1.2 m long, 1.0 m wide |
| `slope-3` | 25°, 1.0 m long, 1.1 m wide |
| `mixed` | all seven non-flat families, alternating stairs and slopes |

Each entry in a `StairProfile` has its own incremental rise, tread depth, and
width. `TerrainGenerator.add_stairs()` accumulates those rises into absolute
top heights and creates one solid box per step. `return_to_floor=True` mirrors
the sequence after a landing, which lets the mixed course return to the base
plane between sections.

`TerrainGenerator.add_slope()` derives rise as `length * sin(angle)`, places
the ramp so its top surface starts at the current floor, adds a landing, and
can generate a matching decline. The mixed course uses this return path.

The uneven generator is deterministic for a given seed. This is useful for
repeating evaluation on the same geometry while allowing randomized training
by changing `terrain_seed`.

For side-on stair ascent with SCONE's C-shaped end frames, use the
simulation-only adaptive controller:

```bash
mjpython -m src.simulation --control scone-stair --terrain stairs-3
```

The controller rolls continuously on easy stairs and activates an alternating
tripod hook assist for a known tall rise or a detected stall. Its geometric
assumptions and measured comparisons are documented in
[`docs/11-scone-stair-climbing.md`](../../../docs/11-scone-stair-climbing.md).
