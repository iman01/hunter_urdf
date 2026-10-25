# Changelog

## Hull reshaped to the photographed vehicle (2026-10-25)

The previous hull was a reconstruction without photos (open deck with railing,
grille noses). It now follows the closed, faceted body of the HUNTeR as
photographed in 2023-2025, measured against the 1 m wheel.

- Hull: flat deck at 1.46 m, upper side flanks leaning ~20 deg inward, ~41 deg
  front glacis, full-width black sensor band (0.80-1.07 m) with 2+2 LED lamps
  and camera windows, chin with name plate, tow bars with bow shackles,
  angular wheel arches with anthracite trims, trapezoidal tub with louvres,
  dark stepped rear deck with a steep rear face above an olive rear panel,
  bolt rows, grab handles, deck-corner outriggers, three whip antennas.
- New meshes: `hull_black`, `hull_trim`, `hull_plates`, `hull_glass`,
  `hull_shackles`, `hull_shackle_pins`, `hull_tiedowns`; new materials
  `hunter_green_light`, `hull_black`, `trim_anthracite`, `pin_blue`,
  `tiedown_orange`; body paint and rims recoloured to match the photos.
- Collision model: 16 boxes following the new hull; boxes may now carry a
  pitch (7th value) for the glacis and rear plates.
- Sensors moved onto the new hull (no railing any more): front driving camera
  in the sensor band, side cameras in the flanks, top-view cameras on the
  front-left outrigger and a rear deck post, lidars on short deck posts.
- PTZ camera removed (mast, pan/tilt joints, Gazebo camera, `ptz_controller`,
  bridge topics, meshes): it was the 2020 fit and is not on the photographed
  vehicles.
- Derived: approach / departure angle now 25 / 30 deg (was 39 deg), set by
  the tow bars.
- New `tools/check_clearance.py` (also run by the URDF tests): hull meshes vs
  tyres, wheels, axles and knuckles over full steering lock and suspension
  travel. No clashes.
- Unchanged: running gear, suspension, steering, drivetrain, mass budget and
  inertias (so the Gazebo dynamics validation still applies; hull-terrain
  contact on the off-road course was not re-run).
