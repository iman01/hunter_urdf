# Validation

Closed-loop results from Gazebo Harmonic (DART, 1 ms step) with the default model (`variant:=2020`, trapezoid steering linkage), produced by `tools/sim_tests/drive_test.py`. Raw reports: [`validation/gazebo_flat.json`](validation/gazebo_flat.json), [`validation/gazebo_offroad.json`](validation/gazebo_offroad.json). Expected values come from the kinematics module and `vehicle.yaml`, so these tests check that the simulated vehicle behaves like its own design.

## Flat ground (`flat.sdf`)

| Test | Expected | Measured |
|---|---|---|
| Front / rear axle heave at kerb mass | 0 / 0 mm | +2.3 / +2.0 mm |
| Front / rear axle roll at rest | 0 deg | +0.005 / -0.005 deg |
| Straight line at 8 m/s: speed | 8.00 m/s | 7.983 m/s |
| Straight line: wheel odometry vs ground truth over 79.8 m | 0 % | +0.29 % |
| Straight line: lateral drift over 79.8 m (open loop) | 0 m | 0.33 m |
| 4WS counter-phase circle, 2 m/s: radius | 8.000 m | 8.007 m (+0.09 %) |
| 4WS counter-phase circle, 2 m/s: yaw rate | 0.2500 rad/s | 0.2491 rad/s |
| 2WS front-steer circle, 2 m/s: radius | 13.402 m | 13.467 m (+0.49 %) |
| 2WS front-steer circle, 2 m/s: yaw rate | 0.1500 rad/s | 0.1490 rad/s |
| 4WS full lock (vehicle centre path): radius | 2.774 m | 2.741 m (-1.20 %) |
| 4WS full lock (vehicle centre path): yaw rate | 0.5407 rad/s | 0.5328 rad/s |
| 2WS full lock (vehicle centre path): radius | 5.710 m | 5.574 m (-2.39 %) |
| 2WS full lock (vehicle centre path): yaw rate | 0.2703 rad/s | 0.2733 rad/s |
| Crab 1.5 + 0.6 m/s: track angle | 21.8 deg | 23.3 deg |
| Crab: heading change over 10 s | 0 deg | -0.22 deg |

Design turning radius at the outer tyre: 3.92 m (4WS), 7.00 m (2WS).

Notes: the full-lock circles come out 1 to 2 % tighter than kinematics because the rigid tyres scrub. The crab track-angle offset comes from the mechanical trapezoid, which cannot hold both wheels of an axle parallel (`linkage:=ideal` gives exactly parallel wheels). The straight-line drift (0.4 % of distance) is an open-loop figure: the 4WS controller has no heading feedback, so a path follower or Nav2 controller removes it.

## Off-road course (`offroad.sdf`, 2 m/s, counter-phase mode)

| Section | Result |
|---|---|
| Articulation twister, 0.30 m humps alternating left/right | passed; axle roll 7.95 / 7.95 deg (limit 7.94), heave -104 .. 101 mm, body roll 6.7 deg |
| 0.30 m step | passed; max pitch 8.3 deg |
| 30 % grade (16.7 deg) | climbed at 2.00 m/s; body pitch 17.7 deg (slope 16.7 deg + suspension squat) |
| Course completed | True (final x 66.0 m) |

The 0.30 m humps exceed the 250 mm stroke, so the axles reach the articulation limit and the body takes the rest as roll. This is the expected behaviour of the published suspension.

## Configurations

| Check | Expected | Measured |
|---|---|---|
| Armed variant (`variant:=2023 payload:=rws`, +550 kg): front / rear sag | 21.8 / 21.2 mm | 24.5 / 23.5 mm |
| RWS position controller: yaw 1.0 rad, pitch 0.3 rad | 1.0 / 0.3 rad | 1.0 / 0.3 rad |
| Sensors (rendered under Xvfb, CPU): lidar clouds 16384 points, 1280x720 images, NavSat, IMU | published | published, frame ids = optical / sensor frames |

Sensor images from the simulation (front driving camera and PTZ looking at the off-road course):

![front driving camera](images/sim_front_drive_camera.png)

![PTZ camera](images/sim_ptz_camera.png)

## Static tests (`colcon test`)

* `hunter_description/test/test_urdf.py`: 7 configurations; `check_urdf`; total mass within 1 % of the published kerb mass (3500 kg; 4050 kg armed; + payload); every inertia positive definite and satisfying the triangle inequality; every mesh present; ros2_control joints exist; heave travel equals the published 250 mm stroke; published envelope (4.70 x 2.20 m, 2.70 m wheelbase, 1 m wheels); design outputs up to date.
* `hunter_control/test/test_kinematics.py`: 25 tests of the 4WS kinematics (straight line, reverse, counter-phase symmetry, Ackermann normals meeting at the instantaneous centre, crab, forward/inverse round trip in every mode, steering limits, trapezoid symmetry and Ackermann error, locked differential, turning radius).
