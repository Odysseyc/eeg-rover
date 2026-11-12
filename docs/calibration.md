# Calibration

All values live in `rover/config/rover.yaml`. Edit, then restart `rover.launch` (no rebuild needed).
Do this with the rover on a box first, then on the floor.

| Check | How | If wrong |
|---|---|---|
| Steering direction | Stick left (or keyboard Left) turns the front wheels left | flip `steer_sign` (-1 ↔ 1) |
| Straight line | Drive forward with no steering; does it drift? | adjust `servo_center` (85 by default) a degree at a time |
| Wheel angle | At full lock, measure the front-wheel angle | set `max_steer` (rad); adjust `servo_deg_per_rad` so full lock is reached at full stick |
| Wheelbase | Measure rear axle to front axle | set `wheelbase` (m); match `--wheelbase` in the Xbox script |
| Speed | Command 0.5 m/s, compare with `/rover/speed` and a tape measure + stopwatch | scale `erpm_per_mps` |
| Reverse | Hold the left trigger: the rover should back up as readily as it drives forward | if it stalls at low speed, raise `min_erpm` slightly |

`/rover/speed` comes from the motor's RPM, so it is only as accurate as `erpm_per_mps`.
