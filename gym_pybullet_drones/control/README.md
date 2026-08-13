# Control interfaces

This directory contains the controller implementations used by the examples and environments in `gym-pybullet-drones`.

## `DSLPIDControl` and `target_rpy_rates`

`DSLPIDControl` is a cascaded **position and attitude** controller for the Crazyflie models. Its `target_rpy_rates` argument should not be interpreted as a direct body-rate/ACRO command interface.

### Units

`target_rpy_rates` is expressed in **radians per second (`rad/s`)**.

The attitude controller obtains the current roll, pitch, and yaw from PyBullet with `getEulerFromQuaternion()` and estimates their rates with a finite difference:

```python
(cur_rpy - self.last_rpy) / control_timestep
```

The controller then computes

```python
rpy_rates_e = target_rpy_rates - (cur_rpy - self.last_rpy) / control_timestep
```

and applies that error through the derivative coefficient of the attitude PID.

Because PyBullet's Euler angles are in radians and `control_timestep` is in seconds, both terms in `rpy_rates_e` are in `rad/s`.

### What the argument does

`target_rpy_rates` acts as the rate reference used by the **derivative term of the attitude controller**. It does not replace the attitude loop, and passing nonzero values does not turn `DSLPIDControl` into a body-rate controller.

The commanded attitude is still determined by `target_euler`, and the proportional/integral attitude terms remain active.

For example, a desired yaw Euler-rate contribution of 30 degrees per second should be passed as:

```python
target_rpy_rates = np.array([0.0, 0.0, np.deg2rad(30.0)])
```

### Euler rates are not body rates

The finite-difference term above is computed from roll, pitch, and yaw Euler angles. These Euler-angle rates are not, in general, identical to the vehicle's body angular rates `(p, q, r)`, especially away from small roll and pitch angles.

`DSLPIDControl.computeControl()` also does not use its `cur_ang_vel` argument. Users who need a true body-rate controller should use an interface designed for rate control rather than treating `target_rpy_rates` as one.

This distinction is particularly important for reinforcement-learning action spaces: an action intended to represent body rates should not be wired directly into `DSLPIDControl.target_rpy_rates` under the assumption that the controller is operating in ACRO/rate mode.

## Related implementations

- `DSLPIDControl.py`: cascaded Crazyflie position/attitude PID controller.
- `CTBRControl.py`: controller support used by the collective-thrust/body-rate path.
- Betaflight SITL examples: appropriate when the experiment specifically requires a flight-stack rate-control interface.

This documentation describes the existing controller behavior; it does not change controller gains, numerical behavior, or simulation dynamics.
