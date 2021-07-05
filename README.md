[![crates.io](https://img.shields.io/crates/v/s_curve.svg)](https://crates.io/crates/s_curve)
![GitHub Workflow Status](https://img.shields.io/github/workflow/status/marcbone/s_curve/Rust)
[![crates.io](https://img.shields.io/crates/l/s_curve.svg)](https://crates.io/crates/s_curve)
[![crates.io](https://img.shields.io/crates/d/s_curve.svg)](https://crates.io/crates/s_curve)
[![docs.rs](https://docs.rs/s_curve/badge.svg)](https://docs.rs/s_curve)





A library to compute S-Curve trajectories. It can be used to generate motion profiles for robotics.

## What is an S-Curve?

An S-Curve is a trajectory which is constrained to maximum jerk, acceleration and velocity.
An S-Curve consists of 7 phases:
 * constant maximum jerk until the desired acceleration is reached
 * constant maximum acceleration phase
 * constant minimum jerk until the desired velocity is reached with an acceleration of zero
 * constant velocity phase
 * constant minimum jerk until the minimum acceleration is reached
 * constant minimum acceleration phase
 * constant maximum jerk until the acceleration is zero and the desired position and end velocity is reached
 
 In the picture below you can see an S-Curve Profile which goes from Position 0 to position 10 within 5.5 seconds with a start and end velocity of 0
, a maximum jerk of 3, a maximum acceleration of 2 and a maximum velocity of 3.
[![image](http://i.imgur.com/BQPhS8n.png)](http://i.imgur.com/BQPhS8n.png)
## Example
  ```rust
  use s_curve::*;
  let constraints = SCurveConstraints {
              max_jerk: 3.,
              max_acceleration: 2.0,
              max_velocity: 3.
  };
  let  start_conditions = SCurveStartConditions {
      q0: 0., // start position
      q1: 10., // end position
      v0: 0., // start velocity
      v1: 0. // end velocity
  };
let input  =  SCurveInput{constraints, start_conditions};
let (params, s_curve) = s_curve_generator( &input,Derivative::Velocity);
for i in 0..101 {
      println!("{}", s_curve(i as f64 * params.time_intervals.total_duration() / 100.));
  }
  ```
## no-std support

To use scurve in an environment without the standard library you can set ```default_features = false``` for the crate.
Then you have to use the eval_{position,velocity,acceleration,jerk} directly.


#### License
Copyright (c) 2020 Marco Boneberger
Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option. 

