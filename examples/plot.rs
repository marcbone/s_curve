//Copyright (c) 2020 Marco Boneberger
use gnuplot::{Figure, Caption, Color, AxesCommon};
use gnuplot::Coordinate::Graph;
use s_curve::*;

fn main() {
    let constraints = SCurveConstraints {
        max_jerk: 3.,
        max_acceleration: 2.0,
        max_velocity: 3.};
    let  start_conditions = SCurveStartConditions {
        q0: 0.,
        q1: 10.,
        v0: 0.,
        v1: 0.
    };
    let input  =  SCurveInput {constraints, start_conditions};
    let s_curve_tmp = s_curve_generator(&input,Derivative::Velocity);
    let s_curve = s_curve_tmp.1;
    let params =s_curve_tmp.0;
    let mut x :Vec<f64> = Vec::new();
    let mut y: Vec<f64> = Vec::new();
    for i in 0..1001 {
        x.push(i as f64 * params.time_intervals.total_duration() / 1000.);
        y.push(s_curve(i as f64 * params.time_intervals.total_duration() / 1000.));
    }
    let mut fg = Figure::new();
    fg.axes2d()
        .set_title("S-Curve Velocity Motion Profile", &[])
        .set_legend(Graph(0.5), Graph(0.9), &[], &[])
        .set_x_label("time in seconds", &[])
        .set_y_label("velocity in m/s", &[])
        .lines(
            x.clone(),
            y.clone(),
            &[Caption("Velocity")],
        );
    fg.show().unwrap();
}

