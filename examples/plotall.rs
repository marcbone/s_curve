//Copyright (c) 2020 Marco Boneberger
use gnuplot::Coordinate::Graph;
use gnuplot::{AxesCommon, Caption, Figure};
use s_curve::*;

fn main() {
    let constraints = SCurveConstraints {
        max_jerk: 3.,
        max_acceleration: 2.0,
        max_velocity: 3.,
    };
    let start_conditions = SCurveStartConditions {
        q0: 5.,
        q1: 0.,
        v0: 0.,
        v1: 0.,
    };
    let input = SCurveInput {
        constraints,
        start_conditions,
    };
    // jerk
    let (paramsjer, s_curvejer) = s_curve_generator(&input, Derivative::Jerk);
    let mut xjer: Vec<f64> = Vec::new();
    let mut yjer: Vec<f64> = Vec::new();
    for i in 0..1001 {
        xjer.push(i as f64 * paramsjer.time_intervals.total_duration() / 1000.);
        yjer.push(s_curvejer(
            i as f64 * paramsjer.time_intervals.total_duration() / 1000.,
        ));
    }
    // pos
    let (paramspos, s_curvepos) = s_curve_generator(&input, Derivative::Position);
    let mut xpos: Vec<f64> = Vec::new();
    let mut ypos: Vec<f64> = Vec::new();
    for i in 0..1001 {
        xpos.push(i as f64 * paramspos.time_intervals.total_duration() / 1000.);
        ypos.push(s_curvepos(
            i as f64 * paramspos.time_intervals.total_duration() / 1000.,
        ));
    }
    // vel
    let (paramsvel, s_curvevel) = s_curve_generator(&input, Derivative::Velocity);
    let mut xvel: Vec<f64> = Vec::new();
    let mut yvel: Vec<f64> = Vec::new();
    for i in 0..1001 {
        xvel.push(i as f64 * paramsvel.time_intervals.total_duration() / 1000.);
        yvel.push(s_curvevel(
            i as f64 * paramsvel.time_intervals.total_duration() / 1000.,
        ));
    }
    // acc
    let (paramsacc, s_curveacc) = s_curve_generator(&input, Derivative::Acceleration);
    let mut xacc: Vec<f64> = Vec::new();
    let mut yacc: Vec<f64> = Vec::new();
    for i in 0..1001 {
        xacc.push(i as f64 * paramsacc.time_intervals.total_duration() / 1000.);
        yacc.push(s_curveacc(
            i as f64 * paramsacc.time_intervals.total_duration() / 1000.,
        ));
    }
    let mut fg = Figure::new();
    fg.axes2d()
        .set_title("S-Curve Velocity Motion Profile", &[])
        .set_legend(Graph(0.5), Graph(0.9), &[], &[])
        .set_x_label("time in seconds", &[])
        .set_y_label("Position derivatives m, m/s, m/s², m/s³", &[])
        .lines(xpos.clone(), ypos.clone(), &[Caption("Position")])
        .lines(xvel.clone(), yvel.clone(), &[Caption("Velocity")])
        .lines(xacc.clone(), yacc.clone(), &[Caption("Acceleration")])
        .lines(xjer.clone(), yjer.clone(), &[Caption("Jerk")]);
    fg.show().unwrap();
}
