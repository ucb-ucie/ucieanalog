use rust_decimal_macros::dec;
use sky130pdk::corner::Sky130Corner;
use substrate::pdk::corner::Pvt;

use crate::paths::get_path;
use crate::sky130_commercial_ctx;
use crate::vco::{DelayCellTuningRange, RingOscillator, VcoTb};

use super::{CurrentStarvedInverter, DelayCellTb};

#[test]
fn current_starved_inverter_delay() {
    let ctx = sky130_commercial_ctx();
    let output = ctx
        .simulate(
            DelayCellTb {
                dut: CurrentStarvedInverter,
                pvt: Pvt::new(Sky130Corner::Tt, dec!(1.8), dec!(25)),
                vtune: dec!(1.8),
                tr: dec!(2e-12),
                tf: dec!(2e-12),
            },
            get_path("current_starved_inverter_delay", "sim/"),
        )
        .expect("failed to run simulation");

    println!("Output: {output:?}");
}

#[test]
fn current_starved_inverter_delay_range() {
    let test_name = "current_starved_inverter_delay_range";
    let ctx = sky130_commercial_ctx();
    let handle = ctx.cache.get_with_state(
        test_name,
        DelayCellTuningRange {
            dut: CurrentStarvedInverter,
            pvt: Pvt::new(Sky130Corner::Tt, dec!(1.8), dec!(25)),
            vtune_min: dec!(0.6),
            vtune_max: dec!(1.8),
            num_points: 41,
            tr: dec!(2e-12),
            tf: dec!(2e-12),
            work_dir: get_path(test_name, "sims"),
        },
        ctx.clone(),
    );
    let output = handle.get().as_ref().unwrap();

    println!("Output: {output:?}");
}

#[test]
fn current_starved_ro_period() {
    let test_name = "current_starved_ro_period";
    let ctx = sky130_commercial_ctx();
    let output = ctx
        .simulate(
            VcoTb {
                vco: RingOscillator::new(7, CurrentStarvedInverter),
                pvt: Pvt::new(Sky130Corner::Tt, dec!(1.8), dec!(25)),
                vtune: dec!(1.8),
                sim_time: dec!(50e-9),
                c_load: dec!(0.5e-15),
            },
            get_path(test_name, "sim/"),
        )
        .unwrap();

    println!("Output: {output:?}");
}
