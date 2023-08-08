use rust_decimal_macros::dec;
use sky130pdk::corner::Sky130Corner;
use substrate::pdk::corner::Pvt;

use crate::paths::get_path;
use crate::sky130_commercial_ctx;

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
