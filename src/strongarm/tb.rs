use approx::abs_diff_eq;
use rust_decimal::prelude::ToPrimitive;
use rust_decimal::Decimal;
use rust_decimal_macros::dec;
use serde::{Deserialize, Serialize};
use sky130pdk::corner::Sky130Corner;
use sky130pdk::Sky130Pdk;
use spectre::analysis::tran::Tran;
use spectre::blocks::{Pulse, Vsource};
use spectre::{ErrPreset, Spectre};
use substrate::block::Block;
use substrate::io::schematic::{Bundle, HardwareType, Node};
use substrate::io::{DiffPair, TestbenchIo};
use substrate::pdk::corner::Pvt;
use substrate::schematic::{Cell, CellBuilder, ExportsNestedData, NestedData, Schematic};
use substrate::simulation::data::{tran, FromSaved, Save, SaveTb};
use substrate::simulation::{SimController, SimulationContext, Simulator, Testbench};

use crate::strongarm::ClockedDiffComparatorIo;

#[derive(Serialize, Deserialize, Block, Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[substrate(io = "TestbenchIo")]
pub struct StrongArmTranTb<T> {
    pub dut: T,
    pub vinp: Decimal,
    pub vinn: Decimal,
    pub pvt: Pvt<Sky130Corner>,
}

#[derive(Clone, Debug, Hash, PartialEq, Eq, NestedData)]
pub struct StrongArmTranTbNodes {
    vop: Node,
    von: Node,
    vinn: Node,
    vinp: Node,
    clk: Node,
}

trait Dut: Block<Io = ClockedDiffComparatorIo> + Schematic<Sky130Pdk> + Clone {}
impl<T: Block<Io = ClockedDiffComparatorIo> + Schematic<Sky130Pdk> + Clone> Dut for T {}

impl<T: Dut> ExportsNestedData for StrongArmTranTb<T> {
    type NestedData = StrongArmTranTbNodes;
}

impl<T: Dut> Schematic<Spectre> for StrongArmTranTb<T> {
    fn schematic(
        &self,
        io: &<<Self as Block>::Io as HardwareType>::Bundle,
        cell: &mut CellBuilder<Spectre>,
    ) -> substrate::error::Result<Self::NestedData> {
        let dut = cell
            .sub_builder::<Sky130Pdk>()
            .instantiate(self.dut.clone());

        let vinp = cell.instantiate(Vsource::dc(self.vinp));
        let vinn = cell.instantiate(Vsource::dc(self.vinn));
        let vdd = cell.instantiate(Vsource::dc(self.pvt.voltage));
        let vclk = cell.instantiate(Vsource::pulse(Pulse {
            val0: dec!(0),
            val1: self.pvt.voltage,
            period: Some(dec!(1000)),
            width: Some(dec!(100)),
            delay: Some(dec!(10e-9)),
            rise: Some(dec!(100e-12)),
            fall: Some(dec!(100e-12)),
        }));

        cell.connect(io.vss, vinp.io().n);
        cell.connect(io.vss, vinn.io().n);
        cell.connect(io.vss, vdd.io().n);
        cell.connect(io.vss, vclk.io().n);

        let output = cell.signal("output", DiffPair::default());

        cell.connect(
            Bundle::<ClockedDiffComparatorIo> {
                input: Bundle::<DiffPair> {
                    p: *vinp.io().p,
                    n: *vinn.io().p,
                },
                output: output.clone(),
                clock: *vclk.io().p,
                vdd: *vdd.io().p,
                vss: io.vss,
            },
            dut.io(),
        );

        Ok(StrongArmTranTbNodes {
            vop: output.p,
            von: output.n,
            vinn: *vinn.io().p,
            vinp: *vinp.io().p,
            clk: *vclk.io().p,
        })
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, FromSaved)]
pub struct ComparatorSim {
    t: tran::Time,
    vop: tran::Voltage,
    von: tran::Voltage,
    vinn: tran::Voltage,
    vinp: tran::Voltage,
    clk: tran::Voltage,
}

/// The decision made by a comparator.
#[derive(Copy, Clone, Hash, Eq, PartialEq, Ord, PartialOrd, Debug, Serialize, Deserialize)]
pub enum ComparatorDecision {
    /// Negative.
    ///
    /// The negative input was larger than the positive input.
    Neg,
    /// Positive.
    ///
    /// The positive input was larger than the negative input.
    Pos,
}

impl<T: Dut> SaveTb<Spectre, Tran, ComparatorSim> for StrongArmTranTb<T> {
    fn save_tb(
        ctx: &SimulationContext<Spectre>,
        cell: &Cell<Self>,
        opts: &mut <Spectre as Simulator>::Options,
    ) -> <ComparatorSim as FromSaved<Spectre, Tran>>::SavedKey {
        ComparatorSimSavedKey {
            t: tran::Time::save(ctx, (), opts),
            vop: tran::Voltage::save(ctx, cell.data().vop, opts),
            von: tran::Voltage::save(ctx, cell.data().von, opts),
            vinn: tran::Voltage::save(ctx, cell.data().vinn, opts),
            vinp: tran::Voltage::save(ctx, cell.data().vinp, opts),
            clk: tran::Voltage::save(ctx, cell.data().clk, opts),
        }
    }
}

impl<T: Dut> Testbench<Spectre> for StrongArmTranTb<T> {
    type Output = Option<ComparatorDecision>;

    fn run(&self, sim: SimController<Spectre, Self>) -> Self::Output {
        let mut opts = spectre::Options::default();
        sim.set_option(self.pvt.corner, &mut opts);
        let wav: ComparatorSim = sim
            .simulate(
                opts,
                Tran {
                    stop: dec!(30e-9),
                    start: None,
                    errpreset: Some(ErrPreset::Conservative),
                },
            )
            .expect("failed to run simulation");

        let von = *wav.von.last().unwrap();
        let vop = *wav.vop.last().unwrap();

        let vdd = self.pvt.voltage.to_f64().unwrap();
        if abs_diff_eq!(von, 0.0, epsilon = 1e-4) && abs_diff_eq!(vop, vdd, epsilon = 1e-4) {
            Some(ComparatorDecision::Pos)
        } else if abs_diff_eq!(von, vdd, epsilon = 1e-4) && abs_diff_eq!(vop, 0.0, epsilon = 1e-4) {
            Some(ComparatorDecision::Neg)
        } else {
            None
        }
    }
}
