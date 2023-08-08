use crate::PowerIo;
use rust_decimal::prelude::ToPrimitive;
use rust_decimal::Decimal;
use rust_decimal_macros::dec;
use serde::{Deserialize, Serialize};
use sky130pdk::corner::Sky130Corner;
use sky130pdk::mos::{Nfet01v8, Pfet01v8};
use sky130pdk::Sky130CommercialPdk;
use spectre::blocks::{Pulse, Vsource};
use spectre::tran::{Tran, TranTime, TranVoltage};
use spectre::{Options, Spectre};
use substrate::arcstr::{self, ArcStr};
use substrate::block::Block;
use substrate::io::Io;
use substrate::io::{Input, Node, Output, SchematicType, Signal, TestbenchIo};
use substrate::pdk::corner::Pvt;
use substrate::schematic::{Cell, CellBuilder, HasSchematic, HasSchematicData, SimCellBuilder};
use substrate::simulation::data::FromSaved;
use substrate::simulation::waveform::{EdgeDir, TimeWaveform, WaveformRef};
use substrate::simulation::{HasSimSchematic, SimController, Testbench};

#[cfg(test)]
mod tests;

#[derive(Debug, Default, Copy, Clone, Io)]
pub struct VcoIo {
    pub tune: Input<Signal>,
    pub out: Output<Signal>,
    pub pwr: PowerIo,
}

#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq, Serialize, Deserialize, Block)]
#[substrate(io = "VcoIo")]
pub struct Vco {
    pub fmin: Decimal,
    pub fmax: Decimal,
    pub jitter: Decimal,
    pub maxcap: Decimal,
    pub tr: Decimal,
    pub tf: Decimal,
}

#[derive(Debug, Default, Copy, Clone, Io)]
pub struct DelayCellIo {
    pub tune: Input<Signal>,
    pub input: Input<Signal>,
    pub output: Output<Signal>,
    pub pwr: PowerIo,
}

#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq, Serialize, Deserialize, Block)]
#[substrate(io = "DelayCellIo")]
pub struct CurrentStarvedInverter;

impl HasSchematicData for CurrentStarvedInverter {
    type Data = ();
}

impl HasSchematic<Sky130CommercialPdk> for CurrentStarvedInverter {
    fn schematic(
        &self,
        io: &<<Self as Block>::Io as SchematicType>::Bundle,
        cell: &mut CellBuilder<Sky130CommercialPdk, Self>,
    ) -> substrate::error::Result<Self::Data> {
        let virtual_vss = cell.signal("virtual_vss", Signal);

        let nmos = cell.instantiate(Nfet01v8::new((1_200, 150)));
        cell.connect(nmos.io().d, io.output);
        cell.connect(nmos.io().g, io.input);
        cell.connect(nmos.io().s, virtual_vss);
        cell.connect(nmos.io().b, io.pwr.vss);

        let ngate = cell.instantiate(Nfet01v8::new((3_000, 150)));
        cell.connect(ngate.io().d, virtual_vss);
        cell.connect(ngate.io().g, io.tune);
        cell.connect(ngate.io().s, io.pwr.vss);
        cell.connect(ngate.io().b, io.pwr.vss);

        let pmos = cell.instantiate(Pfet01v8::new((2_400, 150)));
        cell.connect(pmos.io().d, io.output);
        cell.connect(pmos.io().g, io.input);
        cell.connect(pmos.io().s, io.pwr.vdd);
        cell.connect(pmos.io().b, io.pwr.vdd);

        Ok(())
    }
}

#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq, Serialize, Deserialize)]
pub struct DelayCellTb<T> {
    pub dut: T,
    pub pvt: Pvt<Sky130Corner>,
    pub tr: Decimal,
    pub tf: Decimal,
}

impl<T: Block> Block for DelayCellTb<T> {
    type Io = TestbenchIo;
    fn id() -> ArcStr {
        arcstr::literal!("delay_cell_tb")
    }
    fn name(&self) -> ArcStr {
        arcstr::literal!("delay_cell_tb")
    }
    fn io(&self) -> Self::Io {
        Default::default()
    }
}

impl<T: Block> HasSchematicData for DelayCellTb<T>
where
    DelayCellTb<T>: Block,
{
    type Data = Node;
}

impl<T: Block + HasSchematic<Sky130CommercialPdk>> HasSimSchematic<Sky130CommercialPdk, Spectre>
    for DelayCellTb<T>
where
    T: Block<Io = DelayCellIo> + Clone,
{
    fn schematic(
        &self,
        io: &<<Self as Block>::Io as SchematicType>::Bundle,
        cell: &mut SimCellBuilder<Sky130CommercialPdk, Spectre, Self>,
    ) -> substrate::error::Result<Self::Data> {
        let dut = cell.instantiate(self.dut.clone());

        let vdd = cell.instantiate_tb(Vsource::dc(self.pvt.voltage));
        cell.connect(vdd.io().p, dut.io().pwr.vdd);
        cell.connect(vdd.io().n, io.vss);
        cell.connect(io.vss, dut.io().pwr.vss);

        let vtune = cell.instantiate_tb(Vsource::dc(dec!(0.0)));
        cell.connect(vtune.io().p, dut.io().tune);
        cell.connect(vtune.io().n, io.vss);

        let vin = Vsource::pulse(Pulse {
            val0: dec!(0),
            val1: self.pvt.voltage,
            period: Some(dec!(1e6)),
            rise: Some(self.tr),
            fall: Some(self.tf),
            width: Some(dec!(1e-9)),
            delay: Some(dec!(1e-9)),
        });

        let vin = cell.instantiate_tb(vin);
        cell.connect(vin.io().p, dut.io().input);
        cell.connect(vin.io().n, io.vss);

        Ok(dut.io().output)
    }
}

impl<T: Block> substrate::simulation::data::Save<Spectre, Tran, &Cell<DelayCellTb<T>>>
    for DelayCellTbSaved
{
    fn save(
        ctx: &substrate::simulation::SimulationContext,
        cell: &Cell<DelayCellTb<T>>,
        opts: &mut <Spectre as substrate::simulation::Simulator>::Options,
    ) -> Self::Key {
        Self::Key {
            time: TranTime::save(ctx, cell, opts),
            vout: TranVoltage::save(ctx, cell.data(), opts),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, FromSaved)]
pub struct DelayCellTbSaved {
    time: TranTime,
    vout: TranVoltage,
}

#[derive(Debug, Clone, Serialize, Deserialize, FromSaved)]
pub struct DelayCellTbOutput {
    td_hl: f64,
    td_lh: f64,
}

impl<T: Block + HasSchematic<Sky130CommercialPdk>> Testbench<Sky130CommercialPdk, Spectre>
    for DelayCellTb<T>
where
    T: Block<Io = DelayCellIo> + Clone,
{
    type Output = DelayCellTbOutput;
    fn run(&self, sim: SimController<Sky130CommercialPdk, Spectre, Self>) -> Self::Output {
        let wavs: DelayCellTbSaved = sim
            .simulate(
                Options::default(),
                Some(&Sky130Corner::Tt),
                Tran {
                    stop: dec!(3e-9),
                    errpreset: Some(spectre::ErrPreset::Conservative),
                    ..Default::default()
                },
            )
            .expect("failed to run simulation");
        let wav = WaveformRef::new(&wavs.time, &wavs.vout);
        let mut edges = wav.edges(0.5 * self.pvt.voltage.to_f64().unwrap());
        let falling = edges.next().unwrap();
        assert_eq!(falling.dir(), EdgeDir::Falling);
        let rising = edges.next().unwrap();
        assert_eq!(rising.dir(), EdgeDir::Rising);

        let td_hl = falling.t() - 1e-9 - self.tr.to_f64().unwrap() / 2.0;
        let td_lh = rising.t() - 2e-9 - self.tf.to_f64().unwrap() / 2.0;

        DelayCellTbOutput { td_hl, td_lh }
    }
}
