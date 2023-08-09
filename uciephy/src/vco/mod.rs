use std::path::PathBuf;

use crate::PowerIo;
use cache::CacheableWithState;
use itertools::Itertools;
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
use substrate::context::Context;
use substrate::io::{Array, Io};
use substrate::io::{Input, Node, Output, SchematicType, Signal, TestbenchIo};
use substrate::pdk::corner::Pvt;
use substrate::pdk::Pdk;
use substrate::schematic::primitives::Capacitor;
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

pub trait Vco: Block<Io = VcoIo> {}
impl<T: Block<Io = VcoIo>> Vco for T {}

#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq, Serialize, Deserialize, Block)]
#[substrate(io = "VcoIo")]
pub struct VcoParams {
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
    pub vtune: Decimal,
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

        let vtune = cell.instantiate_tb(Vsource::dc(self.vtune));
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

impl<T: Block> substrate::simulation::data::Save<Spectre, Tran, &Cell<DelayCellTb<T>>> for Vout {
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

impl<V: Vco> substrate::simulation::data::Save<Spectre, Tran, &Cell<VcoTb<V>>> for Vout {
    fn save(
        ctx: &substrate::simulation::SimulationContext,
        cell: &Cell<VcoTb<V>>,
        opts: &mut <Spectre as substrate::simulation::Simulator>::Options,
    ) -> Self::Key {
        Self::Key {
            time: TranTime::save(ctx, cell, opts),
            vout: TranVoltage::save(ctx, cell.data(), opts),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, FromSaved)]
pub struct Vout {
    time: TranTime,
    vout: TranVoltage,
}

impl Vout {
    /// Returns the output voltage waveform as a [`WaveformRef`].
    #[inline]
    pub fn as_waveform(&self) -> WaveformRef {
        WaveformRef::new(&self.time, &self.vout)
    }
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
        let wavs: Vout = sim
            .simulate(
                Options::default(),
                Some(&self.pvt.corner),
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

#[derive(Debug, Clone, Hash, Eq, PartialEq, Serialize, Deserialize)]
pub struct DelayCellTuningRange<T> {
    pub dut: T,
    pub pvt: Pvt<Sky130Corner>,
    pub vtune_min: Decimal,
    pub vtune_max: Decimal,
    pub num_points: usize,
    pub tr: Decimal,
    pub tf: Decimal,
    pub work_dir: PathBuf,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TuningRange {
    td_hl_min: f64,
    td_hl_max: f64,
    td_lh_min: f64,
    td_lh_max: f64,
}

impl<T, PDK: Pdk> CacheableWithState<Context<PDK>> for DelayCellTuningRange<T>
where
    DelayCellTb<T>: Testbench<PDK, Spectre, Output = DelayCellTbOutput>,
    T: Clone + Block,
{
    type Output = TuningRange;
    type Error = ();
    fn generate_with_state(
        &self,
        ctx: Context<PDK>,
    ) -> std::result::Result<Self::Output, Self::Error> {
        assert!(self.num_points > 1);
        let incr: Decimal = (self.vtune_max - self.vtune_min) / Decimal::from(self.num_points - 1);
        let outputs = (0..self.num_points)
            .map(|i| {
                let work_dir = self.work_dir.join(format!("sim{i}/"));
                ctx.simulate(
                    DelayCellTb {
                        dut: self.dut.clone(),
                        pvt: self.pvt,
                        vtune: self.vtune_min + Decimal::from(i) * incr,
                        tr: self.tr,
                        tf: self.tf,
                    },
                    work_dir,
                )
                .unwrap()
            })
            .collect::<Vec<_>>();
        let (td_hl_min, td_hl_max) = outputs
            .iter()
            .map(|o| o.td_hl)
            .minmax()
            .into_option()
            .unwrap();
        let (td_lh_min, td_lh_max) = outputs
            .iter()
            .map(|o| o.td_lh)
            .minmax()
            .into_option()
            .unwrap();
        Ok(Self::Output {
            td_hl_min,
            td_hl_max,
            td_lh_min,
            td_lh_max,
        })
    }
}

/// A ring oscillator.
#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq, Serialize, Deserialize, Block)]
#[substrate(io = "VcoIo")]
pub struct RingOscillator<E> {
    stages: usize,
    element: E,
}

impl<E> RingOscillator<E> {
    /// Create a new ring oscillator by repeating the given element.
    ///
    /// Stages must be an odd integer.
    #[inline]
    pub fn new(stages: usize, element: E) -> Self {
        assert_eq!(
            stages % 2,
            1,
            "ring oscillator must have an odd number of stages"
        );
        Self { stages, element }
    }
}

impl<E: Block> HasSchematicData for RingOscillator<E> {
    type Data = ();
}

impl<E, PDK> HasSchematic<PDK> for RingOscillator<E>
where
    E: Clone + HasSchematic<PDK> + Block<Io = DelayCellIo>,
    PDK: Pdk,
{
    fn schematic(
        &self,
        io: &<<Self as Block>::Io as SchematicType>::Bundle,
        cell: &mut CellBuilder<PDK, Self>,
    ) -> substrate::error::Result<Self::Data> {
        let nodes = cell.signal("nodes", Array::new(self.stages, Signal));
        cell.connect(nodes[self.stages - 1], io.out);

        for i in 0..self.stages {
            let input = if i == 0 {
                nodes[self.stages - 1]
            } else {
                nodes[i - 1]
            };
            let output = nodes[i];
            let elt = cell.instantiate(self.element.clone());
            cell.connect(&elt.io().pwr, &io.pwr);
            cell.connect(elt.io().input, input);
            cell.connect(elt.io().output, output);
            cell.connect(elt.io().tune, io.tune);
        }
        Ok(())
    }
}

#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq, Serialize, Deserialize, Block)]
#[substrate(io = "TestbenchIo")]
pub struct VcoTb<V> {
    pub vco: V,
    pub pvt: Pvt<Sky130Corner>,
    pub vtune: Decimal,
    pub sim_time: Decimal,
    pub c_load: Decimal,
}

impl<V: Vco> HasSchematicData for VcoTb<V> {
    type Data = Node;
}

impl<V, PDK> HasSimSchematic<PDK, Spectre> for VcoTb<V>
where
    V: Vco + Clone + HasSchematic<PDK>,
    PDK: Pdk,
{
    fn schematic(
        &self,
        io: &<<Self as Block>::Io as SchematicType>::Bundle,
        cell: &mut SimCellBuilder<PDK, Spectre, Self>,
    ) -> substrate::error::Result<Self::Data> {
        let dut = cell.instantiate(self.vco.clone());

        let vdd = cell.instantiate_tb(Vsource::dc(self.pvt.voltage));
        cell.connect(vdd.io().p, dut.io().pwr.vdd);
        cell.connect(vdd.io().n, io.vss);
        cell.connect(io.vss, dut.io().pwr.vss);

        let vtune = cell.instantiate_tb(Vsource::dc(self.vtune));
        cell.connect(vtune.io().p, dut.io().tune);
        cell.connect(vtune.io().n, io.vss);

        let c_load = cell.instantiate(Capacitor::new(self.c_load));
        cell.connect(c_load.io().p, dut.io().out);
        cell.connect(c_load.io().n, io.vss);

        Ok(dut.io().out)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, FromSaved)]
pub struct VcoTbOutput {
    period: f64,
}

impl VcoTbOutput {
    /// The period of the VCO output.
    #[inline]
    pub fn period(&self) -> f64 {
        self.period
    }
    /// The frequency of the VCO output.
    #[inline]
    pub fn freq(&self) -> f64 {
        1f64 / self.period
    }
}

impl<V> Testbench<Sky130CommercialPdk, Spectre> for VcoTb<V>
where
    V: Vco + Clone + HasSchematic<Sky130CommercialPdk>,
{
    type Output = VcoTbOutput;
    fn run(&self, sim: SimController<Sky130CommercialPdk, Spectre, Self>) -> Self::Output {
        let wavs: Vout = sim
            .simulate(
                Options::default(),
                Some(&self.pvt.corner),
                Tran {
                    stop: self.sim_time,
                    errpreset: Some(spectre::ErrPreset::Conservative),
                    ..Default::default()
                },
            )
            .expect("failed to run simulation");
        let wav = wavs.as_waveform();
        let (sum, n) = wav
            .edges(self.pvt.voltage.to_f64().unwrap() / 2.0)
            .map(|e| e.t())
            .tuple_windows()
            .map(|(a, b)| (b - a, 1))
            .fold((0.0, 0), |acc, x| (acc.0 + x.0, acc.1 + x.1));
        let period = sum / n as f64;
        VcoTbOutput { period }
    }
}
