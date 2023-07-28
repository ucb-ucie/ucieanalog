use rust_decimal::Decimal;
use rust_decimal_macros::dec;
use serde::{Deserialize, Serialize};
use sky130pdk::corner::Sky130Corner;
use sky130pdk::mos::{Nfet01v8, Pfet01v8};
use sky130pdk::Sky130CommercialPdk;
use spectre::blocks::{Pulse, Vsource};
use spectre::{Options, Spectre};
use spectre::Tran;
use substrate::{Block, Io};
use substrate::block::Block;
use substrate::io::{Input, Node, Output, SchematicType, Signal};
use substrate::pdk::corner::{InstallCorner, Pvt};
use substrate::schematic::{Cell, CellBuilder, HasSchematic, HasSchematicImpl, TestbenchCellBuilder};
use substrate::simulation::{HasTestbenchSchematicImpl, SimController, Testbench};
use crate::PowerIo;

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

impl HasSchematic for CurrentStarvedInverter {
    type Data = ();
}

impl HasSchematicImpl<Sky130CommercialPdk> for CurrentStarvedInverter {
    fn schematic(&self, io: &<<Self as Block>::Io as SchematicType>::Data, cell: &mut CellBuilder<Sky130CommercialPdk, Self>) -> substrate::error::Result<Self::Data> {
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

#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq, Serialize, Deserialize, Block)]
#[substrate(io = "substrate::io::TestbenchIo")]
pub struct DelayCellTb<T> {
    pub dut: T,
    pub pvt: Pvt<Sky130Corner>,
    pub tr: Decimal,
    pub tf: Decimal,
}

impl<T> HasSchematic for DelayCellTb<T> {
    type Data = Node;
}

impl<T> HasTestbenchSchematicImpl<Sky130CommercialPdk, Spectre> for DelayCellTb<T>
where T: Block<Io = DelayCellIo> + Clone {
    fn schematic(&self, io: &<<Self as Block>::Io as SchematicType>::Data, cell: &mut TestbenchCellBuilder<Sky130CommercialPdk, Spectre, Self>) -> substrate::error::Result<Self::Data> {
        let dut = cell.instantiate(self.dut.clone());

        let vdd = cell.instantiate(Vsource::dc(self.pvt.voltage));
        cell.connect(vdd.io().p, dut.io().pwr.vdd);
        cell.connect(vdd.io().n, io.vss);
        cell.connect(io.vss, dut.io().pwr.vss);

        let vtune = cell.instantiate(Vsource::dc(dec!(0.0)));
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

        let vin = cell.instantiate(vin);
        cell.connect(vin.io().p, dut.io().input);
        cell.connect(vin.io().n, io.vss);

        Ok(*dut.io().output)
    }
}

impl<T> Testbench<Sky130CommercialPdk, Spectre> for DelayCellTb<T>
where T: Block<Io = DelayCellIo> + Clone {
    type Output = ();
    fn run(&self, cell: &Cell<Self>, sim: SimController<Sky130CommercialPdk, Spectre>) -> Self::Output {
        let mut opts = Options::default();
        sim.pdk.pdk.install_corner(self.pvt.corner, &mut opts);
        let output = sim.simulate(opts, Tran {
            stop: dec!(3e-9),
            errpreset: Some(spectre::ErrPreset::Conservative),
            ..Default::default()
        }).expect("failed to run simulation");
    }
}