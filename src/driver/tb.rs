//! Driver verification testbenches.

use crate::driver::DriverIo;

use rust_decimal::Decimal;
use rust_decimal_macros::dec;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use spectre::analysis::ac::{Ac, Sweep};
use spectre::blocks::{AcSource, Isource, Vsource};
use spectre::{ErrPreset, Spectre};
use std::any::Any;
use std::fmt::Debug;
use std::hash::Hash;
use std::marker::PhantomData;
use substrate::arcstr;
use substrate::arcstr::ArcStr;
use substrate::block::Block;
use substrate::io::schematic::{HardwareType, Node};
use substrate::io::{Array, FlatLen, Signal, TestbenchIo, TwoTerminalIoSchematic};
use substrate::pdk::corner::Pvt;
use substrate::schematic::primitives::Resistor;
use substrate::schematic::schema::Schema;
use substrate::schematic::{Cell, CellBuilder, ExportsNestedData, NestedData, Schematic};
use substrate::scir::schema::FromSchema;
use substrate::simulation::data::{ac, FromSaved, Save, SaveTb};
use substrate::simulation::options::SimOption;
use substrate::simulation::{SimController, SimulationContext, Simulator, Testbench};

/// An AC testbench that sweeps frequency and measures output resistance.
#[derive_where::derive_where(Clone, Debug, Hash, PartialEq, Eq; T, C)]
#[derive(Serialize, Deserialize)]
pub struct DriverAcTb<T, PDK, C> {
    /// The device-under-test.
    pub dut: T,
    /// The start frequency.
    pub fstart: Decimal,
    /// The stop frequency.
    pub fstop: Decimal,
    /// The DC input bias voltage.
    pub vin: Decimal,
    /// The PVT corner.
    pub pvt: Pvt<C>,
    /// Pull-up enable mask.
    pub pu_mask: Vec<bool>,
    /// Pull-down enable mask.
    pub pd_mask: Vec<bool>,
    #[serde(bound(deserialize = ""))]
    phantom: PhantomData<fn() -> PDK>,
}

impl<T, PDK, C> DriverAcTb<T, PDK, C> {
    /// Creates a new [`DriverAcTb`].
    pub fn new(
        dut: T,
        fstart: Decimal,
        fstop: Decimal,
        vin: Decimal,
        pu_mask: Vec<bool>,
        pd_mask: Vec<bool>,
        pvt: Pvt<C>,
    ) -> Self {
        Self {
            dut,
            fstart,
            fstop,
            vin,
            pvt,
            pu_mask,
            pd_mask,
            phantom: PhantomData,
        }
    }
}

impl<
        T: Block,
        PDK: Any,
        C: Serialize
            + DeserializeOwned
            + Copy
            + Clone
            + Debug
            + Hash
            + PartialEq
            + Eq
            + Send
            + Sync
            + Any,
    > Block for DriverAcTb<T, PDK, C>
{
    type Io = TestbenchIo;

    fn id() -> ArcStr {
        arcstr::literal!("driver_ac_tb")
    }

    fn name(&self) -> ArcStr {
        arcstr::literal!("driver_ac_tb")
    }

    fn io(&self) -> Self::Io {
        Default::default()
    }
}

/// Nodes measured by [`DriverAcTb`].
#[derive(Clone, Debug, Hash, PartialEq, Eq, NestedData)]
pub struct DriverAcTbNodes {
    vout: Node,
}

impl<T, PDK, C> ExportsNestedData for DriverAcTb<T, PDK, C>
where
    DriverAcTb<T, PDK, C>: Block,
{
    type NestedData = DriverAcTbNodes;
}

impl<T: Block<Io = DriverIo> + Schematic<PDK> + Clone, PDK: Schema, C> Schematic<Spectre>
    for DriverAcTb<T, PDK, C>
where
    DriverAcTb<T, PDK, C>: Block<Io = TestbenchIo>,
    Spectre: FromSchema<PDK>,
{
    fn schematic(
        &self,
        io: &<<Self as Block>::Io as HardwareType>::Bundle,
        cell: &mut CellBuilder<Spectre>,
    ) -> substrate::error::Result<Self::NestedData> {
        let vin = cell.signal("vin", Signal);
        let vout = cell.signal("vout", Signal);
        let vdd = cell.signal("vdd", Signal);

        let dut = cell.sub_builder::<PDK>().instantiate(self.dut.clone());
        let pu_ctl = cell.signal("pu_ctl", Array::new(dut.io().pu_ctl.len(), Signal));
        let pd_ctlb = cell.signal("pd_ctlb", Array::new(dut.io().pu_ctl.len(), Signal));

        assert_eq!(pu_ctl.len(), self.pu_mask.len());
        assert_eq!(pd_ctlb.len(), self.pd_mask.len());

        for i in 0..pu_ctl.len() {
            cell.connect(&dut.io().pu_ctl[i], &pu_ctl[i]);
            let supply = if self.pu_mask[i] { vdd } else { io.vss };
            cell.instantiate_connected(
                Resistor::new(dec!(100)),
                TwoTerminalIoSchematic {
                    p: pu_ctl[i],
                    n: supply,
                },
            );
        }
        for i in 0..pd_ctlb.len() {
            cell.connect(&dut.io().pd_ctlb[i], &pd_ctlb[i]);
            let supply = if self.pd_mask[i] { io.vss } else { vdd };
            cell.instantiate_connected(
                Resistor::new(dec!(100)),
                TwoTerminalIoSchematic {
                    p: pd_ctlb[i],
                    n: supply,
                },
            );
        }

        cell.connect(dut.io().vdd, vdd);
        cell.connect(dut.io().vss, io.vss);
        cell.connect(dut.io().din, vin);
        cell.connect(dut.io().dout, vout);

        cell.instantiate_connected(
            Vsource::dc(self.vin),
            TwoTerminalIoSchematic { p: vin, n: io.vss },
        );
        cell.instantiate_connected(
            Vsource::dc(self.pvt.voltage),
            TwoTerminalIoSchematic { p: vdd, n: io.vss },
        );
        cell.instantiate_connected(
            Isource::ac(AcSource {
                dc: dec!(0),
                mag: dec!(1),
                phase: dec!(0),
            }),
            TwoTerminalIoSchematic { p: io.vss, n: vout },
        );

        Ok(DriverAcTbNodes { vout })
    }
}

/// The resulting waveforms of a [`DriverAcTb`].
#[derive(Debug, Clone, Serialize, Deserialize, FromSaved)]
pub struct DriverAcSim {
    /// The simulation frequency.
    pub freq: ac::Freq,
    /// The output voltage.
    pub vout: ac::Voltage,
}

impl<T, PDK, C> SaveTb<Spectre, Ac, DriverAcSim> for DriverAcTb<T, PDK, C>
where
    DriverAcTb<T, PDK, C>: Block<Io = TestbenchIo>,
{
    fn save_tb(
        ctx: &SimulationContext<Spectre>,
        cell: &Cell<Self>,
        opts: &mut <Spectre as Simulator>::Options,
    ) -> <DriverAcSim as FromSaved<Spectre, Ac>>::SavedKey {
        DriverAcSimSavedKey {
            freq: ac::Freq::save(ctx, (), opts),
            vout: ac::Voltage::save(ctx, &cell.vout, opts),
        }
    }
}

impl<T, PDK, C: SimOption<Spectre> + Copy> Testbench<Spectre> for DriverAcTb<T, PDK, C>
where
    DriverAcTb<T, PDK, C>: Block<Io = TestbenchIo> + Schematic<Spectre>,
{
    type Output = DriverAcSim;

    fn run(&self, sim: SimController<Spectre, Self>) -> Self::Output {
        let mut opts = spectre::Options::default();
        sim.set_option(self.pvt.corner, &mut opts);
        let wav: DriverAcSim = sim
            .simulate(
                opts,
                Ac {
                    start: dec!(1e3),
                    stop: dec!(50e9),
                    sweep: Sweep::Decade(40),
                    errpreset: Some(ErrPreset::Conservative),
                },
            )
            .expect("failed to run simulation");
        wav
    }
}
