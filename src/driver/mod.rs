//! Driver layout generators.

use crate::tiles::{
    MosTileParams, ResistorIo, ResistorIoSchematic, ResistorTileParams, TapIo, TapTileParams,
    TileKind,
};
use atoll::route::{GreedyRouter, ViaMaker};
use atoll::{IoBuilder, Orientation, Tile, TileBuilder};
use serde::{Deserialize, Serialize};
use std::any::Any;
use std::marker::PhantomData;
use substrate::arcstr::ArcStr;
use substrate::block::Block;
use substrate::error::Result;
use substrate::geometry::align::AlignMode;
use substrate::io::{InOut, Input, Io, MosIo, MosIoSchematic, Output, Signal};
use substrate::layout::ExportsLayoutData;
use substrate::pdk::Pdk;
use substrate::schematic::schema::Schema;
use substrate::schematic::ExportsNestedData;

/// The interface to a driver unit.
#[derive(Debug, Default, Clone, Io)]
pub struct DriverUnitIo {
    /// The buffer input.
    pub din: Input<Signal>,
    /// The buffered output.
    pub dout: Output<Signal>,
    /// The pull-up control.
    pub pu_ctl: Input<Signal>,
    /// The pull-down control.
    pub pd_ctl: Input<Signal>,
    /// The VDD rail.
    pub vdd: InOut<Signal>,
    /// The VSS rail.
    pub vss: InOut<Signal>,
}

/// The parameters of the [`DriverUnit`] layout generator.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct DriverUnitParams {
    /// The width of the pull-up transistors of the NOR gate.
    pub nor_pu_w: i64,
    /// The width of the pull-down transistors of the NOR gate.
    pub nor_pd_w: i64,
    /// Half of the width of the driver pull-down transistor.
    pub driver_pd_w: i64,
    /// The length of the pull-down resistor.
    pub pd_res_l: i64,
    /// The length of the pull-up resistor.
    pub pu_res_l: i64,
    /// Half of the width of the driver pull-up transistor.
    pub driver_pu_w: i64,
    /// The width of the pull-up transistors of the NAND gate.
    pub nand_pu_w: i64,
    /// The width of the pull-down transistors of the NAND gate.
    pub nand_pd_w: i64,
}

/// A driver implementation.
pub trait DriverImpl<PDK: Pdk + Schema> {
    /// The MOS tile used to implement the pull-up and pull-down transistors.
    type MosTile: Tile<PDK> + Block<Io = MosIo> + Clone;
    /// The tap tile.
    type TapTile: Tile<PDK> + Block<Io = TapIo> + Clone;
    /// The resistor tile.
    type ResistorTile: Tile<PDK> + Block<Io = ResistorIo> + Clone;
    /// A PDK-specific via maker.
    type ViaMaker: ViaMaker<PDK>;

    /// Creates an instance of the MOS tile.
    fn mos(params: MosTileParams) -> Self::MosTile;
    /// Creates an instance of the tap tile.
    fn tap(params: TapTileParams) -> Self::TapTile;
    /// Creates an instance of the resistor tile.
    fn resistor(params: ResistorTileParams) -> Self::ResistorTile;
    /// Creates a PDK-specific via maker.
    fn via_maker() -> Self::ViaMaker;
    /// Additional layout hooks to run after the inverter layout is complete.
    fn post_layout_hooks(_cell: &mut TileBuilder<'_, PDK>) -> Result<()> {
        Ok(())
    }
}

/// A driver unit.
#[derive_where::derive_where(Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[derive(Serialize, Deserialize)]
pub struct DriverUnit<T>(
    DriverUnitParams,
    #[serde(bound(deserialize = ""))] PhantomData<fn() -> T>,
);

impl<T> DriverUnit<T> {
    /// Creates a new [`DriverUnit`].
    pub fn new(params: DriverUnitParams) -> Self {
        Self(params, PhantomData)
    }
}

impl<T: Any> Block for DriverUnit<T> {
    type Io = DriverUnitIo;

    fn id() -> ArcStr {
        substrate::arcstr::literal!("driver_unit")
    }

    // todo: include parameters in name
    fn name(&self) -> ArcStr {
        substrate::arcstr::literal!("driver_unit")
    }

    fn io(&self) -> Self::Io {
        Default::default()
    }
}

impl<T: Any> ExportsNestedData for DriverUnit<T> {
    type NestedData = ();
}

impl<T: Any> ExportsLayoutData for DriverUnit<T> {
    type LayoutData = ();
}

impl<PDK: Pdk + Schema + Sized, T: DriverImpl<PDK> + Any> Tile<PDK> for DriverUnit<T> {
    fn tile<'a>(
        &self,
        io: IoBuilder<'a, Self>,
        cell: &mut TileBuilder<'a, PDK>,
    ) -> substrate::error::Result<(
        <Self as ExportsNestedData>::NestedData,
        <Self as ExportsLayoutData>::LayoutData,
    )> {
        let driver_pd_params = MosTileParams::new(TileKind::N, self.0.driver_pd_w);
        let pd_res = ResistorTileParams::new(self.0.pd_res_l);
        let pu_res = ResistorTileParams::new(self.0.pu_res_l);
        let driver_pu_params = MosTileParams::new(TileKind::P, self.0.driver_pu_w);

        let pd_x = cell.signal("pd_x", Signal::new());
        let pu_x = cell.signal("pu_x", Signal::new());

        let mut driver_pd_pair = (0..2)
            .map(|_| {
                cell.generate_connected(
                    T::mos(driver_pd_params),
                    MosIoSchematic {
                        d: io.schematic.vss,
                        g: io.schematic.din,
                        s: pd_x,
                        b: io.schematic.vss,
                    },
                )
                .orient(Orientation::ReflectVert)
            })
            .collect::<Vec<_>>();
        let mut pd_res = cell
            .generate_connected(
                T::resistor(pd_res),
                ResistorIoSchematic {
                    p: io.schematic.dout,
                    n: pd_x,
                    b: io.schematic.vdd,
                },
            )
            .orient(Orientation::ReflectVert);
        let mut pu_res = cell.generate_connected(
            T::resistor(pu_res),
            ResistorIoSchematic {
                p: io.schematic.dout,
                n: pu_x,
                b: io.schematic.vdd,
            },
        );
        let mut driver_pu_pair = (0..2)
            .map(|_| {
                cell.generate_connected(
                    T::mos(driver_pu_params),
                    MosIoSchematic {
                        d: io.schematic.vdd,
                        g: io.schematic.din,
                        s: pu_x,
                        b: io.schematic.vdd,
                    },
                )
            })
            .collect::<Vec<_>>();

        let mut ptap = cell.generate(T::tap(TapTileParams::new(TileKind::P, 2)));
        let ntap = cell.generate(T::tap(TapTileParams::new(TileKind::N, 2)));
        cell.connect(ptap.io().x, io.schematic.vss);
        cell.connect(ntap.io().x, io.schematic.vdd);

        driver_pu_pair[0].align_mut(&ntap, AlignMode::Left, 0);
        driver_pu_pair[0].align_mut(&ntap, AlignMode::Beneath, 0);
        let prev_bounds = driver_pu_pair[0].lcm_bounds();
        driver_pu_pair[1].align_rect_mut(prev_bounds, AlignMode::ToTheRight, 0);
        driver_pu_pair[1].align_rect_mut(prev_bounds, AlignMode::Bottom, 0);

        pu_res.align_mut(&driver_pu_pair[0], AlignMode::Left, 0);
        pu_res.align_mut(&driver_pu_pair[0], AlignMode::Beneath, 0);

        pd_res.align_mut(&pu_res, AlignMode::Left, 0);
        pd_res.align_mut(&pu_res, AlignMode::Beneath, 0);

        driver_pd_pair[0].align_mut(&pd_res, AlignMode::Left, 0);
        driver_pd_pair[0].align_mut(&pd_res, AlignMode::Beneath, 0);
        let prev_bounds = driver_pd_pair[0].lcm_bounds();
        driver_pd_pair[1].align_rect_mut(prev_bounds, AlignMode::ToTheRight, 0);
        driver_pd_pair[1].align_rect_mut(prev_bounds, AlignMode::Bottom, 0);

        ptap.align_mut(&driver_pd_pair[0], AlignMode::Left, 0);
        ptap.align_mut(&driver_pd_pair[0], AlignMode::Beneath, 0);

        let driver_pd_pair = driver_pd_pair
            .into_iter()
            .map(|mos| cell.draw(mos))
            .collect::<Result<Vec<_>>>()?;
        let pd_res = cell.draw(pd_res)?;
        let pu_res = cell.draw(pu_res)?;
        let driver_pu_pair = driver_pu_pair
            .into_iter()
            .map(|mos| cell.draw(mos))
            .collect::<Result<Vec<_>>>()?;
        let ptap = cell.draw(ptap)?;
        let ntap = cell.draw(ntap)?;

        cell.set_top_layer(2);
        cell.set_router(GreedyRouter);
        cell.set_via_maker(T::via_maker());

        io.layout.din.merge(driver_pd_pair[0].layout.io().g);
        io.layout.din.merge(driver_pu_pair[0].layout.io().g);
        io.layout.dout.merge(driver_pd_pair[0].layout.io().s);
        io.layout.dout.merge(driver_pu_pair[0].layout.io().s);
        io.layout.pu_ctl.merge(driver_pu_pair[0].layout.io().g);
        io.layout.pd_ctl.merge(driver_pd_pair[0].layout.io().s);
        io.layout.vdd.merge(ntap.layout.io().x);
        io.layout.vss.merge(ptap.layout.io().x);

        T::post_layout_hooks(cell)?;

        Ok(((), ()))
    }
}
