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
use substrate::geometry::bbox::Bbox;
use substrate::geometry::dir::Dir;
use substrate::geometry::rect::Rect;
use substrate::io::{Array, InOut, Input, Io, MosIo, MosIoSchematic, Output, Signal};
use substrate::layout::bbox::LayerBbox;
use substrate::layout::element::Shape;
use substrate::layout::tracks::RoundingMode;
use substrate::layout::ExportsLayoutData;
use substrate::pdk::layers::HasPin;
use substrate::pdk::layers::{Layer, LayerId};
use substrate::pdk::{Pdk, PdkLayers};
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

/// The parameters of the [`HorizontalDriverUnit`] and [`VerticalDriverUnit`] layout generators.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct DriverUnitParams {
    /// The width of the enable pull-up transistor of the NOR gate.
    pub nor_pu_en_w: i64,
    /// The width of the data pull-up transistor of the NOR gate.
    pub nor_pu_data_w: i64,
    /// The width of the enable pull-down transistor of the NOR gate.
    pub nor_pd_en_w: i64,
    /// The width of the data pull-down transistor of the NOR gate.
    pub nor_pd_data_w: i64,
    /// Half of the width of the driver pull-down transistor.
    pub driver_pd_w: i64,
    /// The length of the pull-down resistor.
    pub pd_res_l: i64,
    /// The length of the pull-up resistor.
    pub pu_res_l: i64,
    /// Half of the width of the driver pull-up transistor.
    pub driver_pu_w: i64,
    /// The width of the enable pull-up transistor of the NAND gate.
    pub nand_pu_en_w: i64,
    /// The width of the data pull-up transistor of the NAND gate.
    pub nand_pu_data_w: i64,
    /// The width of the enable pull-down transistor of the NAND gate.
    pub nand_pd_en_w: i64,
    /// The width of the data pull-down transistor of the NAND gate.
    pub nand_pd_data_w: i64,
}

/// The interface to a driver.
#[derive(Debug, Clone, Io)]
pub struct DriverIo {
    /// The buffer input.
    pub din: Input<Signal>,
    /// The buffered output.
    pub dout: Output<Signal>,
    /// The pull-up control.
    pub pu_ctl: Array<Input<Signal>>,
    /// The pull-down control.
    pub pd_ctl: Array<Input<Signal>>,
    /// The VDD rail.
    pub vdd: InOut<Signal>,
    /// The VSS rail.
    pub vss: InOut<Signal>,
}

/// The parameters of the [`HorizontalDriver`] and [`VerticalDriver`] layout generators.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct DriverParams {
    /// Parameters of the driver unit.
    pub unit: DriverUnitParams,
    /// Number of segments.
    pub num_segments: usize,
}

/// A horizontal driver implementation.
pub trait HorizontalDriverImpl<PDK: Pdk + Schema> {
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

/// A vertical driver implementation.
pub trait VerticalDriverImpl<PDK: Pdk + Schema> {
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
    /// Returns the n-well layer ID.
    fn nwell_id(layers: &PdkLayers<PDK>) -> LayerId;
    /// Transforms the given n-well rectangle to be DRC clean.
    fn nwell_transform(rect: Rect) -> Rect {
        rect
    }
    /// Defines the layer 2 track numbers that correspond to the DIN bus.
    fn define_din_bus(cell: &mut TileBuilder<'_, PDK>) -> Vec<i64> {
        let virtual_layers = cell.layout.ctx.install_layers::<atoll::VirtualLayers>();
        let bbox = cell.layout.layer_bbox(virtual_layers.outline.id()).unwrap();
        vec![cell.layer_stack.layers[2]
            .inner
            .tracks()
            .to_track_idx(bbox.center().x, RoundingMode::Down)]
    }
    /// Defines the layer 2 track numbers that correspond to the DOUT bus.
    fn define_dout_bus(cell: &mut TileBuilder<'_, PDK>) -> Vec<i64> {
        let virtual_layers = cell.layout.ctx.install_layers::<atoll::VirtualLayers>();
        let bbox = cell.layout.layer_bbox(virtual_layers.outline.id()).unwrap();
        vec![
            cell.layer_stack.layers[2]
                .inner
                .tracks()
                .to_track_idx(bbox.center().x, RoundingMode::Down)
                + 1,
        ]
    }
    /// Additional layout hooks to run after the inverter layout is complete.
    fn post_layout_hooks(_cell: &mut TileBuilder<'_, PDK>) -> Result<()> {
        Ok(())
    }
}

/// A horizontal driver unit.
#[derive_where::derive_where(Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[derive(Serialize, Deserialize)]
pub struct HorizontalDriverUnit<T>(
    DriverUnitParams,
    #[serde(bound(deserialize = ""))] PhantomData<fn() -> T>,
);

impl<T> HorizontalDriverUnit<T> {
    /// Creates a new [`HorizontalDriverUnit`].
    pub fn new(params: DriverUnitParams) -> Self {
        Self(params, PhantomData)
    }
}

impl<T: Any> Block for HorizontalDriverUnit<T> {
    type Io = DriverUnitIo;

    fn id() -> ArcStr {
        substrate::arcstr::literal!("horizontal_driver_unit")
    }

    // todo: include parameters in name
    fn name(&self) -> ArcStr {
        substrate::arcstr::literal!("horizontal_driver_unit")
    }

    fn io(&self) -> Self::Io {
        Default::default()
    }
}

impl<T: Any> ExportsNestedData for HorizontalDriverUnit<T> {
    type NestedData = ();
}

impl<T: Any> ExportsLayoutData for HorizontalDriverUnit<T> {
    type LayoutData = ();
}

impl<PDK: Pdk + Schema + Sized, T: HorizontalDriverImpl<PDK> + Any> Tile<PDK>
    for HorizontalDriverUnit<T>
{
    fn tile<'a>(
        &self,
        io: IoBuilder<'a, Self>,
        cell: &mut TileBuilder<'a, PDK>,
    ) -> substrate::error::Result<(
        <Self as ExportsNestedData>::NestedData,
        <Self as ExportsLayoutData>::LayoutData,
    )> {
        let nor_pu_en_params = MosTileParams::new(TileKind::P, self.0.nor_pu_en_w);
        let nor_pu_data_params = MosTileParams::new(TileKind::P, self.0.nor_pu_data_w);
        let nor_pd_en_params = MosTileParams::new(TileKind::N, self.0.nor_pd_en_w);
        let nor_pd_data_params = MosTileParams::new(TileKind::N, self.0.nor_pd_data_w);
        let driver_pd_params = MosTileParams::new(TileKind::N, self.0.driver_pd_w);
        let pd_res_params = ResistorTileParams::new(self.0.pd_res_l);
        let pu_res_params = ResistorTileParams::new(self.0.pu_res_l);
        let driver_pu_params = MosTileParams::new(TileKind::P, self.0.driver_pu_w);
        let nand_pu_en_params = MosTileParams::new(TileKind::P, self.0.nand_pu_en_w);
        let nand_pu_data_params = MosTileParams::new(TileKind::P, self.0.nand_pu_data_w);
        let nand_pd_en_params = MosTileParams::new(TileKind::N, self.0.nand_pd_en_w);
        let nand_pd_data_params = MosTileParams::new(TileKind::N, self.0.nand_pd_data_w);

        let nor_x = cell.signal("nor_x", Signal::new());
        let nand_x = cell.signal("nand_x", Signal::new());
        let pd_en = cell.signal("pd_en", Signal::new());
        let pu_en = cell.signal("pu_en", Signal::new());
        let pd_x = cell.signal("pd_x", Signal::new());
        let pu_x = cell.signal("pu_x", Signal::new());

        let mut nor_pu_en = cell
            .generate_connected(
                T::mos(nor_pu_en_params),
                MosIoSchematic {
                    d: io.schematic.vdd,
                    g: io.schematic.pd_ctl,
                    s: nor_x,
                    b: io.schematic.vdd,
                },
            )
            .orient(Orientation::ReflectVert);
        let mut nor_pu_data = cell
            .generate_connected(
                T::mos(nor_pu_data_params),
                MosIoSchematic {
                    d: nor_x,
                    g: io.schematic.din,
                    s: pd_en,
                    b: io.schematic.vdd,
                },
            )
            .orient(Orientation::ReflectVert);
        let mut nor_pd_en = cell.generate_connected(
            T::mos(nor_pd_en_params),
            MosIoSchematic {
                d: pd_en,
                g: io.schematic.pu_ctl,
                s: io.schematic.vss,
                b: io.schematic.vss,
            },
        );
        let mut nor_pd_data = cell.generate_connected(
            T::mos(nor_pd_data_params),
            MosIoSchematic {
                d: pd_en,
                g: io.schematic.din,
                s: io.schematic.vss,
                b: io.schematic.vss,
            },
        );
        let mut driver_pd = cell
            .generate_connected(
                T::mos(driver_pd_params),
                MosIoSchematic {
                    d: io.schematic.vss,
                    g: io.schematic.din,
                    s: pd_x,
                    b: io.schematic.vss,
                },
            )
            .orient(Orientation::ReflectVert);
        let mut pd_res = cell
            .generate_connected(
                T::resistor(pd_res_params),
                ResistorIoSchematic {
                    p: io.schematic.dout,
                    n: pd_x,
                    b: io.schematic.vdd,
                },
            )
            .orient(Orientation::ReflectVert);
        let mut pu_res = cell.generate_connected(
            T::resistor(pu_res_params),
            ResistorIoSchematic {
                p: io.schematic.dout,
                n: pu_x,
                b: io.schematic.vdd,
            },
        );
        let mut driver_pu = cell.generate_connected(
            T::mos(driver_pu_params),
            MosIoSchematic {
                d: io.schematic.vdd,
                g: io.schematic.din,
                s: pu_x,
                b: io.schematic.vdd,
            },
        );
        let mut nand_pu_en = cell
            .generate_connected(
                T::mos(nand_pu_en_params),
                MosIoSchematic {
                    d: pu_en,
                    g: io.schematic.pu_ctl,
                    s: io.schematic.vdd,
                    b: io.schematic.vdd,
                },
            )
            .orient(Orientation::ReflectVert);
        let mut nand_pu_data = cell
            .generate_connected(
                T::mos(nand_pu_data_params),
                MosIoSchematic {
                    d: pu_en,
                    g: io.schematic.din,
                    s: io.schematic.vdd,
                    b: io.schematic.vdd,
                },
            )
            .orient(Orientation::ReflectVert);
        let mut nand_pd_en = cell.generate_connected(
            T::mos(nand_pd_en_params),
            MosIoSchematic {
                d: io.schematic.vss,
                g: io.schematic.pd_ctl,
                s: nand_x,
                b: io.schematic.vss,
            },
        );
        let mut nand_pd_data = cell.generate_connected(
            T::mos(nand_pd_data_params),
            MosIoSchematic {
                d: nand_x,
                g: io.schematic.din,
                s: pu_en,
                b: io.schematic.vss,
            },
        );

        let mut ntap_bot = cell.generate(T::tap(TapTileParams::new(TileKind::N, 1)));
        let mut ptap = cell.generate(T::tap(TapTileParams::new(TileKind::P, 1)));
        let mut ntap = cell.generate(T::tap(TapTileParams::new(TileKind::N, 1)));
        let ptap_top = cell.generate(T::tap(TapTileParams::new(TileKind::P, 1)));
        cell.connect(ntap_bot.io().x, io.schematic.vdd);
        cell.connect(ptap.io().x, io.schematic.vss);
        cell.connect(ntap.io().x, io.schematic.vdd);
        cell.connect(ptap_top.io().x, io.schematic.vss);

        nand_pd_en.align_mut(&ptap_top, AlignMode::Left, 0);
        nand_pd_en.align_mut(&ptap_top, AlignMode::Beneath, 0);
        nand_pd_data.align_mut(&nand_pd_en, AlignMode::Left, 0);
        nand_pd_data.align_mut(&nand_pd_en, AlignMode::Beneath, 0);
        nand_pu_data.align_mut(&nand_pd_data, AlignMode::Left, 0);
        nand_pu_data.align_mut(&nand_pd_data, AlignMode::Beneath, 0);
        nand_pu_en.align_mut(&nand_pu_data, AlignMode::Left, 0);
        nand_pu_en.align_mut(&nand_pu_data, AlignMode::Beneath, 0);

        ntap.align_mut(&nand_pu_en, AlignMode::Left, 0);
        ntap.align_mut(&nand_pu_en, AlignMode::Beneath, 0);

        driver_pu.align_mut(&ntap, AlignMode::Left, 0);
        driver_pu.align_mut(&ntap, AlignMode::Beneath, 0);

        pu_res.align_mut(&driver_pu, AlignMode::Left, 0);
        pu_res.align_mut(&driver_pu, AlignMode::Beneath, 0);

        pd_res.align_mut(&pu_res, AlignMode::Left, 0);
        pd_res.align_mut(&pu_res, AlignMode::Beneath, 0);

        driver_pd.align_mut(&pd_res, AlignMode::Left, 0);
        driver_pd.align_mut(&pd_res, AlignMode::Beneath, 0);

        ptap.align_mut(&driver_pd, AlignMode::Left, 0);
        ptap.align_mut(&driver_pd, AlignMode::Beneath, 0);

        nor_pd_en.align_mut(&ptap, AlignMode::Left, 0);
        nor_pd_en.align_mut(&ptap, AlignMode::Beneath, 0);
        nor_pd_data.align_mut(&nor_pd_en, AlignMode::Left, 0);
        nor_pd_data.align_mut(&nor_pd_en, AlignMode::Beneath, 0);
        nor_pu_data.align_mut(&nor_pd_data, AlignMode::Left, 0);
        nor_pu_data.align_mut(&nor_pd_data, AlignMode::Beneath, 0);
        nor_pu_en.align_mut(&nor_pu_data, AlignMode::Left, 0);
        nor_pu_en.align_mut(&nor_pu_data, AlignMode::Beneath, 0);

        ntap_bot.align_mut(&nor_pu_en, AlignMode::Left, 0);
        ntap_bot.align_mut(&nor_pu_en, AlignMode::Beneath, 0);

        let nor_pd_en = cell.draw(nor_pd_en)?;
        let nor_pd_data = cell.draw(nor_pd_data)?;
        let _nor_pu_en = cell.draw(nor_pu_en)?;
        let _nor_pu_data = cell.draw(nor_pu_data)?;
        let _driver_pd = cell.draw(driver_pd)?;
        let _pd_res = cell.draw(pd_res)?;
        let pu_res = cell.draw(pu_res)?;
        let _driver_pu = cell.draw(driver_pu)?;
        let nand_pd_en = cell.draw(nand_pd_en)?;
        let _nand_pd_data = cell.draw(nand_pd_data)?;
        let _nand_pu_en = cell.draw(nand_pu_en)?;
        let _nand_pu_data = cell.draw(nand_pu_data)?;

        let _ntap_bot = cell.draw(ntap_bot)?;
        let ptap = cell.draw(ptap)?;
        let ntap = cell.draw(ntap)?;
        let _ptap_top = cell.draw(ptap_top)?;

        cell.set_top_layer(2);
        cell.set_router(GreedyRouter);
        cell.set_via_maker(T::via_maker());

        io.layout.din.merge(nor_pd_data.layout.io().g);
        io.layout.dout.merge(pu_res.layout.io().p);
        io.layout.pu_ctl.merge(nor_pd_en.layout.io().g);
        io.layout.pd_ctl.merge(nand_pd_en.layout.io().g);
        io.layout.vdd.merge(ntap.layout.io().x);
        io.layout.vss.merge(ptap.layout.io().x);

        T::post_layout_hooks(cell)?;

        Ok(((), ()))
    }
}

/// A vertical driver unit.
#[derive_where::derive_where(Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[derive(Serialize, Deserialize)]
pub struct VerticalDriverUnit<T>(
    DriverUnitParams,
    #[serde(bound(deserialize = ""))] PhantomData<fn() -> T>,
);

impl<T> VerticalDriverUnit<T> {
    /// Creates a new [`VerticalDriverUnit`].
    pub fn new(params: DriverUnitParams) -> Self {
        Self(params, PhantomData)
    }
}

impl<T: Any> Block for VerticalDriverUnit<T> {
    type Io = DriverUnitIo;

    fn id() -> ArcStr {
        substrate::arcstr::literal!("vertical_driver_unit")
    }

    // todo: include parameters in name
    fn name(&self) -> ArcStr {
        substrate::arcstr::literal!("vertical_driver_unit")
    }

    fn io(&self) -> Self::Io {
        Default::default()
    }
}

impl<T: Any> ExportsNestedData for VerticalDriverUnit<T> {
    type NestedData = ();
}

impl<T: Any> ExportsLayoutData for VerticalDriverUnit<T> {
    type LayoutData = ();
}

impl<PDK: Pdk + Schema + Sized, T: VerticalDriverImpl<PDK> + Any> Tile<PDK>
    for VerticalDriverUnit<T>
{
    fn tile<'a>(
        &self,
        io: IoBuilder<'a, Self>,
        cell: &mut TileBuilder<'a, PDK>,
    ) -> substrate::error::Result<(
        <Self as ExportsNestedData>::NestedData,
        <Self as ExportsLayoutData>::LayoutData,
    )> {
        let nor_pu_en_params = MosTileParams::new(TileKind::P, self.0.nor_pu_en_w);
        let nor_pu_data_params = MosTileParams::new(TileKind::P, self.0.nor_pu_data_w);
        let nor_pd_en_params = MosTileParams::new(TileKind::N, self.0.nor_pd_en_w);
        let nor_pd_data_params = MosTileParams::new(TileKind::N, self.0.nor_pd_data_w);
        let driver_pd_params = MosTileParams::new(TileKind::N, self.0.driver_pd_w);
        let pd_res_params = ResistorTileParams::new(self.0.pd_res_l);
        let pu_res_params = ResistorTileParams::new(self.0.pu_res_l);
        let driver_pu_params = MosTileParams::new(TileKind::P, self.0.driver_pu_w);
        let nand_pu_en_params = MosTileParams::new(TileKind::P, self.0.nand_pu_en_w);
        let nand_pu_data_params = MosTileParams::new(TileKind::P, self.0.nand_pu_data_w);
        let nand_pd_en_params = MosTileParams::new(TileKind::N, self.0.nand_pd_en_w);
        let nand_pd_data_params = MosTileParams::new(TileKind::N, self.0.nand_pd_data_w);

        let nor_x = cell.signal("nor_x", Signal::new());
        let nand_x = cell.signal("nand_x", Signal::new());
        let pd_en = cell.signal("pd_en", Signal::new());
        let pu_en = cell.signal("pu_en", Signal::new());
        let pd_x = cell.signal("pd_x", Signal::new());
        let pu_x = cell.signal("pu_x", Signal::new());

        let mut nor_pu_en = cell.generate_connected(
            T::mos(nor_pu_en_params),
            MosIoSchematic {
                d: io.schematic.vdd,
                g: io.schematic.pd_ctl,
                s: nor_x,
                b: io.schematic.vdd,
            },
        );
        let mut nor_pu_data = cell.generate_connected(
            T::mos(nor_pu_data_params),
            MosIoSchematic {
                d: nor_x,
                g: io.schematic.din,
                s: pd_en,
                b: io.schematic.vdd,
            },
        );
        let mut nor_pd_en = cell.generate_connected(
            T::mos(nor_pd_en_params),
            MosIoSchematic {
                d: pd_en,
                g: io.schematic.pu_ctl,
                s: io.schematic.vss,
                b: io.schematic.vss,
            },
        );
        let mut nor_pd_data = cell.generate_connected(
            T::mos(nor_pd_data_params),
            MosIoSchematic {
                d: pd_en,
                g: io.schematic.din,
                s: io.schematic.vss,
                b: io.schematic.vss,
            },
        );
        let mut driver_pd = cell.generate_connected(
            T::mos(driver_pd_params),
            MosIoSchematic {
                d: io.schematic.vss,
                g: io.schematic.din,
                s: pd_x,
                b: io.schematic.vss,
            },
        );
        let mut pd_res = cell
            .generate_connected(
                T::resistor(pd_res_params),
                ResistorIoSchematic {
                    p: io.schematic.dout,
                    n: pd_x,
                    b: io.schematic.vdd,
                },
            )
            .orient(Orientation::ReflectHoriz);
        let mut pu_res = cell.generate_connected(
            T::resistor(pu_res_params),
            ResistorIoSchematic {
                p: io.schematic.dout,
                n: pu_x,
                b: io.schematic.vdd,
            },
        );
        let mut driver_pu = cell.generate_connected(
            T::mos(driver_pu_params),
            MosIoSchematic {
                d: io.schematic.vdd,
                g: io.schematic.din,
                s: pu_x,
                b: io.schematic.vdd,
            },
        );
        let mut nand_pu_en = cell.generate_connected(
            T::mos(nand_pu_en_params),
            MosIoSchematic {
                d: pu_en,
                g: io.schematic.pu_ctl,
                s: io.schematic.vdd,
                b: io.schematic.vdd,
            },
        );
        let mut nand_pu_data = cell.generate_connected(
            T::mos(nand_pu_data_params),
            MosIoSchematic {
                d: pu_en,
                g: io.schematic.din,
                s: io.schematic.vdd,
                b: io.schematic.vdd,
            },
        );
        let mut nand_pd_en = cell.generate_connected(
            T::mos(nand_pd_en_params),
            MosIoSchematic {
                d: io.schematic.vss,
                g: io.schematic.pd_ctl,
                s: nand_x,
                b: io.schematic.vss,
            },
        );
        let mut nand_pd_data = cell.generate_connected(
            T::mos(nand_pd_data_params),
            MosIoSchematic {
                d: nand_x,
                g: io.schematic.din,
                s: pu_en,
                b: io.schematic.vss,
            },
        );

        let mut ntap_bot = cell.generate(T::tap(TapTileParams::new(TileKind::N, 1)));
        let mut ptap = cell.generate(T::tap(TapTileParams::new(TileKind::P, 1)));
        let mut ntap = cell.generate(T::tap(TapTileParams::new(TileKind::N, 1)));
        let ptap_top = cell.generate(T::tap(TapTileParams::new(TileKind::P, 1)));
        cell.connect(ntap_bot.io().x, io.schematic.vdd);
        cell.connect(ptap.io().x, io.schematic.vss);
        cell.connect(ntap.io().x, io.schematic.vdd);
        cell.connect(ptap_top.io().x, io.schematic.vss);

        nand_pd_en.align_mut(&ptap_top, AlignMode::ToTheLeft, 0);
        nand_pd_en.align_mut(&ptap_top, AlignMode::Bottom, 0);
        nand_pd_data.align_mut(&nand_pd_en, AlignMode::ToTheLeft, 0);
        nand_pd_data.align_mut(&nand_pd_en, AlignMode::Bottom, 0);
        nand_pu_data.align_mut(&nand_pd_data, AlignMode::ToTheLeft, 0);
        nand_pu_data.align_mut(&nand_pd_data, AlignMode::Bottom, 0);
        nand_pu_en.align_mut(&nand_pu_data, AlignMode::ToTheLeft, 0);
        nand_pu_en.align_mut(&nand_pu_data, AlignMode::Bottom, 0);

        ntap.align_mut(&nand_pu_en, AlignMode::ToTheLeft, 0);
        ntap.align_mut(&nand_pu_en, AlignMode::Bottom, 0);

        driver_pu.align_mut(&ntap, AlignMode::ToTheLeft, 0);
        driver_pu.align_mut(&ntap, AlignMode::Bottom, 0);

        pu_res.align_mut(&driver_pu, AlignMode::ToTheLeft, 0);
        pu_res.align_mut(&driver_pu, AlignMode::Bottom, 0);

        pd_res.align_mut(&pu_res, AlignMode::ToTheLeft, 0);
        pd_res.align_mut(&pu_res, AlignMode::Bottom, 0);

        driver_pd.align_mut(&pd_res, AlignMode::ToTheLeft, 0);
        driver_pd.align_mut(&pd_res, AlignMode::Bottom, 0);

        ptap.align_mut(&driver_pd, AlignMode::ToTheLeft, 0);
        ptap.align_mut(&driver_pd, AlignMode::Bottom, 0);

        nor_pd_en.align_mut(&ptap, AlignMode::ToTheLeft, 0);
        nor_pd_en.align_mut(&ptap, AlignMode::Bottom, 0);
        nor_pd_data.align_mut(&nor_pd_en, AlignMode::ToTheLeft, 0);
        nor_pd_data.align_mut(&nor_pd_en, AlignMode::Bottom, 0);
        nor_pu_data.align_mut(&nor_pd_data, AlignMode::ToTheLeft, 0);
        nor_pu_data.align_mut(&nor_pd_data, AlignMode::Bottom, 0);
        nor_pu_en.align_mut(&nor_pu_data, AlignMode::ToTheLeft, 0);
        nor_pu_en.align_mut(&nor_pu_data, AlignMode::Bottom, 0);

        ntap_bot.align_mut(&nor_pu_en, AlignMode::ToTheLeft, 0);
        ntap_bot.align_mut(&nor_pu_en, AlignMode::Bottom, 0);

        let nor_pd_en = cell.draw(nor_pd_en)?;
        let nor_pd_data = cell.draw(nor_pd_data)?;
        let _nor_pu_en = cell.draw(nor_pu_en)?;
        let nor_pu_data = cell.draw(nor_pu_data)?;
        let _driver_pd = cell.draw(driver_pd)?;
        let pd_res = cell.draw(pd_res)?;
        let pu_res = cell.draw(pu_res)?;
        let _driver_pu = cell.draw(driver_pu)?;
        let nand_pd_en = cell.draw(nand_pd_en)?;
        let _nand_pd_data = cell.draw(nand_pd_data)?;
        let _nand_pu_en = cell.draw(nand_pu_en)?;
        let nand_pu_data = cell.draw(nand_pu_data)?;

        let ntap_bot = cell.draw(ntap_bot)?;
        let ptap = cell.draw(ptap)?;
        let ntap = cell.draw(ntap)?;
        let ptap_top = cell.draw(ptap_top)?;

        for tap in [&ntap_bot, &ptap, &ntap, &ptap_top] {
            for shape in tap.layout.io().x.shapes() {
                cell.layout.draw(Shape::new(
                    shape.layer().drawing(),
                    shape.bbox_rect().expand_dir(Dir::Vert, 136),
                ))?;
            }
        }

        let nwell = T::nwell_id(&cell.ctx().layers);

        cell.layout.draw(Shape::new(
            nwell,
            T::nwell_transform(
                ntap_bot
                    .layout
                    .layer_bbox(nwell)
                    .unwrap()
                    .union(nor_pu_data.layout.layer_bbox(nwell).unwrap()),
            ),
        ))?;

        cell.layout.draw(Shape::new(
            nwell,
            T::nwell_transform(
                pd_res
                    .layout
                    .layer_bbox(nwell)
                    .unwrap()
                    .union(nand_pu_data.layout.layer_bbox(nwell).unwrap()),
            ),
        ))?;

        let din_tracks = T::define_din_bus(cell);
        let dout_tracks = T::define_dout_bus(cell);

        let virtual_layers = cell.layout.ctx.install_layers::<atoll::VirtualLayers>();
        let bbox = cell.layout.layer_bbox(virtual_layers.outline.id()).unwrap();

        for (i, (track, is_din)) in din_tracks
            .into_iter()
            .map(|track| (track, true))
            .chain(dout_tracks.into_iter().map(|track| (track, false)))
            .enumerate()
        {
            let x = cell.signal(
                format!("{}_track_{i}", if is_din { "din" } else { "dout" }),
                Signal::new(),
            );
            cell.connect(
                if is_din {
                    io.schematic.din
                } else {
                    io.schematic.dout
                },
                x,
            );
            let track_rect = Rect::from_spans(
                cell.layer_stack.layers[2].inner.tracks().get(track),
                bbox.vspan(),
            );
            cell.layout
                .draw(Shape::new(cell.layer_stack.layers[2].id, track_rect))?;
            cell.assign_grid_points(
                x,
                2,
                cell.layer_stack
                    .slice(0..3)
                    .shrink_to_lcm_units(track_rect)
                    .unwrap(),
            );
        }

        cell.set_top_layer(2);
        cell.set_router(GreedyRouter);
        cell.set_via_maker(T::via_maker());

        io.layout.din.merge(nor_pd_data.layout.io().g);
        io.layout.dout.merge(pu_res.layout.io().p);
        io.layout.pu_ctl.merge(nor_pd_en.layout.io().g);
        io.layout.pd_ctl.merge(nand_pd_en.layout.io().g);
        io.layout.vdd.merge(ntap.layout.io().x);
        io.layout.vss.merge(ptap.layout.io().x);

        T::post_layout_hooks(cell)?;

        Ok(((), ()))
    }
}

/// A vertical driver.
#[derive_where::derive_where(Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[derive(Serialize, Deserialize)]
pub struct VerticalDriver<T>(
    DriverParams,
    #[serde(bound(deserialize = ""))] PhantomData<fn() -> T>,
);

impl<T> VerticalDriver<T> {
    /// Creates a new [`VerticalDriver`].
    pub fn new(params: DriverParams) -> Self {
        Self(params, PhantomData)
    }
}

impl<T: Any> Block for VerticalDriver<T> {
    type Io = DriverIo;

    fn id() -> ArcStr {
        substrate::arcstr::literal!("vertical_driver")
    }

    // todo: include parameters in name
    fn name(&self) -> ArcStr {
        substrate::arcstr::literal!("vertical_driver")
    }

    fn io(&self) -> Self::Io {
        DriverIo {
            din: Default::default(),
            dout: Default::default(),
            pu_ctl: Array::new(self.0.num_segments, Default::default()),
            pd_ctl: Array::new(self.0.num_segments, Default::default()),
            vdd: Default::default(),
            vss: Default::default(),
        }
    }
}

impl<T: Any> ExportsNestedData for VerticalDriver<T> {
    type NestedData = ();
}

impl<T: Any> ExportsLayoutData for VerticalDriver<T> {
    type LayoutData = ();
}

impl<PDK: Pdk + Schema + Sized, T: VerticalDriverImpl<PDK> + Any> Tile<PDK> for VerticalDriver<T> {
    fn tile<'a>(
        &self,
        io: IoBuilder<'a, Self>,
        cell: &mut TileBuilder<'a, PDK>,
    ) -> substrate::error::Result<(
        <Self as ExportsNestedData>::NestedData,
        <Self as ExportsLayoutData>::LayoutData,
    )> {
        let mut units = Vec::new();
        for i in 0..self.0.num_segments {
            let mut unit = cell.generate_connected(
                VerticalDriverUnit::<T>::new(self.0.unit),
                DriverUnitIoSchematic {
                    din: io.schematic.din,
                    dout: io.schematic.dout,
                    pu_ctl: io.schematic.pu_ctl[i],
                    pd_ctl: io.schematic.pd_ctl[i],
                    vdd: io.schematic.vdd,
                    vss: io.schematic.vss,
                },
            );
            if let Some(prev) = units.last() {
                unit.align_mut(prev, AlignMode::Beneath, 0);
                unit.align_mut(prev, AlignMode::Left, 0);
            }
            units.push(unit);
        }

        for (i, unit) in units.into_iter().enumerate() {
            let unit = cell.draw(unit)?;
            io.layout.din.merge(unit.layout.io().din);
            io.layout.dout.merge(unit.layout.io().dout);
            io.layout.pu_ctl[i].merge(unit.layout.io().pu_ctl);
            io.layout.pd_ctl[i].merge(unit.layout.io().pd_ctl);
            io.layout.vdd.merge(unit.layout.io().vdd);
            io.layout.vss.merge(unit.layout.io().vss);
        }

        // cell.set_top_layer(2);
        // cell.set_router(GreedyRouter);
        // cell.set_via_maker(T::via_maker());

        T::post_layout_hooks(cell)?;

        Ok(((), ()))
    }
}
