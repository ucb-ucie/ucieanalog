//! Driver layout generators.

pub mod tb;

use crate::tiles::{
    MosKind, MosTileParams, ResistorConn, ResistorIo, ResistorIoSchematic, ResistorTileParams,
    TapIo, TapIoSchematic, TapTileParams, TileKind,
};
use atoll::abs::TrackCoord;
use atoll::grid::AtollLayer;
use atoll::route::{GreedyRouter, ViaMaker};
use atoll::straps::{GreedyStrapper, LayerStrappingParams, StrappingParams};
use atoll::{IoBuilder, Orientation, Tile, TileBuilder};
use serde::{Deserialize, Serialize};
use std::any::Any;
use std::marker::PhantomData;
use substrate::arcstr::ArcStr;
use substrate::block::Block;
use substrate::error::Result;
use substrate::geometry::align::{AlignMode, AlignRect};
use substrate::geometry::bbox::Bbox;
use substrate::geometry::corner::Corner;
use substrate::geometry::dir::Dir;
use substrate::geometry::point::Point;
use substrate::geometry::rect::Rect;
use substrate::geometry::sign::Sign;
use substrate::geometry::span::Span;
use substrate::geometry::transform::Translate;
use substrate::io::layout::IoShape;
use substrate::io::{Array, InOut, Input, Io, MosIo, MosIoSchematic, Output, Signal};
use substrate::layout::bbox::LayerBbox;
use substrate::layout::element::Shape;
use substrate::layout::tracks::RoundingMode;
use substrate::layout::{ExportsLayoutData, Layout, LayoutData};
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
    /// The pull-down control (inverted).
    pub pd_ctlb: Input<Signal>,
    /// The VDD rail.
    pub vdd: InOut<Signal>,
    /// The VSS rail.
    pub vss: InOut<Signal>,
}

/// The parameters of a driver unit schematic/layout generator.
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
    /// The number of legs of the resistors.
    pub res_legs: i64,
    /// The width of the resistors.
    pub res_w: i64,
    /// The length of the pull-down resistor.
    pub pd_res_l: i64,
    /// The connection type of the pull-down resistor.
    pub pd_res_conn: ResistorConn,
    /// The length of the pull-up resistor.
    pub pu_res_l: i64,
    /// The connection type of the pull-up resistor.
    pub pu_res_conn: ResistorConn,
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
pub struct DriverWithGuardRingRailsIo {
    /// The buffer input.
    pub din: Input<Signal>,
    /// The buffered output.
    pub dout: Output<Signal>,
    /// The pull-up control.
    pub pu_ctl: Array<Input<Signal>>,
    /// The pull-down control (inverted).
    pub pd_ctlb: Array<Input<Signal>>,
    /// The VDD rail.
    pub vdd: InOut<Signal>,
    /// The VSS rail.
    pub vss: InOut<Signal>,
    /// The guard ring VDD rail.
    pub guard_ring_vdd: InOut<Signal>,
    /// The guard ring VSS rail.
    pub guard_ring_vss: InOut<Signal>,
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
    /// The pull-down control (inverted).
    pub pd_ctlb: Array<Input<Signal>>,
    /// The VDD rail.
    pub vdd: InOut<Signal>,
    /// The VSS rail.
    pub vss: InOut<Signal>,
}

/// The parameters of the horizontal and vertical driver generators.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct DriverParams {
    /// Parameters of the driver unit.
    pub unit: DriverUnitParams,
    /// Number of segments.
    pub num_segments: usize,
    /// Number of banks.
    pub banks: usize,
}

/// A horizontal driver implementation.
pub trait HorizontalDriverImpl<PDK: Pdk + Schema> {
    /// The MOS tile.
    type MosTile: Tile<PDK> + Block<Io = MosIo> + Clone;
    /// The tap tile.
    type TapTile: Tile<PDK> + Block<Io = TapIo> + Clone;
    /// A filler layout cell.
    type Filler: Layout<PDK>;
    /// A guard ring tile.
    type GuardRingTile: Tile<PDK> + Block<Io = TapIo> + Clone;
    /// The resistor tile.
    type ResistorTile: Tile<PDK> + Block<Io = ResistorIo> + Clone;
    /// A PDK-specific via maker.
    type ViaMaker: ViaMaker<PDK>;
    /// The `pu_ctl`/`pu_ctlb` pin layer for driver unit cells.
    type Pin: HasPin;
    /// Height of guard ring top and bottom sides in layer 1 tracks.
    const GUARD_RING_ANNULAR_HEIGHT: i64;
    /// Width of the bump rectangle.
    const BUMP_RECT_WIDTH: i64;

    /// Creates an instance of the MOS tile.
    fn mos(kind: TileKind, max_nf: i64, w: i64) -> Self::MosTile;
    /// Creates an instance of the MOS tile for the driver transistors.
    fn driver_mos(kind: TileKind, max_nf: i64, w: i64) -> Self::MosTile;
    /// Creates an instance of the tap tile.
    fn tap(kind: TileKind, nf: i64) -> Self::TapTile;
    /// The number of fingers needed for the MOS tile to match the width of the resistor tile.
    ///
    /// Must return an even number of fingers.
    fn nf(legs: i64, w: i64) -> i64;
    /// Creates an instance of the resistor tile.
    fn resistor(legs: i64, w: i64, l: i64, conn: ResistorConn) -> Self::ResistorTile;
    /// Creates a filler to be placed around the edge of the guard ring with height given in layer 1 tracks.
    fn filler(kind: TileKind, height: i64) -> Self::Filler;
    /// Returns the filler boundary layer ID.
    fn filler_boundary_id(layers: &PdkLayers<PDK>) -> LayerId;
    /// Creates a guard ring around the given number of horizontally-arrayed MOS devices,
    /// each with the given `nf`. `height` gives the height of the contained devices in layer 1 tracks.
    fn guard_ring(kind: TileKind, n_device: i64, nf: i64, height: i64) -> Self::GuardRingTile;
    /// Creates a PDK-specific via maker.
    fn via_maker() -> Self::ViaMaker;
    /// Returns the `pu_ctl`/`pu_ctlb` pin layer.
    fn pin(layers: &PdkLayers<PDK>) -> Self::Pin;
    /// Draws a dummy MOS with the given position/orientation.
    fn draw_dummy_mos(
        cell: &mut TileBuilder<'_, PDK>,
        kind: TileKind,
        nf: i64,
        w: i64,
        loc: Point,
        orientation: Orientation,
    ) -> Result<()>;
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
    /// The `din`/`dout` pin layer for driver unit cells.
    type Pin: HasPin;

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
    /// Returns the `din`/`dout` pin layer.
    fn pin(layers: &PdkLayers<PDK>) -> Self::Pin;
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

/// Layout data returned by the [`HorizontalDriverUnit`] layout generator.
#[derive(LayoutData)]
pub struct HorizontalDriverUnitLayoutData {
    /// Bounding box of the driver pull-down transistor.
    pub driver_pd_bbox: Rect,
    /// Bounding box of the driver pull-up transistor.
    pub driver_pu_bbox: Rect,
    /// Bounding boxes of the driver n-taps.
    pub driver_ntap_bboxes: Vec<Rect>,
    /// Bounding boxes of the driver p-taps.
    pub driver_ptap_bboxes: Vec<Rect>,
    /// The `dout` pin geometry located on layer 3.
    pub dout: Rect,
    /// Bounding boxes of geometry that requires fillers on the edges
    /// (i.e. not surrounded by guard ring).
    pub filler_bboxes: Vec<Rect>,
    /// Bounding boxes of geometry that requires n-well fillers on the edges
    /// (i.e. not surrounded by guard ring).
    pub nwell_filler_bboxes: Vec<Rect>,
}

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
    type LayoutData = HorizontalDriverUnitLayoutData;
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
        let nf = T::nf(self.0.res_legs, self.0.res_w);

        // Intermediate nodes in the NOR/NAND gates.
        let nor_x = cell.signal("nor_x", Signal::new());
        let nand_x = cell.signal("nand_x", Signal::new());

        // Signals to gates of pull-up and pull-down transistors.
        let pd_en = cell.signal("pd_en", Signal::new());
        let pu_en = cell.signal("pu_en", Signal::new());

        // Intermediate signals between pull-up/pull-down transistors and resistors.
        let pd_x = cell.signal("pd_x", Signal::new());
        let pu_x = cell.signal("pu_x", Signal::new());

        let mos = |kind, w| T::mos(kind, nf, w);
        let driver_mos = |kind, w| T::driver_mos(kind, nf, w);

        // Instantiate all transistors.
        let mut nor_pu_en = cell
            .generate_connected(
                mos(TileKind::P, self.0.nor_pu_en_w),
                MosIoSchematic {
                    d: io.schematic.vdd,
                    g: io.schematic.pd_ctlb,
                    s: nor_x,
                    b: io.schematic.vdd,
                },
            )
            .orient(Orientation::ReflectVert);
        let mut nor_pu_data = cell
            .generate_connected(
                mos(TileKind::P, self.0.nor_pu_data_w),
                MosIoSchematic {
                    d: nor_x,
                    g: io.schematic.din,
                    s: pd_en,
                    b: io.schematic.vdd,
                },
            )
            .orient(Orientation::ReflectVert);
        let mut nor_pd_en = cell.generate_connected(
            mos(TileKind::N, self.0.nor_pd_en_w),
            MosIoSchematic {
                d: pd_en,
                g: io.schematic.pd_ctlb,
                s: io.schematic.vss,
                b: io.schematic.vss,
            },
        );
        let mut nor_pd_data = cell.generate_connected(
            mos(TileKind::N, self.0.nor_pd_data_w),
            MosIoSchematic {
                d: pd_en,
                g: io.schematic.din,
                s: io.schematic.vss,
                b: io.schematic.vss,
            },
        );
        let mut driver_pd = cell.generate_connected(
            driver_mos(TileKind::N, self.0.driver_pd_w),
            MosIoSchematic {
                d: pd_x,
                g: pd_en,
                s: io.schematic.vss,
                b: io.schematic.vss,
            },
        );
        let mut pd_res = cell.generate_connected(
            T::resistor(
                self.0.res_legs,
                self.0.res_w,
                self.0.pd_res_l,
                self.0.pd_res_conn,
            ),
            ResistorIoSchematic {
                p: io.schematic.dout,
                n: pd_x,
                b: io.schematic.vdd,
            },
        );
        let mut pu_res = cell
            .generate_connected(
                T::resistor(
                    self.0.res_legs,
                    self.0.res_w,
                    self.0.pu_res_l,
                    self.0.pu_res_conn,
                ),
                ResistorIoSchematic {
                    p: io.schematic.dout,
                    n: pu_x,
                    b: io.schematic.vdd,
                },
            )
            .orient(Orientation::ReflectVert);
        let mut driver_pu = cell
            .generate_connected(
                driver_mos(TileKind::P, self.0.driver_pu_w),
                MosIoSchematic {
                    d: pu_x,
                    g: pu_en,
                    s: io.schematic.vdd,
                    b: io.schematic.vdd,
                },
            )
            .orient(Orientation::ReflectVert);
        let mut nand_pu_en = cell
            .generate_connected(
                mos(TileKind::P, self.0.nand_pu_en_w),
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
                mos(TileKind::P, self.0.nand_pu_data_w),
                MosIoSchematic {
                    d: pu_en,
                    g: io.schematic.din,
                    s: io.schematic.vdd,
                    b: io.schematic.vdd,
                },
            )
            .orient(Orientation::ReflectVert);
        let mut nand_pd_en = cell.generate_connected(
            mos(TileKind::N, self.0.nand_pd_en_w),
            MosIoSchematic {
                d: io.schematic.vss,
                g: io.schematic.pu_ctl,
                s: nand_x,
                b: io.schematic.vss,
            },
        );
        let mut nand_pd_data = cell.generate_connected(
            mos(TileKind::N, self.0.nand_pd_data_w),
            MosIoSchematic {
                d: nand_x,
                g: io.schematic.din,
                s: pu_en,
                b: io.schematic.vss,
            },
        );

        // Instantiate all taps.
        let mut ntap_nor = cell.generate(T::tap(TileKind::N, nf));
        let mut ptap_nor = cell.generate(T::tap(TileKind::P, nf));
        let mut ptap_driver_bot = cell.generate(T::tap(TileKind::P, nf));
        let mut ptap_driver_top = cell.generate(T::tap(TileKind::P, nf));
        let mut ntap_driver_bot = cell.generate(T::tap(TileKind::N, nf));
        let mut ntap_driver_top = cell.generate(T::tap(TileKind::N, nf));
        let mut ntap_nand = cell.generate(T::tap(TileKind::N, nf));
        let ptap_nand = cell.generate(T::tap(TileKind::P, nf));
        for tap in [&ntap_nor, &ntap_driver_bot, &ntap_driver_top, &ntap_nand] {
            cell.connect(tap.io().x, io.schematic.vdd);
        }
        for tap in [&ptap_nor, &ptap_driver_bot, &ptap_driver_top, &ptap_nand] {
            cell.connect(tap.io().x, io.schematic.vss);
        }

        // Place NAND gate.
        nand_pd_en.align_mut(&ptap_nand, AlignMode::Left, 0);
        nand_pd_en.align_mut(&ptap_nand, AlignMode::Beneath, 0);
        nand_pd_data.align_mut(&nand_pd_en, AlignMode::Left, 0);
        nand_pd_data.align_mut(&nand_pd_en, AlignMode::Beneath, 0);
        nand_pu_data.align_mut(&nand_pd_data, AlignMode::Left, 0);
        nand_pu_data.align_mut(&nand_pd_data, AlignMode::Beneath, 0);
        nand_pu_en.align_mut(&nand_pu_data, AlignMode::Left, 0);
        nand_pu_en.align_mut(&nand_pu_data, AlignMode::Beneath, 0);
        ntap_nand.align_mut(&nand_pu_en, AlignMode::Left, 0);
        ntap_nand.align_mut(&nand_pu_en, AlignMode::Beneath, 0);

        // Place pull-up transistor and taps.
        ntap_driver_top.align_mut(&ntap_nand, AlignMode::Left, 0);
        ntap_driver_top.align_mut(
            &ntap_nand,
            AlignMode::Beneath,
            -T::GUARD_RING_ANNULAR_HEIGHT,
        );
        driver_pu.align_mut(&ntap_driver_top, AlignMode::Left, 0);
        driver_pu.align_mut(&ntap_driver_top, AlignMode::Beneath, 0);
        ntap_driver_bot.align_mut(&driver_pu, AlignMode::Left, 0);
        ntap_driver_bot.align_mut(&driver_pu, AlignMode::Beneath, 0);

        // Place resistors.
        pu_res.align_mut(&ntap_driver_bot, AlignMode::Left, 0);
        pu_res.align_mut(
            &ntap_driver_bot,
            AlignMode::Beneath,
            -T::GUARD_RING_ANNULAR_HEIGHT,
        );
        pd_res.align_mut(&pu_res, AlignMode::Left, 0);
        pd_res.align_mut(&pu_res, AlignMode::Beneath, 0);

        // Place pull-down transistor.
        ptap_driver_top.align_mut(&pd_res, AlignMode::Left, 0);
        ptap_driver_top.align_mut(&pd_res, AlignMode::Beneath, -T::GUARD_RING_ANNULAR_HEIGHT);
        driver_pd.align_mut(&ptap_driver_top, AlignMode::Left, 0);
        driver_pd.align_mut(&ptap_driver_top, AlignMode::Beneath, 0);
        ptap_driver_bot.align_mut(&driver_pd, AlignMode::Left, 0);
        ptap_driver_bot.align_mut(&driver_pd, AlignMode::Beneath, 0);

        // Place NOR gate.
        ptap_nor.align_mut(&ptap_driver_bot, AlignMode::Left, 0);
        ptap_nor.align_mut(
            &ptap_driver_bot,
            AlignMode::Beneath,
            -T::GUARD_RING_ANNULAR_HEIGHT,
        );
        nor_pd_en.align_mut(&ptap_nor, AlignMode::Left, 0);
        nor_pd_en.align_mut(&ptap_nor, AlignMode::Beneath, 0);
        nor_pd_data.align_mut(&nor_pd_en, AlignMode::Left, 0);
        nor_pd_data.align_mut(&nor_pd_en, AlignMode::Beneath, 0);
        nor_pu_data.align_mut(&nor_pd_data, AlignMode::Left, 0);
        nor_pu_data.align_mut(&nor_pd_data, AlignMode::Beneath, 0);
        nor_pu_en.align_mut(&nor_pu_data, AlignMode::Left, 0);
        nor_pu_en.align_mut(&nor_pu_data, AlignMode::Beneath, 0);
        ntap_nor.align_mut(&nor_pu_en, AlignMode::Left, 0);
        ntap_nor.align_mut(&nor_pu_en, AlignMode::Beneath, 0);

        // Block layer 0 where guard ring will be present.
        for (top, bot) in [
            (ntap_nand.lcm_bounds(), ntap_driver_top.lcm_bounds()),
            (ntap_driver_bot.lcm_bounds(), pu_res.lcm_bounds()),
            (pd_res.lcm_bounds(), ptap_driver_top.lcm_bounds()),
            (ptap_driver_bot.lcm_bounds(), ptap_nor.lcm_bounds()),
        ] {
            cell.assign_grid_points(
                None,
                0,
                Rect::from_spans(top.hspan(), Span::new(bot.top(), top.bot())),
            );
        }

        // Draw transistors.
        let _nor_pd_en = cell.draw(nor_pd_en)?;
        let nor_pd_data = cell.draw(nor_pd_data)?;
        let _nor_pu_en = cell.draw(nor_pu_en)?;
        let nor_pu_data = cell.draw(nor_pu_data)?;
        let driver_pd = cell.draw(driver_pd)?;
        let pd_res = cell.draw(pd_res)?;
        let pu_res = cell.draw(pu_res)?;
        let driver_pu = cell.draw(driver_pu)?;
        let _nand_pd_en = cell.draw(nand_pd_en)?;
        let nand_pd_data = cell.draw(nand_pd_data)?;
        let _nand_pu_en = cell.draw(nand_pu_en)?;
        let nand_pu_data = cell.draw(nand_pu_data)?;

        // Draw taps.
        let ntap_nor = cell.draw(ntap_nor)?;
        let ptap_nor = cell.draw(ptap_nor)?;
        let ptap_driver_top = cell.draw(ptap_driver_top)?;
        let ptap_driver_bot = cell.draw(ptap_driver_bot)?;
        let ntap_driver_top = cell.draw(ntap_driver_top)?;
        let ntap_driver_bot = cell.draw(ntap_driver_bot)?;
        let ntap_nand = cell.draw(ntap_nand)?;
        let ptap_nand = cell.draw(ptap_nand)?;

        cell.set_top_layer(3);
        cell.set_router(GreedyRouter::with_seed([1; 32]));
        cell.set_via_maker(T::via_maker());

        // Route `dout` to layer 3.
        let virtual_layers = cell.layout.ctx.install_layers::<atoll::VirtualLayers>();
        let bbox = cell.layout.layer_bbox(virtual_layers.outline.id()).unwrap();
        let center_track_y = cell.layer_stack.layers[3]
            .inner
            .tracks()
            .to_track_idx(bbox.center().y, RoundingMode::Nearest);
        let center_track_x = cell.layer_stack.layers[2]
            .inner
            .tracks()
            .to_track_idx(bbox.center().x, RoundingMode::Nearest);
        let dout_rect = Rect::from_spans(
            cell.layer_stack.layers[2]
                .inner
                .tracks()
                .get(center_track_x),
            cell.layer_stack.layers[3]
                .inner
                .tracks()
                .get(center_track_y),
        );
        cell.assign_grid_points(
            Some(io.schematic.dout),
            3,
            cell.layer_stack
                .slice(0..4)
                .shrink_to_lcm_units(dout_rect)
                .unwrap(),
        );
        cell.layout
            .draw(Shape::new(cell.layer_stack.layers[3].id, dout_rect))?;

        // Route `pu_ctl` and `pd_ctlb` to layer 2 at bottom of unit.
        let bot_track_y = cell.layer_stack.layers[3]
            .inner
            .tracks()
            .to_track_idx(bbox.bot(), RoundingMode::Up);
        let left_track_x = cell.layer_stack.layers[2]
            .inner
            .tracks()
            .to_track_idx(bbox.left(), RoundingMode::Up);

        for (i, (port, layout)) in [
            (io.schematic.pu_ctl, &mut io.layout.pu_ctl),
            (io.schematic.pd_ctlb, &mut io.layout.pd_ctlb),
        ]
        .into_iter()
        .enumerate()
        {
            let y_track_idx = bot_track_y + 1;
            let x_track_idx = left_track_x + 1 + i as i64;
            let y_track = cell.layer_stack.layers[3].inner.tracks().get(y_track_idx);
            let x_track = cell.layer_stack.layers[2].inner.tracks().get(x_track_idx);
            cell.layout.draw(Shape::new(
                cell.layer_stack.layers[2].id,
                Rect::from_spans(x_track, y_track),
            ))?;
            cell.assign_grid_points(
                Some(port),
                2,
                Rect::from_point(Point::new(x_track_idx, y_track_idx)),
            );
            layout.push(IoShape::with_layers(
                T::pin(&cell.ctx().layers),
                Rect::from_spans(x_track, y_track),
            ));
        }

        io.layout.din.merge(nor_pd_data.layout.io().g);
        io.layout.dout.merge(pu_res.layout.io().p);
        io.layout.vdd.merge(ntap_driver_top.layout.io().x);
        io.layout.vss.merge(ptap_driver_bot.layout.io().x);

        // Route these signals by straps at a higher level in the hierarchy.
        cell.skip_routing_all(io.schematic.vss);
        cell.skip_routing_all(io.schematic.vdd);
        cell.skip_routing_all(io.schematic.din);

        T::post_layout_hooks(cell)?;

        Ok((
            (),
            HorizontalDriverUnitLayoutData {
                driver_pd_bbox: driver_pd.layout.bbox_rect(),
                driver_pu_bbox: driver_pu.layout.bbox_rect(),
                driver_ntap_bboxes: vec![
                    ntap_driver_bot.layout.bbox_rect(),
                    ntap_driver_top.layout.bbox_rect(),
                ],
                driver_ptap_bboxes: vec![
                    ptap_driver_bot.layout.bbox_rect(),
                    ptap_driver_top.layout.bbox_rect(),
                ],
                dout: dout_rect,
                filler_bboxes: [
                    (
                        &ptap_nand.layout.bbox_rect(),
                        &nand_pd_data.layout.bbox_rect(),
                    ),
                    (
                        &nor_pd_data.layout.bbox_rect(),
                        &ptap_nor.layout.bbox_rect(),
                    ),
                ]
                .into_iter()
                .map(|(a, b)| a.union(*b))
                .collect(),
                nwell_filler_bboxes: [
                    (
                        &ntap_nand.layout.bbox_rect(),
                        &nand_pu_data.layout.bbox_rect(),
                    ),
                    (
                        &nor_pu_data.layout.bbox_rect(),
                        &ntap_nor.layout.bbox_rect(),
                    ),
                    (&pu_res.layout.bbox_rect(), &pd_res.layout.bbox_rect()),
                ]
                .into_iter()
                .map(|(a, b)| a.union(*b))
                .collect(),
            },
        ))
    }
}

/// A horizontal driver with separated guard ring rails.
#[derive_where::derive_where(Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[derive(Serialize, Deserialize)]
pub struct HorizontalDriverWithGuardRingRails<T>(
    DriverParams,
    #[serde(bound(deserialize = ""))] PhantomData<fn() -> T>,
);

impl<T> HorizontalDriverWithGuardRingRails<T> {
    /// Creates a new [`HorizontalDriverWithGuardRingRails`].
    pub fn new(params: DriverParams) -> Self {
        Self(params, PhantomData)
    }
}

impl<T: Any> Block for HorizontalDriverWithGuardRingRails<T> {
    type Io = DriverWithGuardRingRailsIo;

    fn id() -> ArcStr {
        substrate::arcstr::literal!("horizontal_driver")
    }

    // todo: include parameters in name
    fn name(&self) -> ArcStr {
        substrate::arcstr::literal!("horizontal_driver")
    }

    fn io(&self) -> Self::Io {
        DriverWithGuardRingRailsIo {
            din: Default::default(),
            dout: Default::default(),
            pu_ctl: Array::new(self.0.num_segments, Default::default()),
            pd_ctlb: Array::new(self.0.num_segments, Default::default()),
            vdd: Default::default(),
            vss: Default::default(),
            guard_ring_vdd: Default::default(),
            guard_ring_vss: Default::default(),
        }
    }
}

impl<T: Any> ExportsNestedData for HorizontalDriverWithGuardRingRails<T> {
    type NestedData = ();
}

/// Layout data returned by the [`HorizontalDriverWithGuardRingRails`] layout generator.
#[derive(LayoutData)]
pub struct HorizontalDriverWithGuardRingRailsLayoutData {
    /// The `dout` pin geometry located on layer 7.
    pub dout: Vec<Rect>,
}

impl<T: Any> ExportsLayoutData for HorizontalDriverWithGuardRingRails<T> {
    type LayoutData = HorizontalDriverWithGuardRingRailsLayoutData;
}

impl<PDK: Pdk + Schema + Sized, T: HorizontalDriverImpl<PDK> + Any> Tile<PDK>
    for HorizontalDriverWithGuardRingRails<T>
{
    fn tile<'a>(
        &self,
        io: IoBuilder<'a, Self>,
        cell: &mut TileBuilder<'a, PDK>,
    ) -> substrate::error::Result<(
        <Self as ExportsNestedData>::NestedData,
        <Self as ExportsLayoutData>::LayoutData,
    )> {
        let mut units = Vec::new();
        // Instantiate driver units.
        for i in 0..self.0.num_segments + 2 {
            let mut unit = cell.generate_connected(
                HorizontalDriverUnit::<T>::new(self.0.unit),
                DriverUnitIoSchematic {
                    din: io.schematic.din,
                    dout: io.schematic.dout,
                    pu_ctl: if i == 0 || i == self.0.num_segments + 1 {
                        io.schematic.vss
                    } else {
                        io.schematic.pu_ctl[i - 1]
                    },
                    pd_ctlb: if i == 0 || i == self.0.num_segments + 1 {
                        io.schematic.vdd
                    } else {
                        io.schematic.pd_ctlb[i - 1]
                    },
                    vdd: io.schematic.vdd,
                    vss: io.schematic.vss,
                },
            );
            if let Some(prev) = units.last() {
                unit.align_mut(prev, AlignMode::ToTheRight, 0);
                unit.align_mut(prev, AlignMode::Bottom, 0);
            }
            units.push(unit);
        }

        // Draw driver units.
        let units = units
            .into_iter()
            .enumerate()
            .map(|(i, unit)| {
                let unit = cell.draw(unit)?;
                io.layout.din.merge(unit.layout.io().din);
                io.layout.dout.merge(unit.layout.io().dout);
                if i > 0 && i < self.0.num_segments + 1 {
                    io.layout.pu_ctl[i - 1].merge(unit.layout.io().pu_ctl);
                    io.layout.pd_ctlb[i - 1].merge(unit.layout.io().pd_ctlb);
                }
                io.layout.vdd.merge(unit.layout.io().vdd);
                io.layout.vss.merge(unit.layout.io().vss);
                Ok(unit)
            })
            .collect::<Result<Vec<_>>>()?;

        // Fill in extra dummies and taps for continuous diffusion for pull-up/pull-down transistors.
        let nf = T::nf(self.0.unit.res_legs, self.0.unit.res_w);
        for unit in units.iter().take(self.0.num_segments + 1) {
            // Draw dummy transistors.
            let pu_bbox = unit.layout.data().driver_pu_bbox;
            let pu_loc = Rect::from_xy(pu_bbox.right(), pu_bbox.center().y);
            T::draw_dummy_mos(
                cell,
                TileKind::P,
                2,
                self.0.unit.driver_pu_w,
                pu_loc.center(),
                Orientation::ReflectVert,
            )?;
            let pd_bbox = unit.layout.data().driver_pd_bbox;
            let pd_loc = Rect::from_xy(pd_bbox.right(), pd_bbox.center().y);
            T::draw_dummy_mos(
                cell,
                TileKind::N,
                2,
                self.0.unit.driver_pd_w,
                pd_loc.center(),
                Orientation::R0,
            )?;

            // Draw additional taps.
            for (tap_bbox, kind, node) in unit
                .layout
                .data()
                .driver_ntap_bboxes
                .iter()
                .map(|bbox| (*bbox, TileKind::N, io.schematic.vdd))
                .chain(
                    unit.layout
                        .data()
                        .driver_ptap_bboxes
                        .iter()
                        .map(|bbox| (*bbox, TileKind::P, io.schematic.vss)),
                )
            {
                let tap_loc = cell
                    .layer_stack
                    .slice(0..2)
                    .expand_to_lcm_units(Rect::from_xy(tap_bbox.right(), tap_bbox.center().y));
                let tap = cell
                    .generate_connected(T::tap(kind, 2), TapIoSchematic { x: node })
                    .orient(Orientation::ReflectVert)
                    .align_rect(tap_loc, AlignMode::CenterVertical, 0)
                    .align_rect(tap_loc, AlignMode::CenterHorizontal, 0);

                let _tap = cell.draw(tap)?;
            }
        }

        // Add filler on the left and right of layout to account for guard ring.
        for sign in [Sign::Neg, Sign::Pos] {
            let unit = &units[match sign {
                Sign::Neg => 0,
                Sign::Pos => self.0.num_segments + 1,
            }];
            for (bbox, kind) in unit
                .layout
                .data()
                .filler_bboxes
                .into_iter()
                .map(|bbox| (bbox, TileKind::P))
                .chain(
                    unit.layout
                        .data()
                        .nwell_filler_bboxes
                        .into_iter()
                        .map(|bbox| (bbox, TileKind::N)),
                )
            {
                let filler_id = T::filler_boundary_id(&cell.ctx().layers);
                let filler = cell.layout.generate(T::filler(
                    kind,
                    bbox.height() / cell.layer_stack.layer(1).pitch(),
                ));
                let layer_bbox = filler.layer_bbox(filler_id).unwrap();
                let filler = filler
                    .align(
                        match sign {
                            Sign::Neg => AlignMode::ToTheLeft,
                            Sign::Pos => AlignMode::ToTheRight,
                        },
                        layer_bbox,
                        bbox,
                        0,
                    )
                    .align(AlignMode::Bottom, layer_bbox, bbox, 0);
                cell.layout.draw(filler)?;
            }
        }

        let pu_bbox = units[0]
            .layout
            .data()
            .driver_pu_bbox
            .union(units[self.0.num_segments + 1].layout.data().driver_pu_bbox);
        let pd_bbox = units[0]
            .layout
            .data()
            .driver_pd_bbox
            .union(units[self.0.num_segments + 1].layout.data().driver_pd_bbox);

        // Draw pull-up and pull-down guard rings.
        let mut guard_rings = Vec::new();
        for (bbox, kind, node) in [
            (pu_bbox, TileKind::P, io.schematic.guard_ring_vss),
            (pd_bbox, TileKind::N, io.schematic.guard_ring_vdd),
        ] {
            let bbox_lcm = cell.layer_stack.slice(0..2).expand_to_lcm_units(bbox);
            let guard_ring = cell
                .generate_connected(
                    T::guard_ring(
                        kind,
                        (self.0.num_segments + 2) as i64,
                        nf,
                        bbox.height() / cell.layer_stack.layer(1).pitch(),
                    ),
                    TapIoSchematic { x: node },
                )
                .align_rect(bbox_lcm, AlignMode::CenterVertical, 0)
                .align_rect(bbox_lcm, AlignMode::CenterHorizontal, 0);
            guard_rings.push(cell.draw(guard_ring)?);
        }
        let guard_ring_n = guard_rings.pop().unwrap();
        let guard_ring_p = guard_rings.pop().unwrap();
        io.layout.guard_ring_vdd.merge(guard_ring_n.layout.io().x);
        io.layout.guard_ring_vss.merge(guard_ring_p.layout.io().x);

        let via_maker = T::via_maker();

        // Via up `dout` to layer 7.
        let mut via_stack: Vec<(usize, Shape)> = Vec::new();
        for layer in 4..8 {
            via_stack.extend(
                via_maker
                    .draw_via(cell.ctx().clone(), TrackCoord { layer, x: 0, y: 0 })
                    .into_iter()
                    .map(|shape| (layer, shape)),
            );
        }
        let mut dout = Vec::new();
        for unit in units.iter() {
            let mut unit_dout = Vec::new();
            // Draw vias.
            for (layer, shape) in &via_stack {
                let shape = shape.clone().translate(
                    unit.layout.data().dout.bbox_rect().center() - shape.bbox_rect().center(),
                );
                cell.layout.draw(shape.clone())?;
                if shape.layer() == cell.layer_stack.layers[7].id {
                    unit_dout.push(shape.bbox_rect());
                }

                // Block ample space for each via in the ATOLL routing grid.
                for layer in [*layer, *layer - 1] {
                    let tracks = cell.layer_stack.tracks(layer);
                    let perp_tracks = cell.layer_stack.tracks(layer - 1);
                    let (xtracks, ytracks) = match cell.layer_stack.layer(layer).dir().track_dir() {
                        Dir::Horiz => (perp_tracks, tracks),
                        Dir::Vert => (tracks, perp_tracks),
                    };
                    let bot_track =
                        ytracks.to_track_idx(shape.bbox_rect().bot(), RoundingMode::Down);
                    let top_track = ytracks.to_track_idx(shape.bbox_rect().top(), RoundingMode::Up);
                    let left_track =
                        xtracks.to_track_idx(shape.bbox_rect().left(), RoundingMode::Down);
                    let right_track =
                        xtracks.to_track_idx(shape.bbox_rect().right(), RoundingMode::Up);
                    let assigned_tracks =
                        Rect::from_sides(left_track, bot_track, right_track, top_track);
                    cell.assign_grid_points(None, layer, assigned_tracks);
                }
            }
            dout.push(unit_dout.bbox_rect());
        }

        let top_slice = cell.layer_stack.slice(0..8);
        let overall_bbox = top_slice.expand_to_lcm_units(cell.layout.bbox_rect());
        let physical_overall_bbox = top_slice.lcm_to_physical_rect(overall_bbox);

        let virtual_layers = cell.layout.ctx.install_layers::<atoll::VirtualLayers>();
        cell.layout
            .draw(Shape::new(virtual_layers.outline, physical_overall_bbox))?;

        // Extend ctl pins to edge.
        for i in 1..=self.0.num_segments {
            for port in [units[i].layout.io().pu_ctl, units[i].layout.io().pd_ctlb] {
                let pin_rect = port.primary.bbox_rect();
                let pin_rect =
                    pin_rect.with_vspan(pin_rect.vspan().add_point(physical_overall_bbox.bot()));
                cell.layout
                    .draw(Shape::new(port.primary.layer().drawing(), pin_rect))?;
                cell.assign_grid_points(
                    None,
                    2,
                    cell.layer_stack.slice(0..3).expand_to_lcm_units(pin_rect),
                );
            }
        }

        let top_slice = cell.layer_stack.slice(0..8);

        // Determine strapping domains.
        let guard_ring_p_bbox = top_slice
            .expand_to_lcm_units(Rect::from_spans(
                cell.layout.bbox_rect().hspan(),
                guard_ring_p.layout.bbox_rect().vspan(),
            ))
            .translate(Point::zero() - overall_bbox.corner(Corner::LowerLeft));
        let guard_ring_n_bbox = top_slice
            .expand_to_lcm_units(Rect::from_spans(
                cell.layout.bbox_rect().hspan(),
                guard_ring_n.layout.bbox_rect().vspan(),
            ))
            .translate(Point::zero() - overall_bbox.corner(Corner::LowerLeft));
        let pu_network_bbox = top_slice
            .expand_to_lcm_units(Rect::from_spans(
                cell.layout.bbox_rect().hspan(),
                guard_ring_p
                    .layout
                    .bbox_rect()
                    .vspan()
                    .add_point(cell.layout.bbox_rect().top()),
            ))
            .translate(Point::zero() - overall_bbox.corner(Corner::LowerLeft));
        let pd_network_bbox = top_slice
            .expand_to_lcm_units(Rect::from_spans(
                cell.layout.bbox_rect().hspan(),
                guard_ring_n
                    .layout
                    .bbox_rect()
                    .vspan()
                    .add_point(cell.layout.bbox_rect().bot()),
            ))
            .translate(Point::zero() - overall_bbox.corner(Corner::LowerLeft));

        // Strap guard ring rails only over the appropriate rings.
        cell.set_strapping(
            io.schematic.guard_ring_vss,
            StrappingParams::new(
                1,
                vec![
                    LayerStrappingParams::ViaDown { min_period: 3 },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 3,
                        period: 5,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 0,
                        period: 7,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 5,
                        period: 9,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 0,
                        period: 2,
                    },
                ],
            )
            .with_bounds(guard_ring_p_bbox),
        );
        cell.set_strapping(
            io.schematic.guard_ring_vdd,
            StrappingParams::new(
                1,
                vec![
                    LayerStrappingParams::ViaDown { min_period: 3 },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 3,
                        period: 5,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 0,
                        period: 7,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 5,
                        period: 9,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 0,
                        period: 2,
                    },
                ],
            )
            .with_bounds(guard_ring_n_bbox),
        );

        // Strap `din`.
        cell.set_strapping(
            io.schematic.din,
            StrappingParams::new(
                1,
                vec![
                    LayerStrappingParams::ViaDown { min_period: 1 },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 2,
                        period: 10,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 8,
                        period: 22,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 8,
                        period: 18,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 8,
                        period: 13,
                    },
                ],
            ),
        );

        // Strap VSS with high density on layer 1 over the pull-up/pull-down networks.
        cell.set_strapping(
            io.schematic.vss,
            StrappingParams::new(1, vec![LayerStrappingParams::ViaDown { min_period: 1 }])
                .with_bounds(pu_network_bbox),
        );
        cell.set_strapping(
            io.schematic.vss,
            StrappingParams::new(1, vec![LayerStrappingParams::ViaDown { min_period: 1 }])
                .with_bounds(pd_network_bbox),
        );
        // Strap VSS over the entire driver.
        cell.set_strapping(
            io.schematic.vss,
            StrappingParams::new(
                1,
                vec![
                    LayerStrappingParams::ViaDown { min_period: 3 },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 0,
                        period: 5,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 0,
                        period: 11,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 0,
                        period: 9,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 0,
                        period: 13,
                    },
                ],
            ),
        );
        // Strap VDD with high density on layer 1 over the pull-up/pull-down networks.
        cell.set_strapping(
            io.schematic.vdd,
            StrappingParams::new(1, vec![LayerStrappingParams::ViaDown { min_period: 1 }])
                .with_bounds(pu_network_bbox),
        );
        cell.set_strapping(
            io.schematic.vdd,
            StrappingParams::new(1, vec![LayerStrappingParams::ViaDown { min_period: 1 }])
                .with_bounds(pd_network_bbox),
        );
        // Strap VDD over the entire driver.
        cell.set_strapping(
            io.schematic.vdd,
            StrappingParams::new(
                1,
                vec![
                    LayerStrappingParams::ViaDown { min_period: 3 },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 1,
                        period: 5,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 1,
                        period: 11,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 1,
                        period: 9,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 1,
                        period: 13,
                    },
                ],
            ),
        );

        cell.set_top_layer(7);
        cell.set_strapper(GreedyStrapper);
        cell.set_via_maker(via_maker);

        T::post_layout_hooks(cell)?;

        Ok(((), HorizontalDriverWithGuardRingRailsLayoutData { dout }))
    }
}

/// A horizontal driver.
#[derive_where::derive_where(Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[derive(Serialize, Deserialize)]
pub struct HorizontalDriver<T>(
    DriverParams,
    #[serde(bound(deserialize = ""))] PhantomData<fn() -> T>,
);

impl<T> HorizontalDriver<T> {
    /// Creates a new [`HorizontalDriver`].
    pub fn new(params: DriverParams) -> Self {
        Self(params, PhantomData)
    }
}

impl<T: Any> Block for HorizontalDriver<T> {
    type Io = DriverIo;

    fn id() -> ArcStr {
        substrate::arcstr::literal!("horizontal_driver")
    }

    // todo: include parameters in name
    fn name(&self) -> ArcStr {
        substrate::arcstr::literal!("horizontal_driver")
    }

    fn io(&self) -> Self::Io {
        DriverIo {
            din: Default::default(),
            dout: Default::default(),
            pu_ctl: Array::new(self.0.num_segments * self.0.banks, Default::default()),
            pd_ctlb: Array::new(self.0.num_segments * self.0.banks, Default::default()),
            vdd: Default::default(),
            vss: Default::default(),
        }
    }
}

impl<T: Any> ExportsNestedData for HorizontalDriver<T> {
    type NestedData = ();
}

impl<T: Any> ExportsLayoutData for HorizontalDriver<T> {
    type LayoutData = ();
}

impl<PDK: Pdk + Schema + Sized, T: HorizontalDriverImpl<PDK> + Any> Tile<PDK>
    for HorizontalDriver<T>
{
    fn tile<'a>(
        &self,
        io: IoBuilder<'a, Self>,
        cell: &mut TileBuilder<'a, PDK>,
    ) -> substrate::error::Result<(
        <Self as ExportsNestedData>::NestedData,
        <Self as ExportsLayoutData>::LayoutData,
    )> {
        let mut layer8_vias = vec![Vec::new(); self.0.num_segments + 2];
        let mut prev_bounds: Option<Rect> = None;
        // Instantiate and draw banks.
        for i in 0..self.0.banks {
            let mut driver = cell
                .generate(HorizontalDriverWithGuardRingRails::<T>::new(self.0))
                .orient(if i % 2 == 0 {
                    Orientation::R0
                } else {
                    Orientation::ReflectVert
                });
            if let Some(prev_bounds) = prev_bounds {
                driver.align_rect_mut(prev_bounds, AlignMode::Above, 1);
            }
            prev_bounds = Some(driver.lcm_bounds());

            let driver = cell.draw(driver)?;

            cell.connect(driver.schematic.io().din, io.schematic.din);
            cell.connect(driver.schematic.io().dout, io.schematic.dout);
            cell.connect(driver.schematic.io().vdd, io.schematic.vdd);
            cell.connect(driver.schematic.io().vss, io.schematic.vss);
            cell.connect(driver.schematic.io().guard_ring_vdd, io.schematic.vdd);
            cell.connect(driver.schematic.io().guard_ring_vss, io.schematic.vss);
            io.layout.din.merge(driver.layout.io().din);
            io.layout.dout.merge(driver.layout.io().dout);
            io.layout.vdd.merge(driver.layout.io().vdd);
            io.layout.vss.merge(driver.layout.io().vss);
            for j in 0..self.0.num_segments {
                cell.connect(
                    driver.schematic.io().pu_ctl[j],
                    io.schematic.pu_ctl[self.0.num_segments * i + j],
                );
                cell.connect(
                    driver.schematic.io().pd_ctlb[j],
                    io.schematic.pd_ctlb[self.0.num_segments * i + j],
                );
                io.layout.pu_ctl[self.0.num_segments * i + j]
                    .merge(driver.layout.io().pu_ctl[j].clone());
                io.layout.pd_ctlb[self.0.num_segments * i + j]
                    .merge(driver.layout.io().pd_ctlb[j].clone());
            }

            // Via up `dout` nets from each unit to layer 9 and draw a rectangle connecting them all.
            let via_maker = T::via_maker();
            let bump_rect = Rect::from_spans(
                cell.layout.bbox_rect().hspan(),
                Span::from_center_span(driver.layout.data().dout[0].center().y, T::BUMP_RECT_WIDTH),
            );
            cell.layout
                .draw(Shape::new(cell.layer_stack.layers[9].id, bump_rect))?;
            let mut via_stack = Vec::new();
            for layer in 8..10 {
                via_stack.extend(
                    via_maker.draw_via(cell.ctx().clone(), TrackCoord { layer, x: 0, y: 0 }),
                );
            }
            for (j, dout) in driver.layout.data().dout.into_iter().enumerate() {
                for shape in &via_stack {
                    let shape = shape
                        .clone()
                        .translate(dout.center() - shape.bbox_rect().center());
                    // Track layer 8 vias to strap with other banks.
                    if shape.layer() == cell.layer_stack.layers[8].id {
                        layer8_vias[j].push(shape.bbox_rect());
                    }
                    cell.layout.draw(shape.clone())?;
                }
            }
        }

        // Strap `dout` across banks.
        for vias in layer8_vias {
            cell.layout
                .draw(Shape::new(cell.layer_stack.layers[8].id, vias.bbox_rect()))?;
        }

        // Strap `din`, `vss`, and `vdd`.
        cell.set_strapping(
            io.schematic.din,
            StrappingParams::new(
                6,
                vec![
                    LayerStrappingParams::OffsetPeriod {
                        offset: 5,
                        period: 8,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 5,
                        period: 8,
                    },
                ],
            ),
        );
        cell.set_strapping(
            io.schematic.vss,
            StrappingParams::new(
                6,
                vec![
                    LayerStrappingParams::OffsetPeriod {
                        offset: 2,
                        period: 8,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 2,
                        period: 8,
                    },
                ],
            ),
        );
        cell.set_strapping(
            io.schematic.vdd,
            StrappingParams::new(
                6,
                vec![
                    LayerStrappingParams::OffsetPeriod {
                        offset: 1,
                        period: 8,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 1,
                        period: 8,
                    },
                ],
            ),
        );

        cell.set_top_layer(9);
        cell.set_strapper(GreedyStrapper);
        cell.set_via_maker(T::via_maker());

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
        let nor_pu_en_params = MosTileParams::new(MosKind::Nom, TileKind::P, self.0.nor_pu_en_w);
        let nor_pu_data_params =
            MosTileParams::new(MosKind::Nom, TileKind::P, self.0.nor_pu_data_w);
        let nor_pd_en_params = MosTileParams::new(MosKind::Nom, TileKind::N, self.0.nor_pd_en_w);
        let nor_pd_data_params =
            MosTileParams::new(MosKind::Nom, TileKind::N, self.0.nor_pd_data_w);
        let driver_pd_params = MosTileParams::new(MosKind::Nom, TileKind::N, self.0.driver_pd_w);
        let pd_res_params = ResistorTileParams::new(self.0.pd_res_l);
        let pu_res_params = ResistorTileParams::new(self.0.pu_res_l);
        let driver_pu_params = MosTileParams::new(MosKind::Nom, TileKind::P, self.0.driver_pu_w);
        let nand_pu_en_params = MosTileParams::new(MosKind::Nom, TileKind::P, self.0.nand_pu_en_w);
        let nand_pu_data_params =
            MosTileParams::new(MosKind::Nom, TileKind::P, self.0.nand_pu_data_w);
        let nand_pd_en_params = MosTileParams::new(MosKind::Nom, TileKind::N, self.0.nand_pd_en_w);
        let nand_pd_data_params =
            MosTileParams::new(MosKind::Nom, TileKind::N, self.0.nand_pd_data_w);

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
                g: io.schematic.pd_ctlb,
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
                g: io.schematic.pd_ctlb,
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
        let _nor_pd_data = cell.draw(nor_pd_data)?;
        let _nor_pu_en = cell.draw(nor_pu_en)?;
        let nor_pu_data = cell.draw(nor_pu_data)?;
        let _driver_pd = cell.draw(driver_pd)?;
        let pd_res = cell.draw(pd_res)?;
        let _pu_res = cell.draw(pu_res)?;
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

        let virtual_layers = cell.layout.ctx.install_layers::<atoll::VirtualLayers>();
        let bbox = cell.layout.layer_bbox(virtual_layers.outline.id()).unwrap();

        let layer2 = cell.layer_stack.layers[2].clone();
        // Route `din` along edges of driver.
        let min_track = layer2
            .inner
            .tracks()
            .to_track_idx(bbox.left() + layer2.pitch() + 1, RoundingMode::Up);
        let max_track = layer2
            .inner
            .tracks()
            .to_track_idx(bbox.right() - layer2.pitch() - 1, RoundingMode::Down);
        for track in [min_track, max_track] {
            let track_rect = Rect::from_spans(layer2.inner.tracks().get(track), bbox.vspan());
            cell.layout.draw(Shape::new(layer2.id, track_rect))?;
            cell.assign_grid_points(
                Some(io.schematic.din),
                2,
                cell.layer_stack
                    .slice(0..3)
                    .shrink_to_lcm_units(track_rect)
                    .unwrap(),
            );
            io.layout
                .din
                .push(IoShape::with_layers(T::pin(&cell.ctx().layers), track_rect));
        }

        // Route `dout` to center track.
        let virtual_layers = cell.layout.ctx.install_layers::<atoll::VirtualLayers>();
        let bbox = cell.layout.layer_bbox(virtual_layers.outline.id()).unwrap();
        let center_track_x = layer2
            .inner
            .tracks()
            .to_track_idx(bbox.center().x, RoundingMode::Nearest);
        let center_track_y = cell.layer_stack.layers[1]
            .inner
            .tracks()
            .to_track_idx(bbox.center().y, RoundingMode::Nearest);

        let track_rect = Rect::from_spans(
            layer2.inner.tracks().get(center_track_x),
            cell.layer_stack.layers[1]
                .inner
                .tracks()
                .get(center_track_y),
        );

        cell.assign_grid_points(
            Some(io.schematic.dout),
            2,
            cell.layer_stack
                .slice(0..3)
                .shrink_to_lcm_units(track_rect)
                .unwrap(),
        );
        cell.layout.draw(Shape::new(layer2.id, track_rect))?;
        io.layout
            .dout
            .push(IoShape::with_layers(T::pin(&cell.ctx().layers), track_rect));

        cell.set_top_layer(2);
        cell.set_router(GreedyRouter::new());
        cell.set_via_maker(T::via_maker());

        io.layout.pu_ctl.merge(nor_pd_en.layout.io().g);
        io.layout.pd_ctlb.merge(nand_pd_en.layout.io().g);
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
            pd_ctlb: Array::new(self.0.num_segments, Default::default()),
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
                    pd_ctlb: io.schematic.pd_ctlb[i],
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

        let units = units
            .into_iter()
            .enumerate()
            .map(|(i, unit)| {
                let unit = cell.draw(unit)?;
                io.layout.din.merge(unit.layout.io().din);
                io.layout.dout.merge(unit.layout.io().dout);
                io.layout.pu_ctl[i].merge(unit.layout.io().pu_ctl);
                io.layout.pd_ctlb[i].merge(unit.layout.io().pd_ctlb);
                io.layout.vdd.merge(unit.layout.io().vdd);
                io.layout.vss.merge(unit.layout.io().vss);
                Ok(unit)
            })
            .collect::<Result<Vec<_>>>()?;

        let layer3 = &cell.layer_stack.layers[3];
        let din_connect_track = layer3.inner.tracks().to_track_idx(
            units[0].layout.io().din.bbox_rect().top(),
            RoundingMode::Nearest,
        );
        let din_pin = Rect::from_spans(
            units[0].layout.io().din.bbox_rect().hspan(),
            layer3.inner.tracks().get(din_connect_track),
        );
        cell.layout.draw(Shape::new(layer3.id, din_pin))?;
        let via_maker = T::via_maker();
        for shape in units[0].layout.io().din.shapes() {
            let x_track = cell.layer_stack.layers[2]
                .inner
                .tracks()
                .to_track_idx(shape.bbox_rect().center().x, RoundingMode::Nearest);
            for shape in via_maker.draw_via(
                cell.ctx().clone(),
                TrackCoord {
                    layer: 3,
                    x: x_track,
                    y: din_connect_track,
                },
            ) {
                cell.layout.draw(shape)?;
            }
        }

        let bump_rect = Rect::from_spans(
            Span::from_center_span(units[0].layout.io().dout.bbox_rect().center().x, 1080),
            cell.layout.bbox_rect().vspan(),
        );
        cell.layout
            .draw(Shape::new(cell.layer_stack.layers[8].id, bump_rect))?;

        let mut via_stack = Vec::new();
        for layer in 3..9 {
            via_stack
                .extend(via_maker.draw_via(cell.ctx().clone(), TrackCoord { layer, x: 0, y: 0 }))
        }
        for unit in units.iter() {
            for shape in &via_stack {
                cell.layout.draw(shape.clone().translate(
                    unit.layout.io().dout.bbox_rect().center() - shape.bbox_rect().center(),
                ))?;
            }
        }

        cell.set_top_layer(3);

        T::post_layout_hooks(cell)?;

        Ok(((), ()))
    }
}
