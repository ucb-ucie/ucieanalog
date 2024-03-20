//! Driver layout generators.

pub mod tb;

use crate::tiles::{
    MosTileParams, ResistorConn, ResistorIo, ResistorIoSchematic, ResistorTileParams, TapIo,
    TapIoSchematic, TapTileParams, TileKind,
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
use substrate::geometry::align::{AlignBbox, AlignMode};
use substrate::geometry::bbox::Bbox;
use substrate::geometry::dir::Dir;
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
    /// The pull-down control.
    pub pd_ctl: Input<Signal>,
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

/// The parameters of the horizontal and vertical driver generators.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct DriverParams {
    /// Parameters of the driver unit.
    pub unit: DriverUnitParams,
    /// Number of segments.
    pub num_segments: usize,
}

/// A horizontal driver implementation.
pub trait HorizontalDriverImpl<PDK: Pdk + Schema> {
    /// The MOS tile.
    type MosTile: Tile<PDK> + Block<Io = MosIo> + Clone;
    /// The MOS tile used as dummies between pull-up/pull-down transistors
    /// on the same row.
    ///
    /// All 4 ports should be tied to the same node.
    type TiedMosTile: Tile<PDK> + Block<Io = Signal> + Clone;
    /// The tap tile.
    type TapTile: Tile<PDK> + Block<Io = TapIo> + Clone;
    /// A filler layout cell.
    type Filler: Layout<PDK>;
    /// A guard ring layout cell.
    type GuardRing: Layout<PDK>;
    /// The resistor tile.
    type ResistorTile: Tile<PDK> + Block<Io = ResistorIo> + Clone;
    /// A PDK-specific via maker.
    type ViaMaker: ViaMaker<PDK>;
    /// Height of guard ring top and bottom sides in layer 1 tracks.
    const GUARD_RING_ANNULAR_HEIGHT: i64;

    /// Creates an instance of the MOS tile.
    fn mos(kind: TileKind, nf: i64, w: i64) -> Self::MosTile;
    /// Creates an instance of the tied MOS tile.
    fn tied_mos(kind: TileKind, nf: i64, w: i64) -> Self::TiedMosTile;
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
    /// Creates a guard ring around the given number of horizontally-arrayed MOS devices,
    /// each with the given `nf`. `height` gives the height of the contained devices in layer 1 tracks.
    fn guard_ring(kind: TileKind, n_device: i64, nf: i64, height: i64) -> Self::GuardRing;
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
        assert_eq!(nf % 2, 0);

        let nor_x = cell.signal("nor_x", Signal::new());
        let nand_x = cell.signal("nand_x", Signal::new());
        let pd_en = cell.signal("pd_en", Signal::new());
        let pu_en = cell.signal("pu_en", Signal::new());
        let pd_x = cell.signal("pd_x", Signal::new());
        let pu_x = cell.signal("pu_x", Signal::new());

        let mos = |kind, w| T::mos(kind, nf, (w - 1) / nf + 1);

        let mut nor_pu_en = cell
            .generate_connected(
                mos(TileKind::P, self.0.nor_pu_en_w),
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
                g: io.schematic.pd_ctl,
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
            mos(TileKind::N, self.0.driver_pd_w),
            MosIoSchematic {
                d: pd_x,
                g: io.schematic.din,
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
                mos(TileKind::P, self.0.driver_pu_w),
                MosIoSchematic {
                    d: pu_x,
                    g: io.schematic.din,
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

        pu_res.align_mut(&ntap_driver_bot, AlignMode::Left, 0);
        pu_res.align_mut(
            &ntap_driver_bot,
            AlignMode::Beneath,
            -T::GUARD_RING_ANNULAR_HEIGHT,
        );

        pd_res.align_mut(&pu_res, AlignMode::Left, 0);
        pd_res.align_mut(&pu_res, AlignMode::Beneath, 0);

        ptap_driver_top.align_mut(&pd_res, AlignMode::Left, 0);
        ptap_driver_top.align_mut(&pd_res, AlignMode::Beneath, -T::GUARD_RING_ANNULAR_HEIGHT);
        driver_pd.align_mut(&ptap_driver_top, AlignMode::Left, 0);
        driver_pd.align_mut(&ptap_driver_top, AlignMode::Beneath, 0);
        ptap_driver_bot.align_mut(&driver_pd, AlignMode::Left, 0);
        ptap_driver_bot.align_mut(&driver_pd, AlignMode::Beneath, 0);

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

        let nor_pd_en = cell.draw(nor_pd_en)?;
        let nor_pd_data = cell.draw(nor_pd_data)?;
        let _nor_pu_en = cell.draw(nor_pu_en)?;
        let nor_pu_data = cell.draw(nor_pu_data)?;
        let driver_pd = cell.draw(driver_pd)?;
        let pd_res = cell.draw(pd_res)?;
        let pu_res = cell.draw(pu_res)?;
        let driver_pu = cell.draw(driver_pu)?;
        let nand_pd_en = cell.draw(nand_pd_en)?;
        let nand_pd_data = cell.draw(nand_pd_data)?;
        let _nand_pu_en = cell.draw(nand_pu_en)?;
        let nand_pu_data = cell.draw(nand_pu_data)?;

        let ntap_nor = cell.draw(ntap_nor)?;
        let ptap_nor = cell.draw(ptap_nor)?;
        let ptap_driver_top = cell.draw(ptap_driver_top)?;
        let ptap_driver_bot = cell.draw(ptap_driver_bot)?;
        let ntap_driver_top = cell.draw(ntap_driver_top)?;
        let ntap_driver_bot = cell.draw(ntap_driver_bot)?;
        let ntap_nand = cell.draw(ntap_nand)?;
        let ptap_nand = cell.draw(ptap_nand)?;

        cell.set_top_layer(3);
        cell.set_router(GreedyRouter);
        cell.set_via_maker(T::via_maker());

        // Route `dout` to center track.
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
            io.schematic.dout,
            3,
            cell.layer_stack
                .slice(0..4)
                .shrink_to_lcm_units(dout_rect)
                .unwrap(),
        );
        cell.layout
            .draw(Shape::new(cell.layer_stack.layers[3].id, dout_rect))?;

        io.layout.din.merge(nor_pd_data.layout.io().g);
        io.layout.dout.merge(pu_res.layout.io().p);
        io.layout.pu_ctl.merge(nand_pd_en.layout.io().g);
        io.layout.pd_ctl.merge(nor_pd_en.layout.io().g);
        io.layout.vdd.merge(ntap_driver_top.layout.io().x);
        io.layout.vss.merge(ptap_driver_bot.layout.io().x);

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
            pu_ctl: Array::new(self.0.num_segments, Default::default()),
            pd_ctl: Array::new(self.0.num_segments, Default::default()),
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
        let mut units = Vec::new();
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
                    pd_ctl: if i == 0 || i == self.0.num_segments + 1 {
                        io.schematic.vss
                    } else {
                        io.schematic.pd_ctl[i - 1]
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

        let units = units
            .into_iter()
            .enumerate()
            .map(|(i, unit)| {
                let unit = cell.draw(unit)?;
                io.layout.din.merge(unit.layout.io().din);
                io.layout.dout.merge(unit.layout.io().dout);
                if i > 0 && i < self.0.num_segments + 1 {
                    io.layout.pu_ctl[i - 1].merge(unit.layout.io().pu_ctl);
                    io.layout.pd_ctl[i - 1].merge(unit.layout.io().pd_ctl);
                }
                io.layout.vdd.merge(unit.layout.io().vdd);
                io.layout.vss.merge(unit.layout.io().vss);
                Ok(unit)
            })
            .collect::<Result<Vec<_>>>()?;

        let nf = T::nf(self.0.unit.res_legs, self.0.unit.res_w);
        for unit in units.iter().take(self.0.num_segments + 1) {
            let pu_bbox = unit.layout.data().driver_pu_bbox;
            let pu_loc = cell
                .layer_stack
                .slice(0..2)
                .expand_to_lcm_units(Rect::from_xy(pu_bbox.right(), pu_bbox.center().y));
            let dummy_pu = cell
                .generate_connected(
                    T::tied_mos(TileKind::P, 2, (self.0.unit.driver_pu_w - 1) / nf + 1),
                    io.schematic.vdd,
                )
                .orient(Orientation::ReflectVert)
                .align_rect(pu_loc, AlignMode::CenterVertical, 0)
                .align_rect(pu_loc, AlignMode::CenterHorizontal, 0);
            let pd_bbox = unit.layout.data().driver_pd_bbox;
            let pd_loc = cell
                .layer_stack
                .slice(0..2)
                .expand_to_lcm_units(Rect::from_xy(pd_bbox.right(), pd_bbox.center().y));
            let dummy_pd = cell
                .generate_connected(
                    T::tied_mos(TileKind::N, 2, (self.0.unit.driver_pd_w - 1) / nf + 1),
                    io.schematic.vss,
                )
                .align_rect(pd_loc, AlignMode::CenterVertical, 0)
                .align_rect(pd_loc, AlignMode::CenterHorizontal, 0);

            let _dummy_pu = cell.draw(dummy_pu)?;
            let _dummy_pd = cell.draw(dummy_pd)?;

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
                let filler = cell
                    .layout
                    .generate(T::filler(
                        kind,
                        bbox.height() / cell.layer_stack.layer(1).pitch(),
                    ))
                    .align_bbox(
                        match sign {
                            Sign::Neg => AlignMode::ToTheLeft,
                            Sign::Pos => AlignMode::ToTheRight,
                        },
                        bbox,
                        0,
                    )
                    .align_bbox(AlignMode::Bottom, bbox, 0);
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

        for (bbox, kind) in [(pu_bbox, TileKind::P), (pd_bbox, TileKind::N)] {
            let guard_ring = cell
                .layout
                .generate(T::guard_ring(
                    kind,
                    (self.0.num_segments + 2) as i64,
                    nf,
                    bbox.height() / cell.layer_stack.layer(1).pitch(),
                ))
                .align_bbox(AlignMode::CenterHorizontal, bbox, 0)
                .align_bbox(AlignMode::CenterVertical, bbox, 0);
            cell.layout.draw(guard_ring)?;
        }

        let via_maker = T::via_maker();

        let bump_rect = Rect::from_spans(
            cell.layout.bbox_rect().hspan(),
            Span::from_center_span(units[0].layout.data().dout.bbox_rect().center().y, 1080),
        );
        cell.layout
            .draw(Shape::new(cell.layer_stack.layers[9].id, bump_rect))?;

        let mut via_stack = Vec::new();
        for layer in 4..10 {
            via_stack
                .extend(via_maker.draw_via(cell.ctx().clone(), TrackCoord { layer, x: 0, y: 0 }))
        }
        for unit in units.iter() {
            for shape in &via_stack {
                cell.layout.draw(shape.clone().translate(
                    unit.layout.data().dout.bbox_rect().center() - shape.bbox_rect().center(),
                ))?;
            }
        }

        cell.set_strapping(
            io.schematic.din,
            StrappingParams::new(
                2,
                vec![
                    LayerStrappingParams::OffsetPeriod {
                        offset: 2,
                        period: 3,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 2,
                        period: 3,
                    },
                ],
            ),
        );
        cell.set_strapping(
            io.schematic.vss,
            StrappingParams::new(
                1,
                vec![
                    LayerStrappingParams::OffsetPeriod {
                        offset: 0,
                        period: 3,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 0,
                        period: 3,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 0,
                        period: 3,
                    },
                ],
            ),
        );
        cell.set_strapping(
            io.schematic.vdd,
            StrappingParams::new(
                1,
                vec![
                    LayerStrappingParams::OffsetPeriod {
                        offset: 1,
                        period: 3,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 1,
                        period: 3,
                    },
                    LayerStrappingParams::OffsetPeriod {
                        offset: 1,
                        period: 3,
                    },
                ],
            ),
        );
        cell.set_top_layer(3);
        cell.set_strapper(GreedyStrapper);
        // cell.set_router(GreedyRouter);
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
                io.schematic.din,
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
            io.schematic.dout,
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
        cell.set_router(GreedyRouter);
        cell.set_via_maker(T::via_maker());

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

        let units = units
            .into_iter()
            .enumerate()
            .map(|(i, unit)| {
                let unit = cell.draw(unit)?;
                io.layout.din.merge(unit.layout.io().din);
                io.layout.dout.merge(unit.layout.io().dout);
                io.layout.pu_ctl[i].merge(unit.layout.io().pu_ctl);
                io.layout.pd_ctl[i].merge(unit.layout.io().pd_ctl);
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
        // cell.set_router(GreedyRouter);
        // cell.set_via_maker(T::via_maker());

        T::post_layout_hooks(cell)?;

        Ok(((), ()))
    }
}
