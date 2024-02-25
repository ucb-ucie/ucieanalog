use atoll::route::{GreedyRouter, ViaMaker};
use atoll::{IoBuilder, Orientation, Tile, TileBuilder};
use serde::{Deserialize, Serialize};
use std::any::Any;
use std::marker::PhantomData;
use substrate::arcstr::ArcStr;
use substrate::block::Block;
use substrate::error::Result;
use substrate::geometry::align::AlignMode;
use substrate::io::{DiffPair, InOut, Input, Io, MosIo, MosIoSchematic, Output, Signal};
use substrate::layout::ExportsLayoutData;
use substrate::pdk::Pdk;
use substrate::schematic::schema::Schema;
use substrate::schematic::ExportsNestedData;

pub mod tb;
pub mod tech;

#[derive(Debug, Default, Clone, Io)]
pub struct ClockedDiffComparatorIo {
    pub input: Input<DiffPair>,
    pub output: Output<DiffPair>,
    pub clock: Input<Signal>,
    pub vdd: InOut<Signal>,
    pub vss: InOut<Signal>,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub enum InputKind {
    N,
    P,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct StrongArmParams {
    pub half_tail_w: i64,
    pub input_pair_w: i64,
    pub inv_input_w: i64,
    pub inv_precharge_w: i64,
    pub precharge_w: i64,
    pub input_kind: InputKind,
}

/// The IO of a tap.
#[derive(Default, Debug, Clone, Copy, Io)]
pub struct TapIo {
    /// The tap contact.
    pub x: InOut<Signal>,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub enum TileKind {
    N,
    P,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct MosTileParams {
    pub kind: TileKind,
    pub w: i64,
}

impl MosTileParams {
    pub fn new(kind: TileKind, w: i64) -> Self {
        Self { kind, w }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct TapTileParams {
    pub kind: TileKind,
    /// Number of MOS devices this tap must span.
    pub mos_span: i64,
}

impl TapTileParams {
    pub fn new(kind: TileKind, mos_span: i64) -> Self {
        Self { kind, mos_span }
    }
}

pub trait HasStrongArmImpl<PDK: Pdk + Schema> {
    type MosTile: Tile<PDK> + Block<Io = MosIo> + Clone;
    type TapTile: Tile<PDK> + Block<Io = TapIo> + Clone;
    type ViaMaker: ViaMaker<PDK>;

    fn mos(params: MosTileParams) -> Self::MosTile;
    fn tap(params: TapTileParams) -> Self::TapTile;
    fn via_maker() -> Self::ViaMaker;
    fn post_layout_hooks(_cell: &mut TileBuilder<'_, PDK>) -> Result<()> {
        Ok(())
    }
}

#[derive(Debug, Default, Clone, Io)]
struct StrongArmHalfIo {
    /// Ports that are exposed at the top level of a StrongARM.
    top_io: InOut<ClockedDiffComparatorIo>,
    /// Drains of input pair.
    input_d: InOut<DiffPair>,
    /// Drain of tail.
    tail_d: InOut<Signal>,
}

#[derive_where::derive_where(Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[derive(Serialize, Deserialize)]
struct StrongArmHalf<T>(
    StrongArmParams,
    #[serde(bound(deserialize = ""))] PhantomData<fn() -> T>,
);

impl<T> StrongArmHalf<T> {
    fn new(params: StrongArmParams) -> Self {
        Self(params, PhantomData)
    }
}

impl<T: Any> Block for StrongArmHalf<T> {
    type Io = StrongArmHalfIo;

    fn id() -> ArcStr {
        substrate::arcstr::literal!("strong_arm_half")
    }

    // todo: include parameters in name
    fn name(&self) -> ArcStr {
        substrate::arcstr::literal!("strong_arm_half")
    }

    fn io(&self) -> Self::Io {
        Default::default()
    }
}

impl<T: Any> ExportsNestedData for StrongArmHalf<T> {
    type NestedData = ();
}

impl<T: Any> ExportsLayoutData for StrongArmHalf<T> {
    type LayoutData = ();
}

impl<PDK: Pdk + Schema + Sized, T: HasStrongArmImpl<PDK> + Any> Tile<PDK> for StrongArmHalf<T> {
    fn tile<'a>(
        &self,
        io: IoBuilder<'a, Self>,
        cell: &mut TileBuilder<'a, PDK>,
    ) -> substrate::error::Result<(
        <Self as ExportsNestedData>::NestedData,
        <Self as ExportsLayoutData>::LayoutData,
    )> {
        let (input_kind, precharge_kind, input_rail, precharge_rail) = match self.0.input_kind {
            InputKind::N => (
                TileKind::N,
                TileKind::P,
                io.schematic.top_io.vss,
                io.schematic.top_io.vdd,
            ),
            InputKind::P => (
                TileKind::P,
                TileKind::N,
                io.schematic.top_io.vss,
                io.schematic.top_io.vdd,
            ),
        };
        let half_tail_params = MosTileParams::new(input_kind, self.0.half_tail_w);
        let input_pair_params = MosTileParams::new(input_kind, self.0.input_pair_w);
        let inv_input_params = MosTileParams::new(input_kind, self.0.inv_input_w);
        let inv_precharge_params = MosTileParams::new(precharge_kind, self.0.inv_precharge_w);
        let precharge_params = MosTileParams::new(precharge_kind, self.0.precharge_w);

        let tail = io.schematic.tail_d;
        let intn = io.schematic.input_d.n;
        let intp = cell.signal("intp", Signal);

        let mut tail_dummy = cell.generate_connected(
            T::mos(half_tail_params),
            MosIoSchematic {
                d: input_rail,
                g: input_rail,
                s: input_rail,
                b: input_rail,
            },
        );
        let mut tail_pair = (0..2)
            .map(|_| {
                cell.generate_connected(
                    T::mos(half_tail_params),
                    MosIoSchematic {
                        d: tail,
                        g: io.schematic.top_io.clock,
                        s: input_rail,
                        b: input_rail,
                    },
                )
            })
            .collect::<Vec<_>>();

        let mut ptap = cell.generate(T::tap(TapTileParams::new(TileKind::P, 3)));
        let ntap = cell.generate(T::tap(TapTileParams::new(TileKind::N, 3)));
        cell.connect(ptap.io().x, io.schematic.top_io.vss);
        cell.connect(ntap.io().x, io.schematic.top_io.vdd);

        let mut input_pair = (0..2)
            .map(|i| {
                cell.generate_connected(
                    T::mos(input_pair_params),
                    MosIoSchematic {
                        d: if i == 0 { intn } else { intp },
                        g: if i == 0 {
                            io.schematic.top_io.input.p
                        } else {
                            io.schematic.top_io.input.n
                        },
                        s: tail,
                        b: input_rail,
                    },
                )
            })
            .collect::<Vec<_>>();
        let mut input_dummy = cell.generate_connected(
            T::mos(input_pair_params),
            MosIoSchematic {
                d: input_rail,
                g: input_rail,
                s: input_rail,
                b: input_rail,
            },
        );
        let mut inv_input_pair = (0..2)
            .map(|i| {
                cell.generate_connected(
                    T::mos(inv_input_params),
                    if i == 0 {
                        MosIoSchematic {
                            d: io.schematic.top_io.output.n,
                            g: io.schematic.top_io.output.p,
                            s: intn,
                            b: input_rail,
                        }
                    } else {
                        MosIoSchematic {
                            d: io.schematic.top_io.output.p,
                            g: io.schematic.top_io.output.n,
                            s: intp,
                            b: input_rail,
                        }
                    },
                )
            })
            .collect::<Vec<_>>();
        let mut inv_input_dummy = cell.generate_connected(
            T::mos(inv_input_params),
            MosIoSchematic {
                d: input_rail,
                g: input_rail,
                s: input_rail,
                b: input_rail,
            },
        );
        let mut inv_precharge_pair = (0..2)
            .map(|i| {
                cell.generate_connected(
                    T::mos(inv_precharge_params),
                    MosIoSchematic {
                        d: if i == 0 {
                            io.schematic.top_io.output.n
                        } else {
                            io.schematic.top_io.output.p
                        },
                        g: if i == 0 {
                            io.schematic.top_io.output.p
                        } else {
                            io.schematic.top_io.output.n
                        },
                        s: precharge_rail,
                        b: precharge_rail,
                    },
                )
            })
            .collect::<Vec<_>>();
        let mut inv_precharge_dummy = cell.generate_connected(
            T::mos(inv_precharge_params),
            MosIoSchematic {
                d: precharge_rail,
                g: precharge_rail,
                s: precharge_rail,
                b: precharge_rail,
            },
        );
        let mut precharge_pair_a = (0..2)
            .map(|i| {
                cell.generate_connected(
                    T::mos(precharge_params),
                    MosIoSchematic {
                        d: if i == 0 {
                            io.schematic.top_io.output.n
                        } else {
                            io.schematic.top_io.output.p
                        },
                        g: io.schematic.top_io.clock,
                        s: precharge_rail,
                        b: precharge_rail,
                    },
                )
            })
            .collect::<Vec<_>>();
        let mut precharge_pair_a_dummy = cell.generate_connected(
            T::mos(precharge_params),
            MosIoSchematic {
                d: precharge_rail,
                g: precharge_rail,
                s: precharge_rail,
                b: precharge_rail,
            },
        );
        let mut precharge_pair_b = (0..2)
            .map(|i| {
                cell.generate_connected(
                    T::mos(precharge_params),
                    MosIoSchematic {
                        d: if i == 0 { intn } else { intp },
                        g: io.schematic.top_io.clock,
                        s: precharge_rail,
                        b: precharge_rail,
                    },
                )
            })
            .collect::<Vec<_>>();
        let mut precharge_pair_b_dummy = cell.generate_connected(
            T::mos(precharge_params),
            MosIoSchematic {
                d: precharge_rail,
                g: precharge_rail,
                s: precharge_rail,
                b: precharge_rail,
            },
        );

        let mut prev = ntap.lcm_bounds();

        let mut rows = [
            (&mut precharge_pair_a_dummy, &mut precharge_pair_a),
            (&mut precharge_pair_b_dummy, &mut precharge_pair_b),
            (&mut inv_precharge_dummy, &mut inv_precharge_pair),
            (&mut inv_input_dummy, &mut inv_input_pair),
            (&mut input_dummy, &mut input_pair),
            (&mut tail_dummy, &mut tail_pair),
        ];

        if self.0.input_kind == InputKind::P {
            rows.reverse();
        }

        for (dummy, mos_pair) in rows {
            dummy.align_rect_mut(prev, AlignMode::Left, 0);
            dummy.align_rect_mut(prev, AlignMode::Beneath, 0);
            prev = dummy.lcm_bounds();
            mos_pair[0].align_rect_mut(prev, AlignMode::Bottom, 0);
            mos_pair[0].align_rect_mut(prev, AlignMode::ToTheRight, 0);
            let left_rect = mos_pair[0].lcm_bounds();
            mos_pair[1].align_rect_mut(left_rect, AlignMode::Bottom, 0);
            mos_pair[1].align_rect_mut(left_rect, AlignMode::ToTheRight, 0);
        }

        ptap.align_rect_mut(prev, AlignMode::Left, 0);
        ptap.align_rect_mut(prev, AlignMode::Beneath, 0);

        let ptap = cell.draw(ptap)?;
        let ntap = cell.draw(ntap)?;
        let tail_pair = tail_pair
            .into_iter()
            .map(|inst| cell.draw(inst))
            .collect::<Result<Vec<_>>>()?;
        let _tail_dummy = cell.draw(tail_dummy)?;
        let input_pair = input_pair
            .into_iter()
            .map(|inst| cell.draw(inst))
            .collect::<Result<Vec<_>>>()?;
        let _input_dummy = cell.draw(input_dummy)?;
        let inv_nmos_pair = inv_input_pair
            .into_iter()
            .map(|inst| cell.draw(inst))
            .collect::<Result<Vec<_>>>()?;
        let _inv_nmos_dummy = cell.draw(inv_input_dummy)?;
        let _inv_pmos_pair = inv_precharge_pair
            .into_iter()
            .map(|inst| cell.draw(inst))
            .collect::<Result<Vec<_>>>()?;
        let _inv_pmos_dummy = cell.draw(inv_precharge_dummy)?;
        let _precharge_pair_a = precharge_pair_a
            .into_iter()
            .map(|inst| cell.draw(inst))
            .collect::<Result<Vec<_>>>()?;
        let _precharge_pair_a_dummy = cell.draw(precharge_pair_a_dummy)?;
        let _precharge_pair_b = precharge_pair_b
            .into_iter()
            .map(|inst| cell.draw(inst))
            .collect::<Result<Vec<_>>>()?;
        let _precharge_pair_b_dummy = cell.draw(precharge_pair_b_dummy)?;

        cell.set_top_layer(2);
        cell.set_router(GreedyRouter);
        cell.set_via_maker(T::via_maker());

        io.layout.top_io.vdd.set_primary(ntap.layout.io().x.primary);
        io.layout.top_io.vss.set_primary(ptap.layout.io().x.primary);
        io.layout.input_d.n.merge(input_pair[0].layout.io().d);
        io.layout.input_d.p.merge(input_pair[1].layout.io().d);
        io.layout.tail_d.merge(tail_pair[0].layout.io().d);
        io.layout.top_io.clock.merge(tail_pair[0].layout.io().g);
        io.layout.top_io.input.p.merge(input_pair[0].layout.io().g);
        io.layout.top_io.input.n.merge(input_pair[1].layout.io().g);
        io.layout
            .top_io
            .output
            .p
            .merge(inv_nmos_pair[1].layout.io().d);
        io.layout
            .top_io
            .output
            .n
            .merge(inv_nmos_pair[0].layout.io().d);

        Ok(((), ()))
    }
}

/// Layout assumes that PDK layer stack has a vertical layer 0.
#[derive_where::derive_where(Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[derive(Serialize, Deserialize)]
pub struct StrongArm<T>(
    StrongArmParams,
    #[serde(bound(deserialize = ""))] PhantomData<fn() -> T>,
);

impl<T> StrongArm<T> {
    pub fn new(params: StrongArmParams) -> Self {
        Self(params, PhantomData)
    }
}

impl<T: Any> Block for StrongArm<T> {
    type Io = ClockedDiffComparatorIo;

    fn id() -> ArcStr {
        substrate::arcstr::literal!("strong_arm")
    }

    // todo: include parameters in name
    fn name(&self) -> ArcStr {
        substrate::arcstr::literal!("strong_arm")
    }

    fn io(&self) -> Self::Io {
        Default::default()
    }
}

impl<T: Any> ExportsNestedData for StrongArm<T> {
    type NestedData = ();
}

impl<T: Any> ExportsLayoutData for StrongArm<T> {
    type LayoutData = ();
}

impl<PDK: Pdk + Schema + Sized, T: HasStrongArmImpl<PDK> + Any> Tile<PDK> for StrongArm<T> {
    fn tile<'a>(
        &self,
        io: IoBuilder<'a, Self>,
        cell: &mut TileBuilder<'a, PDK>,
    ) -> substrate::error::Result<(
        <Self as ExportsNestedData>::NestedData,
        <Self as ExportsLayoutData>::LayoutData,
    )> {
        let tail_d = cell.signal("tail_d", Signal::new());
        let input_d = cell.signal("input_d", DiffPair::default());

        let conn = StrongArmHalfIoSchematic {
            top_io: io.schematic.clone(),
            input_d,
            tail_d,
        };
        let left_half = cell.generate_connected(StrongArmHalf::<T>::new(self.0), conn.clone());

        let right_half = cell
            .generate_connected(StrongArmHalf::<T>::new(self.0), conn)
            .orient(Orientation::ReflectHoriz)
            .align(&left_half, AlignMode::ToTheRight, 0);

        let left_half = cell.draw(left_half)?;
        let right_half = cell.draw(right_half)?;

        cell.set_top_layer(2);
        cell.set_router(GreedyRouter);
        cell.set_via_maker(T::via_maker());

        io.layout.vdd.merge(left_half.layout.io().top_io.vdd);
        io.layout.vdd.merge(right_half.layout.io().top_io.vdd);
        io.layout.vss.merge(left_half.layout.io().top_io.vss);
        io.layout.vss.merge(right_half.layout.io().top_io.vss);
        io.layout.clock.merge(left_half.layout.io().top_io.clock);
        io.layout.clock.merge(right_half.layout.io().top_io.clock);
        io.layout
            .input
            .p
            .merge(left_half.layout.io().top_io.input.p);
        io.layout
            .input
            .p
            .merge(right_half.layout.io().top_io.input.p);
        io.layout
            .input
            .n
            .merge(left_half.layout.io().top_io.input.n);
        io.layout
            .input
            .n
            .merge(right_half.layout.io().top_io.input.n);
        io.layout
            .output
            .p
            .merge(left_half.layout.io().top_io.output.p);
        io.layout
            .output
            .p
            .merge(right_half.layout.io().top_io.output.p);
        io.layout
            .output
            .n
            .merge(left_half.layout.io().top_io.output.n);
        io.layout
            .output
            .n
            .merge(right_half.layout.io().top_io.output.n);

        T::post_layout_hooks(cell)?;

        Ok(((), ()))
    }
}
