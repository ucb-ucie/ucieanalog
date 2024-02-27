use crate::strongarm::tb::Dut;
use crate::tiles::{MosTileParams, TapIo, TapTileParams, TileKind};
use atoll::route::{GreedyRouter, ViaMaker};
use atoll::{IoBuilder, Orientation, Tile, TileBuilder, TileWrapper};
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

/// The interface to a buffer.
#[derive(Debug, Default, Clone, Io)]
pub struct BufferIo {
    /// The buffer input.
    pub din: Input<Signal>,
    /// The (possibly inverted) buffered output.
    pub dout: Output<Signal>,
    /// The VDD rail.
    pub vdd: InOut<Signal>,
    /// The VSS rail.
    pub vss: InOut<Signal>,
}

/// The parameters of the [`Inverter`] layout generator.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct InverterParams {
    /// The width of the NMOS.
    pub nmos_w: i64,
    /// The width of the PMOS.
    pub pmos_w: i64,
}

pub trait HasInverterImpl<PDK: Pdk + Schema> {
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

#[derive_where::derive_where(Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[derive(Serialize, Deserialize)]
struct Inverter<T>(
    InverterParams,
    #[serde(bound(deserialize = ""))] PhantomData<fn() -> T>,
);

impl<T> Inverter<T> {
    fn new(params: InverterParams) -> Self {
        Self(params, PhantomData)
    }
}

impl<T: Any> Block for Inverter<T> {
    type Io = BufferIo;

    fn id() -> ArcStr {
        substrate::arcstr::literal!("inverter")
    }

    // todo: include parameters in name
    fn name(&self) -> ArcStr {
        substrate::arcstr::literal!("inverter")
    }

    fn io(&self) -> Self::Io {
        Default::default()
    }
}

impl<T: Any> ExportsNestedData for Inverter<T> {
    type NestedData = ();
}

impl<T: Any> ExportsLayoutData for Inverter<T> {
    type LayoutData = ();
}

impl<PDK: Pdk + Schema + Sized, T: HasInverterImpl<PDK> + Any> Tile<PDK> for Inverter<T> {
    fn tile<'a>(
        &self,
        io: IoBuilder<'a, Self>,
        cell: &mut TileBuilder<'a, PDK>,
    ) -> substrate::error::Result<(
        <Self as ExportsNestedData>::NestedData,
        <Self as ExportsLayoutData>::LayoutData,
    )> {
        let nmos_params = MosTileParams::new(TileKind::N, self.0.nmos_w);
        let pmos_params = MosTileParams::new(TileKind::P, self.0.pmos_w);

        let mut nmos = cell
            .generate_connected(
                T::mos(nmos_params),
                MosIoSchematic {
                    d: io.schematic.vss,
                    g: io.schematic.din,
                    s: io.schematic.dout,
                    b: io.schematic.vss,
                },
            )
            .orient(Orientation::R180);
        let mut pmos = cell.generate_connected(
            T::mos(pmos_params),
            MosIoSchematic {
                d: io.schematic.vdd,
                g: io.schematic.din,
                s: io.schematic.dout,
                b: io.schematic.vdd,
            },
        );

        let mut ptap = cell.generate(T::tap(TapTileParams::new(TileKind::P, 1)));
        let ntap = cell.generate(T::tap(TapTileParams::new(TileKind::N, 1)));
        cell.connect(ptap.io().x, io.schematic.vss);
        cell.connect(ntap.io().x, io.schematic.vdd);

        let mut prev = ntap.lcm_bounds();

        for mos in [&mut pmos, &mut nmos] {
            mos.align_rect_mut(prev, AlignMode::Left, 0);
            mos.align_rect_mut(prev, AlignMode::Beneath, 0);
            prev = mos.lcm_bounds();
        }

        ptap.align_rect_mut(prev, AlignMode::Left, 0);
        ptap.align_rect_mut(prev, AlignMode::Beneath, 0);

        let nmos = cell.draw(nmos)?;
        let pmos = cell.draw(pmos)?;
        let ptap = cell.draw(ptap)?;
        let ntap = cell.draw(ntap)?;

        cell.set_top_layer(1);
        cell.set_router(GreedyRouter);
        cell.set_via_maker(T::via_maker());

        io.layout.din.merge(nmos.layout.io().g);
        io.layout.din.merge(pmos.layout.io().g);
        io.layout.dout.merge(nmos.layout.io().s);
        io.layout.dout.merge(pmos.layout.io().s);
        io.layout.vdd.merge(ntap.layout.io().x);
        io.layout.vss.merge(ptap.layout.io().x);

        T::post_layout_hooks(cell)?;

        Ok(((), ()))
    }
}

/// Layout assumes that PDK layer stack has a vertical layer 0.
#[derive_where::derive_where(Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[derive(Serialize, Deserialize)]
pub struct Buffer<T>(
    InverterParams,
    #[serde(bound(deserialize = ""))] PhantomData<fn() -> T>,
);

impl<T> Buffer<T> {
    pub fn new(params: InverterParams) -> Self {
        Self(params, PhantomData)
    }
}

impl<T: Any> Block for Buffer<T> {
    type Io = BufferIo;

    fn id() -> ArcStr {
        substrate::arcstr::literal!("buffer")
    }

    // todo: include parameters in name
    fn name(&self) -> ArcStr {
        substrate::arcstr::literal!("buffer")
    }

    fn io(&self) -> Self::Io {
        Default::default()
    }
}

impl<T: Any> ExportsNestedData for Buffer<T> {
    type NestedData = ();
}

impl<T: Any> ExportsLayoutData for Buffer<T> {
    type LayoutData = ();
}

impl<PDK: Pdk + Schema + Sized, T: HasInverterImpl<PDK> + Any> Tile<PDK> for Buffer<T> {
    fn tile<'a>(
        &self,
        io: IoBuilder<'a, Self>,
        cell: &mut TileBuilder<'a, PDK>,
    ) -> substrate::error::Result<(
        <Self as ExportsNestedData>::NestedData,
        <Self as ExportsLayoutData>::LayoutData,
    )> {
        let int = cell.signal("int", Signal::new());

        let inv1 = cell.generate_connected(
            Inverter::<T>::new(self.0),
            BufferIoSchematic {
                din: io.schematic.din,
                dout: int,
                vdd: io.schematic.vdd,
                vss: io.schematic.vss,
            },
        );
        let inv2 = cell
            .generate_connected(
                Inverter::<T>::new(self.0),
                BufferIoSchematic {
                    din: int,
                    dout: io.schematic.dout,
                    vdd: io.schematic.vdd,
                    vss: io.schematic.vss,
                },
            )
            .align(&inv1, AlignMode::ToTheRight, 0);

        let inv1 = cell.draw(inv1)?;
        let inv2 = cell.draw(inv2)?;

        cell.set_top_layer(1);
        cell.set_router(GreedyRouter);
        cell.set_via_maker(T::via_maker());

        io.layout.vdd.merge(inv1.layout.io().vdd);
        io.layout.vdd.merge(inv2.layout.io().vdd);
        io.layout.vss.merge(inv1.layout.io().vss);
        io.layout.vss.merge(inv2.layout.io().vss);
        io.layout.din.merge(inv1.layout.io().din);
        io.layout.dout.merge(inv2.layout.io().dout);

        T::post_layout_hooks(cell)?;

        Ok(((), ()))
    }
}
