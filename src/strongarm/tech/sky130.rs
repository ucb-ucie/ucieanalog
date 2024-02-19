use crate::strongarm::{
    ClockedDiffComparatorIo, HasStrongArmImpl, MosTileParams, TapIo, TapTileParams, TileKind,
};
use atoll::route::GreedyRouter;
use atoll::{IoBuilder, Tile, TileBuilder};
use serde::{Deserialize, Serialize};
use sky130pdk::atoll::{MosLength, NmosTile, PmosTile, Sky130ViaMaker};
use sky130pdk::layers::Met1;
use sky130pdk::Sky130Pdk;
use substrate::arcstr;
use substrate::arcstr::ArcStr;
use substrate::block::Block;
use substrate::io::layout::Builder;
use substrate::io::schematic::{Bundle, Node};
use substrate::io::{InOut, Input, Io, MosIo, Signal};
use substrate::layout::{ExportsLayoutData, Layout};
use substrate::pdk::Pdk;
use substrate::schematic::{CellBuilder, ExportsNestedData, Schematic};

pub struct Sky130;

impl HasStrongArmImpl<Sky130Pdk> for Sky130 {
    type MosTile = TwoFingerMosTile;
    type TapTile = TapTile;
    type PortLayer = Met1;
    type ViaMaker = Sky130ViaMaker;

    fn mos(params: MosTileParams) -> Self::MosTile {
        TwoFingerMosTile::new(params.w, MosLength::L150, params.kind)
    }
    fn tap(params: TapTileParams) -> Self::TapTile {
        TapTile::new(params)
    }
    fn via_maker() -> Self::ViaMaker {
        Sky130ViaMaker
    }
    fn port_layer(layers: &<Sky130Pdk as Pdk>::Layers) -> Self::PortLayer {
        layers.met1
    }
}

#[derive(Serialize, Deserialize, Block, Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[substrate(io = "MosIo")]
pub struct TwoFingerMosTile {
    w: i64,
    l: MosLength,
    kind: TileKind,
}

impl TwoFingerMosTile {
    pub fn new(w: i64, l: MosLength, kind: TileKind) -> Self {
        Self { w, l, kind }
    }

    pub fn pmos(w: i64, l: MosLength) -> Self {
        Self::new(w, l, TileKind::P)
    }

    pub fn nmos(w: i64, l: MosLength) -> Self {
        Self::new(w, l, TileKind::N)
    }
}

impl ExportsNestedData for TwoFingerMosTile {
    type NestedData = ();
}

impl ExportsLayoutData for TwoFingerMosTile {
    type LayoutData = ();
}

impl Tile<Sky130Pdk> for TwoFingerMosTile {
    fn tile<'a>(
        &self,
        io: IoBuilder<'a, Self>,
        cell: &mut TileBuilder<'a, Sky130Pdk>,
    ) -> substrate::error::Result<(
        <Self as ExportsNestedData>::NestedData,
        <Self as ExportsLayoutData>::LayoutData,
    )> {
        cell.flatten();
        match self.kind {
            TileKind::P => {
                let pmos = cell.generate_primitive(PmosTile::new(self.w, self.l, 2));
                cell.connect(pmos.io().g[0], io.schematic.g);
                cell.connect(pmos.io().b, io.schematic.b);
                cell.connect(pmos.io().sd[0], io.schematic.s);
                cell.connect(pmos.io().sd[1], io.schematic.d);
                cell.connect(pmos.io().sd[2], io.schematic.s);
                let pmos = cell.draw(pmos)?;
                io.layout.g.merge(pmos.layout.io().g[0].clone());
                io.layout.s.merge(pmos.layout.io().sd[0].clone());
                io.layout.d.merge(pmos.layout.io().sd[1].clone());
                io.layout.s.merge(pmos.layout.io().sd[2].clone());
                io.layout.b.merge(pmos.layout.io().b);
            }
            TileKind::N => {
                let nmos = cell.generate_primitive(NmosTile::new(self.w, self.l, 2));
                cell.connect(nmos.io().g[0], io.schematic.g);
                cell.connect(nmos.io().b, io.schematic.b);
                cell.connect(nmos.io().sd[0], io.schematic.s);
                cell.connect(nmos.io().sd[1], io.schematic.d);
                cell.connect(nmos.io().sd[2], io.schematic.s);
                let nmos = cell.draw(nmos)?;
                io.layout.g.merge(nmos.layout.io().g[0].clone());
                io.layout.s.merge(nmos.layout.io().sd[0].clone());
                io.layout.d.merge(nmos.layout.io().sd[1].clone());
                io.layout.s.merge(nmos.layout.io().sd[2].clone());
                io.layout.b.merge(nmos.layout.io().b);
            }
        }

        cell.set_top_layer(1);
        cell.set_router(GreedyRouter);
        cell.set_via_maker(Sky130ViaMaker);

        Ok(((), ()))
    }
}
/// A tile containing a N/P tap for biasing an N-well or P-substrate.
/// These can be used to connect to the body terminals of MOS devices.
#[derive(Debug, Clone, Copy, Hash, Eq, PartialEq, Serialize, Deserialize)]
pub struct TapTile(TapTileParams);

impl TapTile {
    pub fn new(params: TapTileParams) -> Self {
        Self(params)
    }
}

impl Block for TapTile {
    type Io = TapIo;

    fn id() -> ArcStr {
        arcstr::literal!("tap_tile")
    }

    fn name(&self) -> ArcStr {
        arcstr::format!(
            "{}tap_tile",
            match self.0.kind {
                TileKind::N => "n",
                TileKind::P => "p",
            }
        )
    }

    fn io(&self) -> Self::Io {
        Default::default()
    }
}

impl ExportsNestedData for TapTile {
    type NestedData = ();
}

impl ExportsLayoutData for TapTile {
    type LayoutData = ();
}

impl Tile<Sky130Pdk> for TapTile {
    fn tile<'a>(
        &self,
        io: IoBuilder<'a, Self>,
        cell: &mut TileBuilder<'a, Sky130Pdk>,
    ) -> substrate::error::Result<(
        <Self as ExportsNestedData>::NestedData,
        <Self as ExportsLayoutData>::LayoutData,
    )> {
        cell.flatten();
        match self.0.kind {
            TileKind::N => {
                let inst = cell.generate_primitive(sky130pdk::atoll::NtapTile::new(
                    4 * self.0.mos_span - 1,
                    2,
                ));
                cell.connect(io.schematic.x, inst.io().vpb);
                let inst = cell.draw(inst)?;
                io.layout.x.merge(inst.layout.io().vpb);
            }
            TileKind::P => {
                let inst = cell.generate_primitive(sky130pdk::atoll::PtapTile::new(
                    4 * self.0.mos_span - 1,
                    2,
                ));
                cell.connect(io.schematic.x, inst.io().vnb);
                let inst = cell.draw(inst)?;
                io.layout.x.merge(inst.layout.io().vnb);
            }
        }
        cell.set_router(GreedyRouter);
        Ok(((), ()))
    }
}
