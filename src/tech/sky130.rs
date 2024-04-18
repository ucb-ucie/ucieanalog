//! SKY130-specific implementations.

use crate::buffer::InverterImpl;
use crate::strongarm::{StrongArmImpl, StrongArmWithOutputBuffersImpl};
use crate::tiles::{MosTileParams, TapIo, TapTileParams, TileKind};
use atoll::route::GreedyRouter;
use atoll::{IoBuilder, Tile, TileBuilder};
use serde::{Deserialize, Serialize};
use sky130pdk::atoll::{MosLength, NmosTile, PmosTile, Sky130ViaMaker};
use sky130pdk::Sky130Pdk;
use substrate::arcstr;
use substrate::arcstr::ArcStr;
use substrate::block::Block;
use substrate::io::MosIo;
use substrate::layout::ExportsLayoutData;
use substrate::schematic::ExportsNestedData;

/// A SKY130 UCIe implementation.
pub struct Sky130Ucie;

impl StrongArmImpl<Sky130Pdk> for Sky130Ucie {
    type MosTile = TwoFingerMosTile;
    type TapTile = TapTile;
    type ViaMaker = Sky130ViaMaker;

    fn mos(params: MosTileParams) -> Self::MosTile {
        TwoFingerMosTile::new(params.w, MosLength::L150, params.tile_kind)
    }
    fn tap(params: TapTileParams) -> Self::TapTile {
        TapTile::new(params)
    }
    fn via_maker() -> Self::ViaMaker {
        Sky130ViaMaker
    }
}

impl InverterImpl<Sky130Pdk> for Sky130Ucie {
    type MosTile = TwoFingerMosTile;
    type TapTile = TapTile;
    type ViaMaker = Sky130ViaMaker;

    fn mos(params: MosTileParams) -> Self::MosTile {
        TwoFingerMosTile::new(params.w, MosLength::L150, params.tile_kind)
    }
    fn tap(params: TapTileParams) -> Self::TapTile {
        TapTile::new(params)
    }
    fn via_maker() -> Self::ViaMaker {
        Sky130ViaMaker
    }
}

impl StrongArmWithOutputBuffersImpl<Sky130Pdk> for Sky130Ucie {
    const BUFFER_SPACING: i64 = 3;
}

/// A two-finger MOS tile.
#[derive(Serialize, Deserialize, Block, Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[substrate(io = "MosIo")]
pub struct TwoFingerMosTile {
    w: i64,
    l: MosLength,
    kind: TileKind,
}

impl TwoFingerMosTile {
    /// Creates a new [`TwoFingerMosTile`].
    pub fn new(w: i64, l: MosLength, kind: TileKind) -> Self {
        Self { w, l, kind }
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
        cell.set_router(GreedyRouter::new());
        cell.set_via_maker(Sky130ViaMaker);

        Ok(((), ()))
    }
}

/// A tile containing a N/P tap for biasing an N-well or P-substrate.
/// These can be used to connect to the body terminals of MOS devices.
#[derive(Debug, Clone, Copy, Hash, Eq, PartialEq, Serialize, Deserialize)]
pub struct TapTile(TapTileParams);

impl TapTile {
    /// Creates a new [`TapTile`].
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
        cell.set_router(GreedyRouter::new());
        Ok(((), ()))
    }
}

#[cfg(test)]
mod tests {
    use crate::buffer::{Buffer, InverterParams};
    use crate::sky130_ctx;
    use crate::strongarm::tb::{ComparatorDecision, StrongArmTranTb};
    use crate::strongarm::{InputKind, StrongArm, StrongArmParams, StrongArmWithOutputBuffers};
    use crate::tech::sky130::Sky130Ucie;
    use crate::tiles::MosKind;
    use atoll::TileWrapper;
    use rust_decimal::Decimal;
    use rust_decimal_macros::dec;
    use sky130pdk::corner::Sky130Corner;
    use sky130pdk::Sky130CommercialSchema;
    use spice::netlist::NetlistOptions;
    use spice::Spice;
    use std::path::PathBuf;
    use substrate::pdk::corner::Pvt;
    use substrate::schematic::netlist::ConvertibleNetlister;

    #[test]
    fn sky130_strongarm_sim() {
        let work_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/build/strongarm_sim");
        let input_kind = InputKind::P;
        let dut = TileWrapper::new(StrongArm::<Sky130Ucie>::new(StrongArmParams {
            nmos_kind: MosKind::Nom,
            pmos_kind: MosKind::Nom,
            half_tail_w: 1_000,
            input_pair_w: 1_000,
            inv_input_w: 1_000,
            inv_precharge_w: 1_000,
            precharge_w: 1_000,
            input_kind,
        }));
        let pvt = Pvt {
            corner: Sky130Corner::Tt,
            voltage: dec!(1.8),
            temp: dec!(25.0),
        };
        let ctx = sky130_ctx();

        for i in 0..=10 {
            for j in [
                dec!(-1.8),
                dec!(-0.5),
                dec!(-0.1),
                dec!(-0.05),
                dec!(0.05),
                dec!(0.1),
                dec!(0.5),
                dec!(1.8),
            ] {
                let vinn = dec!(0.18) * Decimal::from(i);
                let vinp = vinn + j;

                match input_kind {
                    InputKind::P => {
                        if (vinp + vinn) / dec!(2) > dec!(1.5) {
                            continue;
                        }
                    }
                    InputKind::N => {
                        if (vinp + vinn) / dec!(2) < dec!(0.3) {
                            continue;
                        }
                    }
                }

                let tb = StrongArmTranTb::new(dut, vinp, vinn, input_kind.is_p(), pvt);
                let decision = ctx
                    .simulate(tb, work_dir)
                    .expect("failed to run simulation")
                    .expect("comparator output did not rail");
                assert_eq!(
                    decision,
                    if j > dec!(0) {
                        ComparatorDecision::Pos
                    } else {
                        ComparatorDecision::Neg
                    },
                    "comparator produced incorrect decision"
                );
            }
        }
    }

    #[test]
    fn sky130_strongarm_lvs() {
        let work_dir = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/build/strongarm_lvs"));
        let gds_path = work_dir.join("layout.gds");
        let netlist_path = work_dir.join("netlist.sp");
        let ctx = sky130_ctx();

        let block = TileWrapper::new(StrongArm::<Sky130Ucie>::new(StrongArmParams {
            nmos_kind: MosKind::Nom,
            pmos_kind: MosKind::Nom,
            half_tail_w: 1_000,
            input_pair_w: 1_000,
            inv_input_w: 1_000,
            inv_precharge_w: 1_000,
            precharge_w: 1_000,
            input_kind: InputKind::P,
        }));

        let scir = ctx
            .export_scir(block)
            .unwrap()
            .scir
            .convert_schema::<Sky130CommercialSchema>()
            .unwrap()
            .convert_schema::<Spice>()
            .unwrap()
            .build()
            .unwrap();
        Spice
            .write_scir_netlist_to_file(&scir, netlist_path, NetlistOptions::default())
            .expect("failed to write netlist");

        ctx.write_layout(block, gds_path)
            .expect("failed to write layout");
    }

    #[test]
    fn sky130_buffer_lvs() {
        let work_dir = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/build/buffer_lvs"));
        let gds_path = work_dir.join("layout.gds");
        let netlist_path = work_dir.join("netlist.sp");
        let ctx = sky130_ctx();

        let block = TileWrapper::new(Buffer::<Sky130Ucie>::new(InverterParams {
            nmos_kind: MosKind::Nom,
            pmos_kind: MosKind::Nom,
            nmos_w: 1_000,
            pmos_w: 1_000,
        }));

        let scir = ctx
            .export_scir(block)
            .unwrap()
            .scir
            .convert_schema::<Sky130CommercialSchema>()
            .unwrap()
            .convert_schema::<Spice>()
            .unwrap()
            .build()
            .unwrap();
        Spice
            .write_scir_netlist_to_file(&scir, netlist_path, NetlistOptions::default())
            .expect("failed to write netlist");

        ctx.write_layout(block, gds_path)
            .expect("failed to write layout");
    }

    #[test]
    fn sky130_strongarm_with_output_buffers_lvs() {
        let work_dir = PathBuf::from(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/build/strongarm_with_output_buffers_lvs"
        ));
        let gds_path = work_dir.join("layout.gds");
        let netlist_path = work_dir.join("netlist.sp");
        let ctx = sky130_ctx();

        let block = TileWrapper::new(StrongArmWithOutputBuffers::<Sky130Ucie>::new(
            StrongArmParams {
                nmos_kind: MosKind::Nom,
                pmos_kind: MosKind::Nom,
                half_tail_w: 1_000,
                input_pair_w: 1_000,
                inv_input_w: 1_000,
                inv_precharge_w: 1_000,
                precharge_w: 1_000,
                input_kind: InputKind::P,
            },
            InverterParams {
                nmos_kind: MosKind::Nom,
                pmos_kind: MosKind::Nom,
                nmos_w: 1_000,
                pmos_w: 1_000,
            },
        ));

        let scir = ctx
            .export_scir(block)
            .unwrap()
            .scir
            .convert_schema::<Sky130CommercialSchema>()
            .unwrap()
            .convert_schema::<Spice>()
            .unwrap()
            .build()
            .unwrap();
        Spice
            .write_scir_netlist_to_file(&scir, netlist_path, NetlistOptions::default())
            .expect("failed to write netlist");

        ctx.write_layout(block, gds_path)
            .expect("failed to write layout");
    }
}
