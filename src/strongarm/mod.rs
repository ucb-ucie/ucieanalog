use atoll::route::GreedyRouter;
use atoll::{IoBuilder, Tile, TileBuilder};
use serde::{Deserialize, Serialize};
use sky130pdk::atoll::{MosLength, NmosTile, NtapTile, PmosTile, PtapTile, Sky130ViaMaker};
use sky130pdk::Sky130Pdk;
use substrate::block::Block;
use substrate::error::Result;
use substrate::geometry::align::AlignMode;
use substrate::geometry::bbox::Bbox;
use substrate::geometry::dir::Dir;
use substrate::geometry::rect::Rect;
use substrate::geometry::span::Span;
use substrate::io::layout::{Builder, IoShape};
use substrate::io::schematic::{Bundle, Node};
use substrate::io::{InOut, Input, Io, Signal, DiffPair, Output};
use substrate::layout::element::Shape;
use substrate::layout::{ExportsLayoutData, Layout};
use substrate::schematic::{CellBuilder, ExportsNestedData, Schematic};

pub mod tb;

#[derive(Debug, Default, Clone, Io)]
pub struct ClockedDiffComparatorIo {
    pub input: Input<DiffPair>,
    pub output: Output<DiffPair>,
    pub clock: Input<Signal>,
    pub vdd: InOut<Signal>,
    pub vss: InOut<Signal>,
}

#[derive(Debug, Default, Clone, Io)]
pub struct TwoFingerMosTileIo {
    pub sd0: InOut<Signal>,
    pub sd1: InOut<Signal>,
    pub sd2: InOut<Signal>,
    pub g: Input<Signal>,
    pub b: InOut<Signal>,
}

impl TwoFingerMosTileIo {
    pub fn dgsb(d: Node, g: Node, s: Node, b: Node) -> Bundle<Self> {
        Bundle::<Self> {
            sd0: s,
            sd1: d,
            sd2: s,
            g,
            b,
        }
    }
}

#[derive(Serialize, Deserialize, Block, Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[substrate(io = "TwoFingerMosTileIo")]
struct TwoFingerMosTile {
    w: i64,
    l: MosLength,
    kind: MosTileKind,
}

impl TwoFingerMosTile {
    pub fn new(w: i64, l: MosLength, kind: MosTileKind) -> Self {
        Self { w, l, kind }
    }

    pub fn pmos(w: i64, l: MosLength) -> Self {
        Self::new(w, l, MosTileKind::Pmos)
    }

    pub fn nmos(w: i64, l: MosLength) -> Self {
        Self::new(w, l, MosTileKind::Nmos)
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub enum MosTileKind {
    Pmos,
    Nmos,
}

impl ExportsNestedData for TwoFingerMosTile {
    type NestedData = ();
}

impl Schematic<Sky130Pdk> for TwoFingerMosTile {
    fn schematic(
        &self,
        io: &Bundle<<Self as Block>::Io>,
        cell: &mut CellBuilder<Sky130Pdk>,
    ) -> substrate::error::Result<Self::NestedData> {
        cell.flatten();
        match self.kind {
            MosTileKind::Pmos => {
                let pmos = cell.instantiate(PmosTile::new(self.w, self.l, 2));
                cell.connect(pmos.io().g, io.g);
                cell.connect(pmos.io().b, io.b);
                cell.connect(pmos.io().sd[0], io.sd0);
                cell.connect(pmos.io().sd[1], io.sd1);
                cell.connect(pmos.io().sd[2], io.sd2);
            }
            MosTileKind::Nmos => {
                let nmos = cell.instantiate(NmosTile::new(self.w, self.l, 2));
                cell.connect(nmos.io().g, io.g);
                cell.connect(nmos.io().b, io.b);
                cell.connect(nmos.io().sd[0], io.sd0);
                cell.connect(nmos.io().sd[1], io.sd1);
                cell.connect(nmos.io().sd[2], io.sd2);
            }
        }
        Ok(())
    }
}

impl ExportsLayoutData for TwoFingerMosTile {
    type LayoutData = ();
}

impl Layout<Sky130Pdk> for TwoFingerMosTile {
    fn layout(
        &self,
        io: &mut Builder<<Self as Block>::Io>,
        cell: &mut substrate::layout::CellBuilder<Sky130Pdk>,
    ) -> substrate::error::Result<Self::LayoutData> {
        match self.kind {
            MosTileKind::Pmos => {
                let pmos = cell.generate(PmosTile::new(self.w, self.l, 2));
                io.g.merge(pmos.io().g);
                io.sd0.merge(pmos.io().sd[0].clone());
                io.sd1.merge(pmos.io().sd[1].clone());
                io.sd2.merge(pmos.io().sd[2].clone());
                io.b.merge(pmos.io().b);
                cell.draw(pmos)?;
            }
            MosTileKind::Nmos => {
                let nmos = cell.generate(NmosTile::new(self.w, self.l, 2));
                io.g.merge(nmos.io().g);
                io.sd0.merge(nmos.io().sd[0].clone());
                io.sd1.merge(nmos.io().sd[1].clone());
                io.sd2.merge(nmos.io().sd[2].clone());
                io.b.merge(nmos.io().b);
                cell.draw(nmos)?;
            }
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Block, Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[substrate(io = "ClockedDiffComparatorIo")]
pub struct StrongArmInstance {
    pub half_tail_w: i64,
    pub input_pair_w: i64,
    pub inv_nmos_w: i64,
    pub inv_pmos_w: i64,
    pub precharge_w: i64,
}

impl ExportsNestedData for StrongArmInstance {
    type NestedData = ();
}

impl ExportsLayoutData for StrongArmInstance {
    type LayoutData = ();
}

impl Tile<Sky130Pdk> for StrongArmInstance {
    fn tile<'a>(
        &self,
        io: IoBuilder<'a, Self>,
        cell: &mut TileBuilder<'a, Sky130Pdk>,
    ) -> substrate::error::Result<(
        <Self as ExportsNestedData>::NestedData,
        <Self as ExportsLayoutData>::LayoutData,
    )> {
        let l = MosLength::L150;

        let half_tail = TwoFingerMosTile::nmos(self.half_tail_w, l);
        let input_pair = TwoFingerMosTile::nmos(self.input_pair_w, l);
        let inv_nmos = TwoFingerMosTile::nmos(self.inv_nmos_w, l);
        let inv_pmos = TwoFingerMosTile::pmos(self.inv_pmos_w, l);
        let precharge = TwoFingerMosTile::pmos(self.precharge_w, l);

        let tail = cell.signal("tail", Signal);
        let intn = cell.signal("intn", Signal);
        let intp = cell.signal("intp", Signal);

        let mut tail_pair = (0..2)
            .map(|_| {
                cell.generate_primitive_connected(
                    half_tail,
                    TwoFingerMosTileIo::dgsb(
                        tail,
                        io.schematic.clock,
                        io.schematic.vss,
                        io.schematic.vss,
                    ),
                )
            })
            .collect::<Vec<_>>();

        // todo: get rid of minus 1
        let mut ptap =
            cell.generate_primitive(PtapTile::new(2 * tail_pair[0].lcm_bounds().width() - 1, 2));
        let ntap =
            cell.generate_primitive(NtapTile::new(2 * tail_pair[1].lcm_bounds().width() - 1, 2));
        cell.connect(ptap.io().vnb, io.schematic.vss);
        cell.connect(ntap.io().vpb, io.schematic.vdd);

        let mut input_pair = (0..2)
            .map(|i| {
                cell.generate_primitive_connected(
                    input_pair,
                    TwoFingerMosTileIo::dgsb(
                        if i == 0 { intn } else { intp },
                        if i == 0 {
                            io.schematic.input.p
                        } else {
                            io.schematic.input.n
                        },
                        tail,
                        io.schematic.vss,
                    ),
                )
            })
            .collect::<Vec<_>>();
        let mut inv_nmos_pair = (0..2)
            .map(|i| {
                cell.generate_primitive_connected(
                    inv_nmos,
                    if i == 0 {
                        TwoFingerMosTileIo::dgsb(
                            io.schematic.output.n,
                            io.schematic.output.p,
                            intn,
                            io.schematic.vss,
                        )
                    } else {
                        TwoFingerMosTileIo::dgsb(
                            io.schematic.output.p,
                            io.schematic.output.n,
                            intp,
                            io.schematic.vss,
                        )
                    },
                )
            })
            .collect::<Vec<_>>();
        let mut inv_pmos_pair = (0..2)
            .map(|i| {
                cell.generate_primitive_connected(
                    inv_pmos,
                    TwoFingerMosTileIo::dgsb(
                        if i == 0 {
                            io.schematic.output.n
                        } else {
                            io.schematic.output.p
                        },
                        if i == 0 {
                            io.schematic.output.p
                        } else {
                            io.schematic.output.n
                        },
                        io.schematic.vdd,
                        io.schematic.vdd,
                    ),
                )
            })
            .collect::<Vec<_>>();
        let mut precharge_pair_a = (0..2)
            .map(|i| {
                cell.generate_primitive_connected(
                    precharge,
                    TwoFingerMosTileIo::dgsb(
                        if i == 0 {
                            io.schematic.output.n
                        } else {
                            io.schematic.output.p
                        },
                        io.schematic.clock,
                        io.schematic.vdd,
                        io.schematic.vdd,
                    ),
                )
            })
            .collect::<Vec<_>>();
        let mut precharge_pair_b = (0..2)
            .map(|i| {
                cell.generate_primitive_connected(
                    precharge,
                    TwoFingerMosTileIo::dgsb(
                        if i == 0 { intn } else { intp },
                        io.schematic.clock,
                        io.schematic.vdd,
                        io.schematic.vdd,
                    ),
                )
            })
            .collect::<Vec<_>>();

        let mut prev = ntap.lcm_bounds();

        for row in [
            &mut precharge_pair_a,
            &mut precharge_pair_b,
            &mut inv_pmos_pair,
            &mut inv_nmos_pair,
            &mut input_pair,
            &mut tail_pair,
        ] {
            row[0].align_rect_mut(prev, AlignMode::Left, 0);
            row[0].align_rect_mut(prev, AlignMode::Beneath, 0);
            prev = row[0].lcm_bounds();
            row[1].align_rect_mut(prev, AlignMode::Bottom, 0);
            row[1].align_rect_mut(prev, AlignMode::ToTheRight, 0);
        }

        ptap.align_rect_mut(prev, AlignMode::Left, 0);
        ptap.align_rect_mut(prev, AlignMode::Beneath, 0);

        let strongarm_lcm_hspan = ptap.lcm_bounds().hspan();

        let ptap = cell.draw(ptap)?;
        let ntap = cell.draw(ntap)?;
        let tail_pair = tail_pair
            .into_iter()
            .map(|inst| cell.draw(inst))
            .collect::<Result<Vec<_>>>()?;
        let input_pair = input_pair
            .into_iter()
            .map(|inst| cell.draw(inst))
            .collect::<Result<Vec<_>>>()?;
        let _inv_nmos_pair = inv_nmos_pair
            .into_iter()
            .map(|inst| cell.draw(inst))
            .collect::<Result<Vec<_>>>()?;
        let inv_pmos_pair = inv_pmos_pair
            .into_iter()
            .map(|inst| cell.draw(inst))
            .collect::<Result<Vec<_>>>()?;
        let _precharge_pair_a = precharge_pair_a
            .into_iter()
            .map(|inst| cell.draw(inst))
            .collect::<Result<Vec<_>>>()?;
        let _precharge_pair_b = precharge_pair_b
            .into_iter()
            .map(|inst| cell.draw(inst))
            .collect::<Result<Vec<_>>>()?;

        cell.set_top_layer(2);
        cell.set_router(GreedyRouter);
        cell.set_via_maker(Sky130ViaMaker);

        io.layout.vdd.set_primary(ntap.layout.io().vpb.primary);
        io.layout.vss.set_primary(ptap.layout.io().vnb.primary);

        let m1slice = cell.layer_stack.slice(0..2);

        let mut lcm_tracks = Vec::new();
        lcm_tracks.push(
            m1slice
                .shrink_to_lcm_units(tail_pair[0].layout.io().g.primary.bbox().unwrap())
                .unwrap()
                .bot(),
        );
        for io in [input_pair[0].layout.io(), inv_pmos_pair[0].layout.io()] {
            let bot_track = m1slice
                .expand_to_lcm_units(io.g.primary.bbox().unwrap())
                .bot();
            lcm_tracks.push(bot_track);
            lcm_tracks.push(bot_track + 1);
        }

        for (i, port) in [
            io.schematic.clock,
            io.schematic.input.p,
            io.schematic.input.n,
            io.schematic.output.p,
            io.schematic.output.n,
        ]
            .into_iter()
            .enumerate()
        {
            cell.assign_grid_points(
                port,
                1,
                Rect::from_spans(strongarm_lcm_hspan, Span::new(lcm_tracks[i], lcm_tracks[i] + 1)),
            );
        }

        let m1slice = cell.layer_stack.slice(0..2);

        let io_rects = lcm_tracks
            .into_iter()
            .map(|track| {
                m1slice
                    .lcm_to_physical_rect(Rect::from_spans(
                        strongarm_lcm_hspan,
                        Span::from_point(track),
                    ))
                    .expand_dir(Dir::Vert, 200)
            })
            .collect::<Vec<_>>();

        for (i, port) in [
            &mut io.layout.clock,
            &mut io.layout.input.p,
            &mut io.layout.input.n,
            &mut io.layout.output.p,
            &mut io.layout.output.n,
        ]
            .into_iter()
            .enumerate()
        {
            cell.layout
                .draw(Shape::new(cell.ctx().layers.met1, io_rects[i]))?;
            port.set_primary(IoShape::with_layers(cell.ctx().layers.met1, io_rects[i]));
        }

        Ok(((), ()))
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::strongarm::tb::{ComparatorDecision, StrongArmTranTb};
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
    use crate::sky130_ctx;

    #[test]
    fn strongarm_sim() {
        let work_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/build/strongarm_sim");
        let dut = TileWrapper::new(StrongArmInstance {
            half_tail_w: 1_250,
            input_pair_w: 4_000,
            inv_nmos_w: 2_000,
            inv_pmos_w: 1_000,
            precharge_w: 1_000,
        });
        let pvt = Pvt {
            corner: Sky130Corner::Tt,
            voltage: dec!(1.8),
            temp: dec!(25.0),
        };
        let ctx = sky130_ctx();

        for i in 3..=10 {
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

                if vinp < dec!(0.5) || vinp > dec!(1.8) {
                    continue;
                }

                let tb = StrongArmTranTb {
                    dut: dut.clone(),
                    vinp,
                    vinn,
                    pvt,
                };
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
    fn strongarm_lvs() {
        let work_dir = PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/build/strongarm_lvs"
        ));
        let gds_path = work_dir.join("layout.gds");
        let netlist_path = work_dir.join("netlist.sp");
        let ctx = sky130_ctx();

        let block = TileWrapper::new(StrongArmInstance {
            half_tail_w: 1_250,
            input_pair_w: 4_000,
            inv_nmos_w: 2_000,
            inv_pmos_w: 1_000,
            precharge_w: 1_000,
        });

        let scir = ctx
            .export_scir(block.clone())
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
