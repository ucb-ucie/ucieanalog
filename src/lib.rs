//! Process-portable analog/mixed-signal generators for a
//! Universal Chiplet Interconnect Express (UCIe)
//! physical layer implementation.
#![warn(missing_docs)]

use sky130pdk::Sky130Pdk;
use spectre::Spectre;
use substrate::context::{Context, PdkContext};

pub mod buffer;
pub mod driver;
pub mod strongarm;
pub mod tech;
pub mod tiles;

/// Returns a configured SKY130 context.
pub fn sky130_ctx() -> PdkContext<Sky130Pdk> {
    let pdk_root = std::env::var("SKY130_COMMERCIAL_PDK_ROOT")
        .expect("the SKY130_COMMERCIAL_PDK_ROOT environment variable must be set");
    Context::builder()
        .install(Spectre::default())
        .install(Sky130Pdk::commercial(pdk_root))
        .build()
        .with_pdk()
}
