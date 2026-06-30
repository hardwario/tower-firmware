//! storage — key-value persistence in the L0 EEPROM
//! ([`storage`](tower::storage) block).
//!
//! Shows both value styles of the shared key-value store, surviving reset/power-cycle:
//!   * a **raw scalar** — a boot counter stored as `u32` little-endian bytes
//!     (no serializer), and
//!   * a **postcard record** — a `Settings` struct (any serde type).
//!
//! Adding a new key never disturbs existing ones, so this is also how you evolve
//! stored data: add a key instead of growing a struct. Reset the board a few
//! times with `just run example storage` and watch the boot count climb.
//!
//!   just run example storage

#![no_std]
#![no_main]

use embassy_time::Timer;
use log::{error, info};
use serde::{Deserialize, Serialize};
use tower::storage::NS_APP;
use tower::{app, board::Board};

// Locals within NS_APP (any u8 — the namespace prefix keeps the full key nonzero, so even
// local 0 is fine). Add a fresh local to add a new value.
const KEY_BOOTS: u8 = 0x00; // raw u32 (local within NS_APP)
const KEY_SETTINGS: u8 = 0x01; // postcard struct (local within NS_APP)

#[derive(Serialize, Deserialize, Debug)]
struct Settings {
    interval_s: u16,
    name: [u8; 4],
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            interval_s: 10,
            name: *b"TOWR",
        }
    }
}

async fn run(b: Board) {
    let kv = b.kv.scope(NS_APP); // this app's namespaced view; locals are u8, can't hit SDK keys

    // --- raw scalar: a persistent boot counter ------------------------------
    let mut buf = [0u8; 4];
    let boots = match kv.get_bytes(KEY_BOOTS, &mut buf) {
        Ok(Some(4)) => u32::from_le_bytes(buf),
        _ => 0, // absent / blank / unexpected -> start at zero
    } + 1;
    if let Err(e) = kv.set_bytes(KEY_BOOTS, &boots.to_le_bytes()) {
        error!(target: "storage", "boot-counter save failed: {e}");
    }

    // --- postcard record: a settings struct ---------------------------------
    let settings = match kv.get::<Settings>(KEY_SETTINGS) {
        Ok(Some(s)) => s,
        Ok(None) => {
            let s = Settings::default();
            if let Err(e) = kv.set(KEY_SETTINGS, &s) {
                error!(target: "storage", "settings save failed: {e}");
            }
            info!(target: "storage", "settings initialized to defaults");
            s
        }
        Err(e) => {
            error!(target: "storage", "settings load failed: {e}");
            Settings::default()
        }
    };

    info!(target: "storage", "boot #{}, settings {:?}", boots, settings);

    loop {
        Timer::after_secs(60).await;
    }
}

app!(run);
