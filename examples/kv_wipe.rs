//! kv_wipe — factory-reset the data EEPROM: zero the whole 6 KiB, returning the KV store to
//! its virgin state (next boot re-initializes fresh at generation 1).
//!
//! Exists because the store has **no key deletion**: a live set only ever grows (peer tables,
//! replay lanes, settings), and once it approaches a half's capacity every few appends force a
//! compaction flip — wear plus a multi-second CPU stall each time (docs/storage.md). Until a
//! selective delete/factory-reset lands in `tower-kv`, this example is the reset tool:
//!
//!   just flash example kv_wipe     # LED: slow blink = wiping, solid = done — then
//!   just flash example <your app>  # boots on a fresh store
//!
//! ⚠️ Erases EVERYTHING the store holds: pairing keys, peer tables, replay lanes, the TX
//! watermark, settings, session counter. A re-keyed/re-paired network is required after —
//! which is exactly the fail-closed behaviour the security model wants on a wiped store.
//!
//! Deliberately NOT an `app!`: it must run before any `Nv`/`Kv` is constructed over the
//! region it is zeroing, so it uses a bare entry with `board::init` + the raw `Storage`.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::flash::Flash;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;
use tower::storage::Storage;

#[embassy_executor::main(executor = "embassy_stm32::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = tower::board::init();
    let mut led = Output::new(p.PH1, Level::Low, Speed::Low);
    let mut storage = Storage::new(Flash::new_blocking(p.FLASH));

    // Zero the full EEPROM in 64 B chunks, blinking so the ~5 s of CPU-stalling writes read
    // as activity, not a hang. (Word writes stall the chip; the blink advances between chunks.)
    let zeros = [0u8; 64];
    let len = storage.len() as u32;
    let mut off = 0u32;
    while off < len {
        let n = 64u32.min(len - off);
        let _ = storage.write(off, &zeros[..n as usize]);
        off += n;
        if off.is_multiple_of(512) {
            led.toggle();
        }
    }

    // Done: LED solid. Flash the real firmware next; it boots on a virgin store.
    led.set_high();
    loop {
        Timer::after_secs(60).await;
    }
}
