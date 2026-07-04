//! Boot-loop guard.
//!
//! A unit wedged in a reset loop — a persistent hang caught by the [`watchdog`](crate::watchdog),
//! or a brown-out cycle — would otherwise rewrite per-boot EEPROM state (the console session
//! counter, the radio TX watermark) on *every* reset, slowly grinding the data EEPROM (see
//! `docs/storage.md`). This counts consecutive resets that never reached a healthy uptime in a
//! **reset-surviving `.uninit` RAM word** — retained across a warm reset but never persisted, so
//! it costs **zero EEPROM wear** — and lets the SDK stop persisting per-boot state once a loop is
//! detected ([`console::init_session`](crate::console::init_session) does exactly that).
//!
//! A cold boot (random RAM) is told from a warm reset by a magic marker; a brown-out that loses
//! RAM retention simply reads as a cold boot (run length 0), which is the safe default.

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

/// Marks [`STATE`] as a live boot-guard record (vs cold-boot RAM garbage).
const MAGIC: u32 = 0xB007_10AD;
/// Consecutive fast resets — each failing to reach [`HEALTHY_UPTIME`] — before a unit is judged to
/// be boot-looping and the SDK suppresses per-boot EEPROM writes.
pub const BOOT_LOOP_THRESHOLD: u32 = 8;
/// Uptime a boot must survive to be declared healthy, which clears the consecutive-reset run.
const HEALTHY_UPTIME: Duration = Duration::from_secs(30);

/// `[magic, resets]`, placed in `.uninit` so it survives a warm reset (RAM is retained) but is
/// **not** zeroed at start-up. Garbage on a cold boot, caught by the magic. `[u32; 2]` has no
/// invalid bit patterns and is only ever touched via volatile ops, so reading pre-init RAM here is
/// well-defined (a volatile read is not assumed to yield `poison`).
#[unsafe(link_section = ".uninit.TOWER_BOOTGUARD")]
static mut STATE: [u32; 2] = [0; 2];

/// The consecutive-reset run observed by the last [`on_boot`] (0 = cold boot / healthy). Cached in
/// `.bss` so [`consecutive_resets`] can report it without re-touching `STATE`.
static RESETS: AtomicU32 = AtomicU32::new(0);

/// Record this boot and return the consecutive-reset run length (0 on a cold boot). Call **once**,
/// early in start-up. Spawns a task that clears the run after [`HEALTHY_UPTIME`], so a unit that
/// stays up is not counted as looping on its next, unrelated reboot.
pub fn on_boot(spawner: Spawner) -> u32 {
    let p = addr_of_mut!(STATE);
    // SAFETY: single-threaded start-up; volatile access to RAM retained across a warm reset (or
    // arbitrary-but-valid u32s on a cold boot, which the magic rejects → run length 0).
    let cur = unsafe { p.read_volatile() };
    let resets = if cur[0] == MAGIC {
        cur[1].saturating_add(1)
    } else {
        0
    };
    unsafe { p.write_volatile([MAGIC, resets]) };
    RESETS.store(resets, Ordering::Relaxed);
    spawner.spawn(healthy_watch().unwrap());
    resets
}

/// The consecutive-reset run length from the last [`on_boot`] — for apps that want to back off
/// their own per-boot persistence (or skip arming the watchdog) while a unit is looping.
pub fn consecutive_resets() -> u32 {
    RESETS.load(Ordering::Relaxed)
}

/// Whether the unit appears to be boot-looping (run ≥ [`BOOT_LOOP_THRESHOLD`]). While true the SDK
/// suppresses per-boot EEPROM writes to spare the store.
pub fn is_looping() -> bool {
    consecutive_resets() >= BOOT_LOOP_THRESHOLD
}

/// Clear the reset run once the unit proves a healthy uptime — so the *next* reboot (after a long,
/// successful session) starts a fresh count rather than inheriting this boot's run.
#[embassy_executor::task]
async fn healthy_watch() {
    Timer::after(HEALTHY_UPTIME).await;
    let p = addr_of_mut!(STATE);
    // SAFETY: single-threaded executor; sequential with `on_boot`'s write, no aliasing.
    unsafe { p.write_volatile([MAGIC, 0]) };
    RESETS.store(0, Ordering::Relaxed);
}
