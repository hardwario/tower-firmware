//! fota_ota — the **real-firmware** over-the-air swap, end to end (docs/fota.md).
//!
//!   # 1) build + sign the update image the host will serve:
//!   just fota-update
//!   # 2) node (ACTIVE-linked, under the bootloader; bootloader+node merged into one flash):
//!   TOWER_PORT=<node-port> TOWER_FEATURES=role-node just flash-fota fota_ota
//!   # 3) gateway (normal app; proxies the image from the host over USB):
//!   TOWER_FEATURES=role-gateway TOWER_PORT=<gw-port> just flash example fota_ota
//!   # 4) host (streams the signed update image to the gateway on demand):
//!   tower -d <gw-port> fota serve --image target/fota-update.bin \
//!                                 --manifest target/fota-update.fmanifest
//!
//! This serves a **real signed firmware** the host holds, and the node actually swaps to it.
//! The node never verifies or arms the swap itself — it stages the image, stashes the signed
//! manifest, and resets; the **bootloader** does the crypto and the swap:
//!
//!   node(v1) advertised → pull manifest → cheap policy (decode/rollback/fits) → stream image
//!     into DFU → stash signed manifest → reset
//!       → BOOTLOADER verifies Ed25519 + SHA-256(DFU) → swaps ACTIVE⇄DFU → node(v2) boots
//!       → mark_booted + persist installed_version → *** UPDATE CONFIRMED ***
//!
//! The gateway holds no image: each chunk the node requests is fetched from the host via
//! `HostProxySource` (docs/fota.md). See `docs/fota.md`.

#![no_std]
#![no_main]

use log::{error, info};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig};
use tower::{app, board::Board};

#[cfg(feature = "role-node")]
use {
    core::cell::RefCell,
    cortex_m::peripheral::SCB,
    embassy_boot_stm32::{AlignedBuffer, BlockingFirmwareState, FirmwareUpdaterConfig, State},
    embassy_stm32::flash::WRITE_SIZE,
    embassy_sync::blocking_mutex::Mutex,
    embassy_sync::blocking_mutex::raw::NoopRawMutex,
    embassy_time::Timer,
    log::warn,
    tower::fota::{PullOutcome, promote_pending_version, pull_update},
    tower::radio::net::SendResult,
    tower::storage::Nv,
};
#[cfg(not(feature = "role-node"))]
use tower::fota::HostProxySource;

#[cfg(feature = "role-node")]
const NODE_ID: u32 = 0xF0_7A_5A_01;
const GW_ID: u32 = 0xF0_7A_5A_02;
const KEY: [u8; 16] = [
    0xF0, 0x7A, 0x5A, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x02,
];

/// Band for this **bench demo**: `Us915`, whose duty governor is unrestricted, so a full
/// ~67 KB image transfers in one shot. EU 868's 1 % duty throttles a full-image bulk transfer
/// (≈ the whole hourly airtime budget — HW-observed stalling ~82 % in); `pull_update`'s
/// **resume** carries it across duty windows there (slow — `PullOutcome::InProgress`, retried),
/// so `Band::Eu868` works for a real EU node. 915 MHz single-channel is **bench-only** (not FCC
/// §15.247-compliant — see docs/radio.md); here it just keeps the demo to one pass.
const BAND: Band = Band::Us915;

/// This image's firmware version — bumped by `fota-v2`. The host serves the `fota-v2` build
/// as the update; after the swap the booted v2 persists this as the installed version.
#[cfg(all(feature = "role-node", not(feature = "fota-v2")))]
const VERSION: u32 = 1;
#[cfg(all(feature = "role-node", feature = "fota-v2"))]
const VERSION: u32 = 2;

async fn run(b: Board) {
    #[cfg(feature = "role-node")]
    node(b).await;
    #[cfg(not(feature = "role-node"))]
    gateway(b).await;
}

// --------------------------------------------------------------------------- node

#[cfg(feature = "role-node")]
async fn node(b: Board) -> ! {
    info!(target: "fota-ota", "NODE v{VERSION} {:08X} (ACTIVE slot, under bootloader)", NODE_ID);
    #[cfg(feature = "fota-diag")]
    {
        info!(target: "fota-ota", "diag: entered node (console alive, pre-radio)");
        Timer::after_millis(400).await;
    }
    // Boot-state machine, done in a SYNCHRONOUS, `#[inline(never)]` helper (see
    // `check_boot_state`): its embassy-boot locals get their own transient stack frame
    // instead of inflating this radio task's huge async poll frame — critical on the 20 KB
    // L0, where the combined frame would otherwise overflow the stack. Returns the reclaimed
    // Kv + whether this is a fresh boot (vs a just-confirmed swap / revert).
    let fresh = check_boot_state(b.kv);
    if !fresh {
        // We just confirmed (or reverted) — promote the *pending* version recorded when this
        // image was staged (the actual signed manifest version) to the rollback floor an offered
        // image must strictly supersede, then idle. Promoting the pending value rather than this
        // image's compile-time `VERSION` keeps the floor aligned with what was advertised, so a
        // manifest whose `--version` differs from the baked constant can't cause an endless
        // reinstall loop (C5). `promote_pending_version` takes the shared `Nv` directly, since
        // `Net` does not exist yet on this path.
        let floor = promote_pending_version(b.kv);
        info!(target: "fota-ota", "confirmed — rollback floor now v{floor}");
        idle().await;
    }

    let radio = Spirit1::new(
        b.radio_spi,
        b.radio_sck,
        b.radio_mosi,
        b.radio_miso,
        b.radio_cs,
        b.radio_sdn,
        b.radio_irq,
    );
    let mut net = match Net::new(
        radio,
        b.kv,
        NetConfig {
            my_id: NODE_ID,
            key: KEY,
            band: BAND,
            channel: 0,
        },
    )
    .await
    {
        Ok(n) => n,
        Err(e) => {
            error!(target: "fota-ota", "net init: {e}");
            idle().await;
        }
    };
    #[cfg(feature = "fota-diag")]
    {
        info!(target: "fota-ota", "diag: net up, entering pull loop");
        Timer::after_millis(400).await;
    }

    // Pull loop: heartbeat → DOWNLINK_PENDING → PULL → pull_update. On Staged the image is in
    // DFU and the signed manifest is stashed; just reset — the bootloader verifies + swaps.
    loop {
        let pending = match net.send(GW_ID, b"HELLO", true, 3).await {
            SendResult::Delivered => net.take_downlink_pending(),
            other => {
                warn!(target: "fota-ota", "heartbeat: {other}");
                false
            }
        };
        if pending && net.send(GW_ID, b"PULL", true, 3).await == SendResult::Delivered {
            info!(target: "fota-ota", "update advertised — pulling");
            match pull_update(&mut net, GW_ID).await {
                PullOutcome::Staged { manifest } => {
                    let s = manifest.sha256;
                    info!(target: "fota-ota",
                        "staged v{} {} B sha {:02x}{:02x}{:02x}{:02x}.. — resetting; bootloader verifies + swaps (~2.5 min)",
                        manifest.version, manifest.size, s[0], s[1], s[2], s[3]);
                    Timer::after_millis(80).await; // let the console drain
                    SCB::sys_reset();
                }
                PullOutcome::InProgress { staged, total } => {
                    info!(target: "fota-ota", "staged {staged}/{total} B — resuming next cycle")
                }
                PullOutcome::NotNewer { version, installed } => {
                    info!(target: "fota-ota", "v{version} not newer than v{installed} — up to date")
                }
                PullOutcome::Malformed => error!(target: "fota-ota", "manifest malformed — rejected"),
                PullOutcome::TooLarge { size } => {
                    error!(target: "fota-ota", "image {size} B too large for ACTIVE")
                }
                PullOutcome::NoManifest | PullOutcome::ImageFailed => {
                    warn!(target: "fota-ota", "pull failed (host serving?) — retry")
                }
                PullOutcome::FloorUnavailable => {
                    error!(target: "fota-ota", "rollback floor unreadable — refusing (fail closed)")
                }
            }
        }
        Timer::after_secs(5).await;
    }
}

/// Confirm a just-swapped image / note a revert. Returns `fresh` (true on a clean boot, no swap
/// to confirm). The program Flash is borrowed **transiently** from the shared KV (via
/// [`Nv::with_flash`]) for the synchronous state read; the KV itself stays shared for the radio.
///
/// **Synchronous + `#[inline(never)]` on purpose:** the embassy-boot state machinery has a
/// sizable stack footprint, and inlining it into the node's async poll pushed that frame
/// over the L0's ~10 KB of stack (silent HardFault → reset loop). A plain `fn` gets its own
/// transient frame, freed before the radio work runs. The whole boot-state check is blocking
/// (no `.await`), so it needn't be `async`.
#[cfg(feature = "role-node")]
#[inline(never)]
fn check_boot_state(kv: Nv) -> bool {
    // `&mut Flash` is itself `NorFlash`, so embassy-boot drives the borrowed handle through the
    // same `Mutex<RefCell<_>>` shape; the borrow ends with the closure, before the radio runs.
    kv.with_flash(|flash| {
        let flash_mutex = Mutex::<NoopRawMutex, _>::new(RefCell::new(flash));
        let mut aligned = AlignedBuffer([0u8; WRITE_SIZE]);
        // BlockingFirmwareState = STATE only, no salty: the bootloader arms swaps; the app
        // only confirms (mark_booted) + reads state.
        let cfg = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash_mutex, &flash_mutex);
        let mut fw = BlockingFirmwareState::new(cfg.state, &mut aligned.0);
        match fw.get_state() {
            Ok(State::Swap) => {
                match fw.mark_booted() {
                    Ok(()) => info!(target: "fota-ota", "*** UPDATE CONFIRMED *** booted swapped v{VERSION}"),
                    Err(e) => warn!(target: "fota-ota", "mark_booted failed: {e:?}"),
                }
                false
            }
            Ok(State::Revert) => {
                error!(target: "fota-ota", "*** REVERTED *** swapped image failed to confirm; rolled back");
                false
            }
            Ok(State::Boot) => true,
            Ok(other) => {
                info!(target: "fota-ota", "boot state {other:?} (idle)");
                false
            }
            Err(e) => {
                warn!(target: "fota-ota", "get_state failed: {e:?}");
                false
            }
        }
    })
}

/// Heartbeat idle so the monitor shows the live version between cycles. Diverges.
#[cfg(feature = "role-node")]
async fn idle() -> ! {
    let mut tick: u32 = 0;
    loop {
        info!(target: "fota-ota", "v{VERSION} alive (tick {tick})");
        tick = tick.wrapping_add(1);
        Timer::after_secs(10).await;
    }
}

// ------------------------------------------------------------------------- gateway

#[cfg(not(feature = "role-node"))]
async fn gateway(b: Board) -> ! {
    let radio = Spirit1::new(
        b.radio_spi,
        b.radio_sck,
        b.radio_mosi,
        b.radio_miso,
        b.radio_cs,
        b.radio_sdn,
        b.radio_irq,
    );
    let mut net = match Net::new(
        radio,
        b.kv,
        NetConfig {
            my_id: GW_ID,
            key: KEY,
            band: BAND,
            channel: 0,
        },
    )
    .await
    {
        Ok(n) => n,
        Err(e) => {
            error!(target: "fota-ota", "net init: {e}");
            loop {
                embassy_time::Timer::after_secs(60).await;
            }
        }
    };
    info!(target: "fota-ota", "GATEWAY {:08X}: advertising; serves the host's image on PULL", GW_ID);
    net.set_downlink_pending(true);
    // The host's FotaData replies arrive on the console RX and are routed to the
    // host-proxy by the console manager (USB must be present — a gateway is USB-powered).

    loop {
        let Some(rx_msg) = net.recv(embassy_time::Duration::from_secs(10)).await else {
            continue;
        };
        if rx_msg.data() != b"PULL" {
            continue;
        }
        let node = rx_msg.src;
        info!(target: "fota-ota", "node {node:08X} requested update — fetching manifest from host");
        match HostProxySource::connect().await {
            Some((mut src, manifest)) => {
                let m = net.bulk_serve(node, &manifest).await; // 1) signed manifest
                let img = net.bulk_serve_from(node, &mut src).await; // 2) image (host-proxied)
                info!(target: "fota-ota", "served manifest={m} image={img}");
            }
            None => error!(target: "fota-ota", "host not serving (run `tower fota serve`?) — skip"),
        }
    }
}

app!(run, no_shell);
