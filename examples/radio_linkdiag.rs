//! radio_linkdiag — deep instrumentation of the TX and RX state machines / FIFOs
//! / interrupts, to verify the packet-handling *infrastructure* (not just RF
//! registers).
//!
//!   TOWER_FEATURES=role-node    just flash radio_linkdiag   # TX: trace each send
//!   TOWER_FEATURES=role-gateway just flash radio_linkdiag   # RX: poll the FIFO
//!
//! TX role: loads a known payload, confirms it landed in the TX FIFO, strobes TX,
//! traces the MC_STATE progression (READY→SYNTH→LOCK→TX→READY), reads the result
//! IRQ, and confirms the FIFO drained — proving the transmitter really sends a
//! packet. RX role: strobes RX and polls ELEM_RXFIFO directly; if *any* bytes
//! arrive it dumps them raw — proving (or disproving) reception below the IRQ /
//! length-decode layer.

#![no_std]
#![no_main]

use embassy_time::{Duration, Timer};
use log::info;
use tower::radio::{RfConfig, Spirit1, config, regs};
use tower::{app, board::Board};

async fn run(b: Board) {
    let mut radio = Spirit1::new(
        b.radio_spi,
        b.radio_sck,
        b.radio_mosi,
        b.radio_miso,
        b.radio_cs,
        b.radio_sdn,
        b.radio_irq,
    );
    let _ = radio.exit_shutdown().await;
    let _ = radio.read_device_id();
    let cfg = RfConfig {
        band: config::Band::DEFAULT,
        channel: 0,
    };
    let _ = config::apply(&mut radio, &cfg).await;

    #[cfg(feature = "role-node")]
    tx_trace(&mut radio).await;
    #[cfg(not(feature = "role-node"))]
    rx_poll(&mut radio).await;
}

#[cfg(feature = "role-node")]
async fn tx_trace(radio: &mut Spirit1) -> ! {
    const LEN: usize = 16;
    let mut seq: u32 = 0;
    info!(target: "txdiag", "TX trace: load FIFO, strobe TX, watch the state machine");
    loop {
        // Build a known payload.
        let mut payload = [0xA5u8; LEN];
        payload[..4].copy_from_slice(&seq.to_le_bytes());

        let _ = radio.to_ready().await;
        let _ =
            radio.set_irq_mask(regs::IRQ_TX_DATA_SENT | regs::IRQ_MAX_BO_CCA_REACH | regs::IRQ_TX_FIFO_ERROR);
        // TX-mode SMPS + VCO_L buffer (the per-strobe WA).
        let _ = radio.spi().write_reg(regs::PM_CONFIG1, 0x20);
        let _ = radio.spi().write_reg(0xA9, 0x11);
        let _ = radio.spi().command(regs::CMD_FLUSHTXFIFO);
        let _ = radio.spi().write_regs(regs::PCKTLEN1, &[0, LEN as u8]);
        let _ = radio.spi().write_fifo(&payload);

        let loaded = radio.tx_fifo_count().unwrap_or(0xFF);
        let _ = radio.irq_status(); // clear

        let _ = radio.spi().command(regs::CMD_TX);

        // Trace distinct MC_STATE values for ~12 ms.
        let mut trace = [0xFFu8; 12];
        let mut n = 0usize;
        let mut last = 0xFE;
        for _ in 0..60 {
            let s = radio.mc_state().unwrap_or(0xFD);
            if s != last {
                if n < trace.len() {
                    trace[n] = s;
                    n += 1;
                }
                last = s;
            }
            Timer::after(Duration::from_micros(200)).await;
        }
        let irq = radio.irq_status().unwrap_or(0);
        let after = radio.tx_fifo_count().unwrap_or(0xFF);

        info!(
            target: "txdiag",
            "seq={} fifo_loaded={} (expect {}) | states={} | irq=0x{:08X} tx_sent={} | fifo_after={}",
            seq, loaded, LEN, StateTrace(&trace[..n]), irq,
            irq & regs::IRQ_TX_DATA_SENT != 0, after
        );
        seq = seq.wrapping_add(1);
        Timer::after_millis(1000).await;
    }
}

#[cfg(not(feature = "role-node"))]
async fn rx_poll(radio: &mut Spirit1) -> ! {
    info!(target: "rxdiag", "RX poll: strobing RX, polling ELEM_RXFIFO directly");
    let _ = radio.to_ready().await;
    let _ = radio.set_irq_mask(
        regs::IRQ_RX_DATA_READY
            | regs::IRQ_VALID_SYNC
            | regs::IRQ_VALID_PREAMBLE
            | regs::IRQ_CRC_ERROR
            | regs::IRQ_RX_DATA_DISC
            | regs::IRQ_RX_FIFO_ERROR,
    );
    let _ = radio.spi().write_reg(regs::PM_CONFIG1, 0x98); // RX-mode SMPS
    let _ = radio.spi().command(regs::CMD_FLUSHRXFIFO);
    let _ = radio.irq_status();
    let _ = radio.spi().command(regs::CMD_RX);

    let mut acc_irq: u32 = 0;
    let mut max_fifo: u8 = 0;
    let mut ticks: u32 = 0;
    loop {
        // Direct FIFO-level reception check.
        let f = radio.rx_fifo_count().unwrap_or(0);
        if f > max_fifo {
            max_fifo = f;
        }
        if f > 0 {
            let mut buf = [0u8; 96];
            let m = (f as usize).min(buf.len());
            let _ = radio.spi().read_fifo(&mut buf[..m]);
            let plen = radio.rx_packet_len().unwrap_or(0);
            info!(target: "rxdiag", ">>> RX FIFO {} bytes, RX_PCKT_LEN={}: {}", f, plen, HexN(&buf[..m.min(20)]));
            let _ = radio.spi().command(regs::CMD_SABORT);
            let _ = radio.spi().command(regs::CMD_FLUSHRXFIFO);
            let _ = radio.spi().command(regs::CMD_RX);
        }
        acc_irq |= radio.irq_status().unwrap_or(0);

        ticks += 1;
        if ticks.is_multiple_of(500) {
            // ~1 s (500 * 2 ms)
            let st = radio.mc_state().unwrap_or(0xFF);
            info!(
                target: "rxdiag",
                "state=0x{:02X} max_fifo={} irq=0x{:08X} [preamble={} sync={} ready={} crc={} disc={} fifoerr={}]",
                st, max_fifo, acc_irq,
                acc_irq & regs::IRQ_VALID_PREAMBLE != 0,
                acc_irq & regs::IRQ_VALID_SYNC != 0,
                acc_irq & regs::IRQ_RX_DATA_READY != 0,
                acc_irq & regs::IRQ_CRC_ERROR != 0,
                acc_irq & regs::IRQ_RX_DATA_DISC != 0,
                acc_irq & regs::IRQ_RX_FIFO_ERROR != 0,
            );
            acc_irq = 0;
            max_fifo = 0;
            // Re-arm if it left RX.
            if st != regs::STATE_RX {
                let _ = radio.spi().command(regs::CMD_FLUSHRXFIFO);
                let _ = radio.spi().command(regs::CMD_RX);
            }
        }
        Timer::after(Duration::from_millis(2)).await;
    }
}

/// Format a state-code trace like `03->4F->0F->5F->03`.
#[cfg(feature = "role-node")]
struct StateTrace<'a>(&'a [u8]);
#[cfg(feature = "role-node")]
impl core::fmt::Display for StateTrace<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for (i, s) in self.0.iter().enumerate() {
            if i > 0 {
                write!(f, "->")?;
            }
            write!(f, "{:02X}", s)?;
        }
        Ok(())
    }
}

#[cfg(not(feature = "role-node"))]
struct HexN<'a>(&'a [u8]);
#[cfg(not(feature = "role-node"))]
impl core::fmt::Display for HexN<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for b in self.0 {
            write!(f, "{:02x} ", b)?;
        }
        Ok(())
    }
}

app!(run);
