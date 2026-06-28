//! EU 868 **LBT + AFA** (Listen-Before-Talk + Adaptive Frequency Agility,
//! EN 300 220) — the EU way to relax the 1 % duty cap. `impl Net` block over
//! [`super::Net`].
//!
//! Unlike US FHSS, **no time-synchronization** is needed: the node listens before
//! every TX (CCA via the SPIRIT1 CSMA engine) and, if a channel is busy or still
//! in its post-TX off-time, hops to another channel in the small AFA set; the
//! gateway simply *scans* the set and ACKs on whatever channel it caught the frame.
//! Both ends favour a shared `primary` channel, so in the clear case they rendezvous
//! immediately and only spread under contention.
//!
//! **Politeness model (replaces the duty cap):** mandatory LBT before each TX +
//! a minimum per-channel off-time after a TX (forcing agility). The exact EN 300 220
//! CCA time/threshold, min-channel count and off-time are config to **verify** before
//! any product claim; the constants here are bench defaults that exercise the mechanism.

use embassy_time::{Duration, Instant, Timer};

use super::{Access, Net, Received, SendResult, TX_TIMEOUT};
use crate::radio::config::{self, AFA_N, Band, afa_freq_hz};
use crate::radio::device::RadioError;
use crate::radio::frame::{self, FrameType, Header, MAX_FRAME, MAX_PAYLOAD, flags};

/// Per-channel minimum off-time after a TX: the channel can't be re-used until this
/// elapses, forcing the node onto another channel (the "agility"). **Verify** vs
/// EN 300 220; bench default.
const AFA_OFFTIME: Duration = Duration::from_millis(100);
/// Gateway per-channel listen slice while scanning the AFA set.
const AFA_SCAN_SLICE: Duration = Duration::from_millis(80);

/// Which side of the EU LBT+AFA link this device plays.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AfaRole {
    /// Transmits with LBT + agility ([`Net::afa_send`]).
    Node,
    /// Scans the channel set and auto-ACKs ([`Net::afa_serve`]).
    Gateway,
}

/// EU LBT+AFA configuration. `primary` defaults to channel 0.
#[derive(Debug, Clone, Copy, Default)]
pub struct AfaConfig {
    /// Preferred channel index (0..[`AFA_N`)): the channel both ends favour when clear.
    pub primary: u8,
}

/// EU LBT+AFA runtime state, held on [`Net`] (inert unless `access == Afa`).
pub(crate) struct Afa {
    role: AfaRole,
    primary: u8,
    /// Channel the node is currently parked on (sticky between sends).
    cur: u8,
    /// Per-channel last-TX instant for the off-time politeness rule.
    last_tx: [Option<Instant>; AFA_N as usize],
}

impl Afa {
    pub(crate) fn disabled() -> Self {
        Self {
            role: AfaRole::Node,
            primary: 0,
            cur: 0,
            last_tx: [None; AFA_N as usize],
        }
    }
}

impl Net {
    /// Enable EU 868 LBT+AFA at runtime (mutually exclusive with other access modes,
    /// like `set_band`). Tunes to the `primary` channel; `role` selects
    /// [`afa_send`](Self::afa_send) (node) vs [`afa_serve`](Self::afa_serve) (gateway).
    pub async fn enable_afa(&mut self, role: AfaRole, cfg: AfaConfig) -> Result<(), RadioError> {
        let primary = cfg.primary % AFA_N;
        self.afa = Afa {
            role,
            primary,
            cur: primary,
            last_tx: [None; AFA_N as usize],
        };
        self.access = Access::Afa;
        config::set_freq_hz(&mut self.radio, afa_freq_hz(primary)).await
    }

    /// Leave AFA → plain EU duty-cycle mode on the band base channel.
    pub async fn disable_afa(&mut self) -> Result<(), RadioError> {
        self.access = Access::Duty;
        config::set_band(&mut self.radio, Band::Eu868, 0).await
    }

    /// The AFA role this device was enabled as (diagnostics).
    #[must_use]
    pub fn afa_role(&self) -> AfaRole {
        self.afa.role
    }

    /// Current AFA channel index (diagnostics).
    #[must_use]
    pub fn afa_channel(&self) -> u8 {
        self.afa.cur
    }

    /// Whether channel `ch` is past its post-TX off-time (re-usable).
    fn afa_channel_ready(&self, ch: u8, now: Instant) -> bool {
        match self.afa.last_tx[ch as usize] {
            Some(t) => now.saturating_duration_since(t) >= AFA_OFFTIME,
            None => true,
        }
    }

    /// **Node:** send `data` to `dest` with LBT + frequency agility. For each
    /// (re)transmission it scans the AFA set from the current channel, skips any
    /// channel still in its off-time, runs CCA (LBT) on the next candidate, and
    /// transmits on the first one the channel-assessment finds free; a confirmed
    /// send then waits for the ACK on that same channel. Returns `Busy` only if
    /// every channel was busy/in-off-time for a whole attempt. One TX counter is
    /// consumed (docs/radio.md), matching [`send`](Self::send).
    pub async fn afa_send(&mut self, dest: u32, data: &[u8], confirmed: bool, reps: u8) -> SendResult {
        if self.access != Access::Afa {
            return SendResult::WrongMode;
        }
        if data.len() > MAX_PAYLOAD {
            return SendResult::Error(RadioError::TooLong);
        }
        let my_id = self.my_id;
        let counter = self.tx_counter;
        let key = self.key_for(dest);
        let hdr = Header {
            frame_type: FrameType::Data,
            flags: if confirmed { flags::CONFIRMED } else { 0 },
            src: my_id,
            dest,
            counter,
            bulk_index: None,
        };
        let mut buf = [0u8; MAX_FRAME];
        let n = match frame::seal_frame(&mut self.ccm, &key, &hdr, data, &mut buf) {
            Ok(n) => n,
            Err(_) => return SendResult::Error(RadioError::TooLong),
        };

        let reps = if confirmed { reps.clamp(1, 10) } else { 1 };
        let mut result = SendResult::NotDelivered;
        'reps: for attempt in 0..reps {
            if attempt > 0 {
                Timer::after(Duration::from_millis(self.backoff_ms() as u64)).await;
            }
            // Agility: from the current channel, find the first off-time-clear
            // channel that LBT (CCA) reports free, and transmit there.
            let mut sent = false;
            for step in 0..AFA_N {
                let ch = (self.afa.cur + step) % AFA_N;
                if !self.afa_channel_ready(ch, Instant::now()) {
                    continue;
                }
                if config::set_freq_hz(&mut self.radio, afa_freq_hz(ch))
                    .await
                    .is_err()
                {
                    continue;
                }
                match self
                    .radio
                    .tx(&buf[..n], /*use_csma (LBT)=*/ true, TX_TIMEOUT)
                    .await
                {
                    Ok(()) => {
                        self.afa.cur = ch;
                        self.afa.last_tx[ch as usize] = Some(Instant::now());
                        sent = true;
                        break;
                    }
                    Err(RadioError::Busy) => continue, // LBT: busy → try the next channel
                    Err(e) => {
                        result = SendResult::Error(e);
                        break 'reps;
                    }
                }
            }
            if !sent {
                result = SendResult::DutyLimited; // every channel busy / in off-time
                continue;
            }
            if !confirmed {
                result = SendResult::Delivered;
                break;
            }
            if self.await_ack(dest, counter).await {
                result = SendResult::Delivered;
                break;
            }
        }
        self.advance_tx_counter();
        result
    }

    /// **Gateway:** scan the AFA channel set (favouring `primary`) for up to
    /// `timeout`, dwelling [`AFA_SCAN_SLICE`] per channel. On the first authenticated
    /// frame, decode it, auto-ACK on that channel (via [`recv`](Self::recv)), and
    /// return it. `None` on timeout. No epoch/beacon — the node's channel choice +
    /// retransmissions guarantee a catch within a scan cycle or two.
    pub async fn afa_serve(&mut self, timeout: Duration) -> Option<Received> {
        let deadline = Instant::now().checked_add(timeout)?;
        while Instant::now() < deadline {
            for step in 0..AFA_N {
                let ch = (self.afa.primary + step) % AFA_N;
                if config::set_freq_hz(&mut self.radio, afa_freq_hz(ch))
                    .await
                    .is_err()
                {
                    continue;
                }
                if let Some(rx) = self.recv(AFA_SCAN_SLICE).await {
                    self.afa.cur = ch;
                    return Some(rx);
                }
                if Instant::now() >= deadline {
                    break;
                }
            }
        }
        None
    }
}
