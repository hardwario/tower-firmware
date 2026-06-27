//! SPIRIT1 RF configuration: band/channel, modulation, deviation, RX bandwidth,
//! sync word, CRC, packet format and PA — derived from the datasheet formulas
//! for the **50 MHz** crystal in the SPSGRF module.
//!
//! Crystal handling: f_XO = 50 MHz. The synthesizer and frequency-deviation
//! formulas use the full f_XO; the digital-domain blocks (data rate, channel
//! filter) use f_DIG = f_XO / 2 = 25 MHz (the SPIRIT1 halves the digital clock
//! for crystals > 30 MHz). IF offsets are taken straight from the datasheet's
//! 50 MHz row (Table 31): ANA = 0x36, DIG = 0xAC.
//!
//! All derivations are constants here (verified against the reset values, which
//! correspond to 868 MHz / 38.4 kbps at 26 MHz). Step 3 proves them on hardware
//! via the CW carrier + partner RSSI; Step 4 proves the digital-domain values
//! (data rate / deviation / filter) via a real modulated link.


use super::device::{RadioError, Spirit1};
use super::regs;
use super::spi::Spirit1Spi;

/// Crystal frequency of the SPSGRF module.
pub const F_XO: u64 = 50_000_000;

/// Reference divider D (Eq. 6). The 50 MHz crystal exceeds the synthesizer's
/// reference range, so REFDIV is enabled (D=2 → 25 MHz PLL reference), and the
/// SYNT divider is scaled accordingly.
const REFDIV: u64 = 2;

/// Operating band. Both lie in the SPIRIT1 *high* VCO band (779–956 MHz), so they
/// share the synthesizer band-select (B=6, BS=1) and every digital-domain setting
/// (modulation, deviation, RX filter, IF) — only the base frequency and channel
/// spacing differ, derived from these accessors.
///
/// **Regulatory note:** EU 868 operation here respects the 1 % duty cycle (§2.2).
/// US 915 is provided for **test/development**: this is a ~216 kHz narrowband
/// signal, which is *not* FCC 15.247-compliant (that requires FHSS over ≥50
/// channels or ≥500 kHz digital bandwidth). Use it on the bench, not in a product,
/// without adding a compliant channel-hopping / wideband scheme.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Band {
    /// EU 868 MHz: base 868.1 MHz, 200 kHz spacing, ch0/1/2 = 868.1/868.3/868.5.
    Eu868,
    /// US 915 MHz (902–928 ISM): base 915.0 MHz, 200 kHz spacing,
    /// ch0/1/2 = 915.0/915.2/915.4. Test/dev only (see the type-level note).
    Us915,
}

impl Band {
    /// Band the examples come up on; they can retune at runtime with
    /// [`Net::set_band`](crate::radio::net::Net::set_band) (a single binary supports
    /// both bands — selection is a runtime decision, never a build flag).
    pub const DEFAULT: Band = Band::Eu868;

    /// Base (channel-0) carrier frequency in Hz.
    pub const fn base_hz(self) -> u64 {
        match self {
            Band::Eu868 => 868_100_000,
            Band::Us915 => 915_000_000,
        }
    }
    /// Channel spacing in Hz. Must keep `CHSPACE = spacing·2^15/f_XO ≤ 255`
    /// (≈389 kHz max at 50 MHz).
    pub const fn spacing_hz(self) -> u64 {
        match self {
            Band::Eu868 | Band::Us915 => 200_000,
        }
    }
    /// Synthesizer band-select divider B (Eq. 5). Both bands are high band → B=6.
    const fn b_div(self) -> u64 {
        match self {
            Band::Eu868 | Band::Us915 => 6,
        }
    }
    /// BS field value for SYNT0 (1 = high band).
    const fn bs_field(self) -> u8 {
        match self {
            Band::Eu868 | Band::Us915 => 1,
        }
    }
}

/// Full RF configuration for a network.
#[derive(Debug, Clone, Copy)]
pub struct RfConfig {
    pub band: Band,
    /// Channel index (0..=2). A node and its gateway must share one.
    pub channel: u8,
}

impl Default for RfConfig {
    fn default() -> Self {
        Self {
            band: Band::Eu868,
            channel: 0,
        }
    }
}

/// Per-reception signal quality (docs/radio.md).
#[derive(Debug, Clone, Copy, Default)]
pub struct SignalQuality {
    /// RSSI in dBm.
    pub rssi_dbm: i16,
    /// Raw RSSI register value (0.5 dB/step).
    pub rssi_raw: u8,
    /// Link/packet quality indicator (the SPIRIT1's PQI).
    pub lqi: u8,
    /// Sync quality indicator.
    pub sqi: u8,
    /// AFC correction word of the packet (signed), for the §2.1 BW-narrowing sweep.
    pub afc_raw: i8,
}

/// SPIRIT1 RSSI conversion offset: `dBm = raw/2 - 130` (datasheet §9.10.1).
const RSSI_OFFSET_DBM: i16 = 130;

/// Convert a raw RSSI register value to dBm.
pub fn rssi_to_dbm(raw: u8) -> i16 {
    (raw as i16) / 2 - RSSI_OFFSET_DBM
}

/// Compute the 26-bit SYNT divider for a base frequency (Eq. 4):
/// `SYNT = round(fbase · 2^18 · (B/2) · D / f_XO)`.
fn synt_value(fbase_hz: u64, b_div: u64) -> u32 {
    let num = fbase_hz * (1u64 << 18) * (b_div / 2) * REFDIV;
    ((num + F_XO / 2) / F_XO) as u32
}

/// Pack the 26-bit SYNT divider for `f_hz` into the four SYNT3..SYNT0 bytes:
/// SYNT[25:0] with WCP[7:5]=0 and the band-select `bs` in SYNT0[2:0].
fn synt_bytes(f_hz: u64, b_div: u64, bs: u8) -> [u8; 4] {
    let synt = synt_value(f_hz, b_div);
    [
        ((synt >> 21) & 0x1F) as u8, // SYNT3: WCP[7:5]=0 | SYNT[25:21]
        ((synt >> 13) & 0xFF) as u8, // SYNT2: SYNT[20:13]
        ((synt >> 5) & 0xFF) as u8,  // SYNT1: SYNT[12:5]
        (((synt & 0x1F) as u8) << 3) | (bs & 0x07), // SYNT0: SYNT[4:0] | BS
    ]
}

/// Write the band/channel-dependent synthesizer registers (SYNT3..0, CHSPACE,
/// CHNUM) — all a band switch needs to rewrite, since every other RF setting is
/// band-independent. Shared by [`apply`] and [`set_band`].
fn write_band_regs(spi: &mut Spirit1Spi, band: Band, channel: u8) -> Result<(), RadioError> {
    // Base frequency: pack SYNT[25:0] + WCP(0) + BS into SYNT3..SYNT0.
    spi.write_regs(regs::SYNT3, &synt_bytes(band.base_hz(), band.b_div(), band.bs_field()))?;
    // Channel spacing (steps of f_XO/2^15) and channel number.
    let chspace = ((band.spacing_hz() * (1u64 << 15) + F_XO / 2) / F_XO) as u8;
    spi.write_reg(regs::CHSPACE, chspace)?;
    spi.write_reg(regs::CHNUM, channel)?;
    Ok(())
}

/// Retune the radio to a different `band`/`channel` at **runtime**: bring the part
/// to READY, rewrite only the synthesizer registers, and let the VCO recalibrate
/// on the next TX/RX entry (auto-cal). All other RF settings are band-independent
/// and untouched, so this is cheap. Prefer
/// [`Net::set_band`](crate::radio::net::Net::set_band), which also moves the duty
/// policy to match the band.
pub async fn set_band(radio: &mut Spirit1, band: Band, channel: u8) -> Result<(), RadioError> {
    radio.to_ready().await?;
    write_band_regs(radio.spi(), band, channel)
}

/// Retune to an **arbitrary** carrier `f_hz` in the high VCO band (779–956 MHz) at
/// runtime: bring the part to READY, write SYNT3..0 directly, and force `CHNUM = 0`
/// so the carrier equals `f_hz` exactly (no CHSPACE offset). The VCO auto-recalibrates
/// on the next TX/RX entry (same path [`set_band`] relies on). This is the per-hop /
/// per-AFA-channel retune primitive used by the FHSS and LBT+AFA spectrum-access modes;
/// it is cheap (one 4-byte SYNT burst + one CHNUM write). `f_hz` must be in 779–956 MHz
/// (the band the SPSGRF config is tuned for); callers own the channel-plan math.
pub async fn set_freq_hz(radio: &mut Spirit1, f_hz: u64) -> Result<(), RadioError> {
    radio.to_ready().await?;
    let spi = radio.spi();
    // B=6, BS=1 (high band) — matches both Eu868/Us915 band-select.
    spi.write_regs(regs::SYNT3, &synt_bytes(f_hz, 6, 1))?;
    spi.write_reg(regs::CHNUM, 0)?;
    Ok(())
}

// --- EU 868 LBT+AFA channel plan (EN 300 220) -------------------------------
// Listen-Before-Talk + Adaptive Frequency Agility lets EU operation relax the
// 1 % duty limit in sub-bands that permit it. The whole set stays inside the
// **865.0–868.6 MHz** sub-band (≈25 mW, 1 %-or-LBT+AFA) so the power/duty rule
// never changes mid-set. (Exact min-channel count, LBT CCA time/threshold, and
// off-time per EN 300 220-1 / ERC 70-03 are config to **verify** before any
// product claim — these values are bench defaults that exercise the mechanism.)

/// Number of EU AFA channels.
pub const AFA_N: u8 = 8;
/// AFA channel-0 carrier (Hz).
pub const AFA_BASE_HZ: u64 = 865_200_000;
/// AFA channel spacing (Hz). 400 kHz > the ~60–100 kHz occupied BW (non-overlapping).
pub const AFA_SPACE_HZ: u64 = 400_000;

/// Carrier (Hz) of AFA channel `k` (0..[`AFA_N`)): 865.2 + k·0.4 MHz, ≤ 868.0 MHz.
pub const fn afa_freq_hz(k: u8) -> u64 {
    AFA_BASE_HZ + (k as u64) * AFA_SPACE_HZ
}

// All AFA channels stay within the 865.0–868.6 MHz sub-band.
const _: () = assert!(afa_freq_hz(AFA_N - 1) <= 868_600_000);

// --- US 915 FHSS channel plan (FCC §15.247) ---------------------------------
// Frequency hopping over the 902–928 MHz ISM band: ≥50 pseudo-random channels,
// ≤0.4 s occupancy per channel per 20 s. We use **80** channels (well above the
// minimum 50) so even the *strict* time-tuned occupancy reading has margin (see
// the slot math in net/fhss.rs), and hop by SYNT rewrite ([`set_freq_hz`]) so the
// set is edge-to-edge. (Exact §15.247 dwell numbers are FCC-KDB config to
// **verify** before any product claim; these exercise/meet the rule on the bench.)

/// Number of FHSS hop channels (> 50 for compliance margin; power-of-two not needed).
pub const FHSS_N: u8 = 80;
/// FHSS channel-0 carrier (Hz).
pub const FHSS_BASE_HZ: u64 = 903_000_000;
/// FHSS channel spacing (Hz). 300 kHz > the ~60–100 kHz occupied BW (non-overlapping).
pub const FHSS_SPACE_HZ: u64 = 300_000;
/// Fixed channel a scanning node parks on to rendezvous (mid-band; the gateway's
/// per-cycle permutation visits it exactly once, so lock is bounded by one cycle).
pub const FHSS_RENDEZVOUS_CH: u8 = 40;

/// Carrier (Hz) of FHSS channel `k` (0..[`FHSS_N`)): 903.0 + k·0.3 MHz.
pub const fn fhss_freq_hz(k: u8) -> u64 {
    FHSS_BASE_HZ + (k as u64) * FHSS_SPACE_HZ
}

// ≥50 channels (§15.247) and the whole set fits below the 928 MHz band edge.
const _: () = assert!(FHSS_N >= 50);
const _: () = assert!(fhss_freq_hz(FHSS_N - 1) < 928_000_000);
const _: () = assert!(FHSS_RENDEZVOUS_CH < FHSS_N);

/// Apply a full RF configuration. Must leave the part in READY. The caller has
/// already brought the radio out of shutdown and verified the device ID.
pub async fn apply(radio: &mut Spirit1, cfg: &RfConfig) -> Result<(), RadioError> {
    radio.to_ready().await?;
    {
    let spi = radio.spi();

    // Route the active-low nIRQ to GPIO0 (PA7) so tx()/rx() can wait on it.
    spi.write_reg(regs::GPIO0_CONF, regs::GPIO_CONF_IRQ)?;

    // Xtal flag: the digital clock is 25 MHz (50 MHz / 2), so select 24 MHz mode
    // (ST XTAL_FLAG: <26 MHz → 24 MHz) — clears 24_26MHZ_SELECT in ANA_FUNC_CONF0.
    // This tunes the synth loop filter + RCO reference; the reset value (26 MHz)
    // mistunes the loop filter for a 25 MHz clock. Reset 0xC0, clear bit6 → 0x80.
    spi.write_reg(regs::ANA_FUNC_CONF0, 0x80)?;

    // Demodulator order = 0 during radio init (datasheet, DEM_CONFIG).
    spi.write_reg(regs::DEM_CONFIG, 0x35)?;

    // Set the REFDIV bit (reference divider D=2) for the 50 MHz crystal: the PLL
    // reference becomes 25 MHz. SYNTH_CONFIG1 reset 0x5B (VCO_H selected) | 0x80.
    spi.write_reg(regs::SYNTH_CONFIG1, 0xDB)?;
    // Longest T-split (3.47 ns) to help the VCO calibrator (datasheet §8.5):
    // SYNTH_CONFIG0 (0x9F) reset 0x20, set SEL_TSPLIT bit7 -> 0xA0.
    spi.write_reg(regs::SYNTH_CONFIG0, 0xA0)?;

    // IF offsets for f_XO = 50 MHz (Table 31).
    spi.write_reg(regs::IF_OFFSET_ANA, 0x36)?;
    spi.write_reg(regs::IF_OFFSET_DIG, 0xAC)?;

    // Base frequency + channel (the only band-dependent registers).
    write_band_regs(spi, cfg.band, cfg.channel)?;

    // Modulation / data rate / deviation / channel filter — M/E values computed
    // with the ST library's exact search algorithms for f_XO=50 MHz, divider on:
    //   MOD1 = DATARATE_M = 147 (0x93), MOD0 = GFSK|DATARATE_E(9) = 0x19  -> 19.2 kbps
    //   FDEV0 = FDEV_E(4)<<4 | FDEV_M(5) = 0x45                           -> ~20 kHz
    //   CHFLT = 0x02 -> ~216 kHz (spec §2.1, covers ±40 ppm crystal tolerance).
    spi.write_reg(regs::MOD1, 147)?;
    spi.write_reg(regs::MOD0, 0x19)?;
    spi.write_reg(regs::FDEV0, 0x45)?;
    spi.write_reg(regs::CHFLT, 0x02)?;

    // IQC correction "optimal values" written by ST's SpiritRadioInit (undocumented
    // demodulator I/Q-correction registers): 0x99=0x80, 0x9A=0xE3, 0xBC=0x22.
    spi.write_regs(0x99, &[0x80, 0xE3])?;
    spi.write_reg(0xBC, 0x22)?;

    // VCO current bump for the 50 MHz crystal (ST WaVcoCalibration) — needed to
    // lock; the reset VCO_GEN_CURR is too low. Auto VCO calibration (PROTOCOL2.
    // VCO_CALIBRATION, on by default) then recalibrates on each TX/RX entry.
    spi.write_reg(regs::VCO_CONFIG, 0x25)?;
    // ST extra-current work-around (SpiritManagementWaExtraCurrent): standby current.
    spi.write_reg(0xB2, 0xCA)?;
    spi.write_reg(0xA8, 0x04)?;
    let _ = spi.read_reg(0xA8)?;
    spi.write_reg(0xA8, 0x00)?;

    // AFC on, freeze-on-sync. AFC2 = FREEZE_ON_SYNC|AFC_ENABLE|leakage(reset).
    spi.write_reg(regs::AFC2, 0xC8)?;

    // RSSI threshold for CCA (-90 dBm = 0x50 with the datasheet mapping). The
    // CSMA engine compares this against the channel RSSI before TX.
    spi.write_reg(regs::RSSI_TH, 0x50)?;

    // CSMA/CCA timing (the engine is gated per-TX by PROTOCOL1.CSMA_ON; the radio
    // measures RSSI for CCA_LENGTH × CCA_PERIOD, then backs off up to MAX_NB times
    // before raising MAX_BO_CCA_REACHED). Layout per ST's SpiritCsmaInit:
    //   CSMA_CONFIG3:2 = BU_COUNTER_SEED (must be non-zero; reset value is 0)
    //   CSMA_CONFIG1   = (BU_PRESCALER << 2) | CCA_PERIOD(00 = 64·Tbit ≈ 3.3 ms)
    //   CSMA_CONFIG0   = CCA_LENGTH(7:4) | MAX_NB(2:0)
    // Non-persistent (PROTOCOL1.CSMA_PERS_ON = 0) so a busy channel gives up after
    // MAX_NB back-offs instead of stalling forever.
    spi.write_reg(regs::CSMA_CONFIG3, 0xFA)?; // seed MSB
    spi.write_reg(regs::CSMA_CONFIG2, 0x21)?; // seed LSB (0xFA21, non-zero)
    spi.write_reg(regs::CSMA_CONFIG1, 32 << 2)?; // BU_PRESCALER=32, CCA_PERIOD=00 (64·Tbit)
    spi.write_reg(regs::CSMA_CONFIG0, (3 << 4) | 5)?; // CCA_LENGTH=3, NBACKOFF_MAX=5
    let (p1c, _) = spi.read_reg(regs::PROTOCOL1)?;
    spi.write_reg(regs::PROTOCOL1, p1c & !0x02)?; // CSMA_PERS_ON = 0 (non-persistent)

    // Packet: basic format, variable length (8-bit len field), 4-byte preamble,
    // 4-byte sync, 16-bit CRC (0x1021), whitening on, no address/control/FEC.
    //   PCKTCTRL4 = 0x00          : no address, no control field
    //   PCKTCTRL3 = 0x07          : basic format, normal RX, LEN_WID=7 (8-bit length)
    //   PCKTCTRL2 = PREAMBLE(3=4B)<<3 | SYNC(3=4B)<<1 | FIX_VAR_LEN(1) = 0x1F
    //   PCKTCTRL1 = CRC_MODE(3)<<5 | WHIT_EN(1)<<4 = 0x70
    spi.write_reg(regs::PCKTCTRL4, 0x00)?;
    spi.write_reg(regs::PCKTCTRL3, 0x07)?;
    spi.write_reg(regs::PCKTCTRL2, 0x1F)?;
    spi.write_reg(regs::PCKTCTRL1, 0x70)?;

    // Enable the automatic packet-filtering engine (PROTOCOL1.AUTO_PCKT_FLT,
    // bit0) and clear the reserved bit0x20 — exactly as ST's SpiritPktBasicInit.
    let (p1, _) = spi.read_reg(regs::PROTOCOL1)?;
    spi.write_reg(regs::PROTOCOL1, (p1 & !0x20) | 0x01)?;

    // PCKT_FLT_OPTIONS = 0x01: the critical RX-completion setting. Clearing
    // RX_TIMEOUT_AND_OR_SELECT (bit6) + no timeout masks selects Table 30 row 1,
    // "the RX timeout never expires and the reception ends at the reception of the
    // packet" — so RX_DATA_READY fires on a good packet. The reset value (bit6=1)
    // instead leaves the part stuck in RX with the packet in the FIFO (§9.3). bit0
    // = CRC_CHECK (drop bad-CRC frames); source/control filters (bits 4/5) cleared.
    spi.write_reg(regs::PCKT_FLT_OPTIONS, 0x01)?;

    // Sync word 0xDB624715 (SYNC4=MSB .. SYNC1=LSB).
    spi.write_regs(regs::SYNC4, &[0xDB, 0x62, 0x47, 0x15])?;

    // PA: enable ramping, max index 7 (the reset PA table is already a monotonic
    // ramp -30..+12 dBm across the 8 slots, so this ramps to ~+12 dBm).
    spi.write_reg(regs::PA_POWER0, 0x27)?;
    } // end of the SPI register block (releases the borrow)

    // VCO calibration: rely on the SPIRIT1's automatic VCO calibration
    // (PROTOCOL2.VCO_CALIBRATION, enabled by default), which recalibrates on each
    // READY→TX/RX transition. With the VCO current bump above it locks reliably,
    // and auto-cal tracks temperature drift — verified on hardware. (ST's manual
    // one-shot calibration WA, which disables auto-cal and stores fixed words, is
    // not needed here and is less temperature-robust, so it's intentionally omitted.)
    Ok(())
}
