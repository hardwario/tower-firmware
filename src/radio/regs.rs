//! SPIRIT1 register map, command codes, state codes and IRQ bit positions.
//!
//! Pure constants transcribed from the SPIRIT1 datasheet (DS022758 Rev 11):
//! §10.2 SPI interface, Table 20 (states), Table 21 (commands), Table 36
//! (interrupts) and §11 (register table). No I/O lives here.
//!
//! **SPI framing** (§10.2). Every transaction starts with a *header byte* whose
//! MSB is A/C (0 = address access, 1 = command) and whose LSB is W/R (1 = read).
//! All other header bits are 0. So the three header bytes are [`WRITE`], [`READ`]
//! and [`COMMAND`]. The address auto-increments in burst mode; the FIFO is a
//! single linear address [`FIFO`] (0xFF) — read for RX, write for TX. The two
//! MC_STATE status bytes are returned on MISO during the header + address bytes
//! of *every* transaction (see [`state_from_status`]).

// --- SPI header bytes (§10.2) ---
/// Header: write to register/FIFO (A/C=0, W/R=0).
pub const WRITE: u8 = 0x00;
/// Header: read from register/FIFO (A/C=0, W/R=1).
pub const READ: u8 = 0x01;
/// Header: command strobe (A/C=1, W/R=0); the command code follows.
pub const COMMAND: u8 = 0x80;
/// Linear FIFO pseudo-address (read = RX FIFO, write = TX FIFO).
pub const FIFO: u8 = 0xFF;

// --- Command codes (Table 21); sent as the byte after the COMMAND header ---
pub const CMD_TX: u8 = 0x60;
pub const CMD_RX: u8 = 0x61;
pub const CMD_READY: u8 = 0x62;
pub const CMD_STANDBY: u8 = 0x63;
pub const CMD_SLEEP: u8 = 0x64;
pub const CMD_LOCKRX: u8 = 0x65;
pub const CMD_LOCKTX: u8 = 0x66;
pub const CMD_SABORT: u8 = 0x67;
pub const CMD_LDC_RELOAD: u8 = 0x68;
pub const CMD_SEQUENCE_UPDATE: u8 = 0x69;
pub const CMD_AES_ENC: u8 = 0x6A;
pub const CMD_AES_KEY: u8 = 0x6B;
pub const CMD_AES_DEC: u8 = 0x6C;
pub const CMD_AES_KEYDEC: u8 = 0x6D;
pub const CMD_SRES: u8 = 0x70;
pub const CMD_FLUSHRXFIFO: u8 = 0x71;
pub const CMD_FLUSHTXFIFO: u8 = 0x72;

// --- MC_STATE STATE[6:0] codes (Table 20) ---
pub const STATE_STANDBY: u8 = 0x40;
pub const STATE_SLEEP: u8 = 0x36;
pub const STATE_READY: u8 = 0x03;
pub const STATE_LOCK: u8 = 0x0F;
pub const STATE_RX: u8 = 0x33;
pub const STATE_TX: u8 = 0x5F;
// Transient states (useful when polling/debugging).
pub const STATE_PM_SETUP: u8 = 0x3D;
pub const STATE_XO_SETTLING: u8 = 0x23;
pub const STATE_SYNTH_SETUP: u8 = 0x53;
pub const STATE_PROTOCOL: u8 = 0x1F;
pub const STATE_SYNTH_CALIBRATION: u8 = 0x4F;
pub const STATE_LOCKWON: u8 = 0x13;

/// Extract the 7-bit MC state from the first MISO status byte returned by any
/// SPI transaction. That byte is the MC_STATE0 register: `bits[7:1]=STATE[6:0]`,
/// bit0=XO_ON. So `state = (status0 >> 1) & 0x7F`.
#[inline]
pub const fn state_from_status(status0: u8) -> u8 {
    (status0 >> 1) & 0x7F
}

/// XO_ON bit of the first status byte (MC_STATE0 bit0).
#[inline]
pub const fn xo_on(status0: u8) -> bool {
    status0 & 0x01 != 0
}

// --- General configuration registers (Table 41) ---
pub const ANA_FUNC_CONF1: u8 = 0x00;
pub const ANA_FUNC_CONF0: u8 = 0x01;
pub const GPIO3_CONF: u8 = 0x02;
pub const GPIO2_CONF: u8 = 0x03;
pub const GPIO1_CONF: u8 = 0x04;
pub const GPIO0_CONF: u8 = 0x05;
pub const MCU_CK_CONF: u8 = 0x06;
pub const IF_OFFSET_ANA: u8 = 0x07;
pub const SYNTH_CONFIG1: u8 = 0x9E;
pub const SYNTH_CONFIG0: u8 = 0x9F;

// --- Radio configuration: analog blocks (Table 42) ---
pub const SYNT3: u8 = 0x08;
pub const SYNT2: u8 = 0x09;
pub const SYNT1: u8 = 0x0A;
pub const SYNT0: u8 = 0x0B;
pub const CHSPACE: u8 = 0x0C;
pub const IF_OFFSET_DIG: u8 = 0x0D;
pub const FC_OFFSET1: u8 = 0x0E;
pub const FC_OFFSET0: u8 = 0x0F;
/// PA power table: PA_POWER8 (0x10, highest slot) .. PA_POWER1 (0x17), then the
/// control register PA_POWER0 (0x18: CWC, ramp enable, step width, max index).
pub const PA_POWER8: u8 = 0x10;
pub const PA_POWER1: u8 = 0x17;
pub const PA_POWER0: u8 = 0x18;

// --- Radio configuration: digital blocks (Table 43) ---
pub const MOD1: u8 = 0x1A;
pub const MOD0: u8 = 0x1B;
pub const FDEV0: u8 = 0x1C;
pub const CHFLT: u8 = 0x1D;
pub const AFC2: u8 = 0x1E;
pub const AFC1: u8 = 0x1F;
pub const AFC0: u8 = 0x20;
pub const RSSI_FLT: u8 = 0x21;
pub const RSSI_TH: u8 = 0x22;
pub const CLOCKREC: u8 = 0x23;
pub const AGCCTRL2: u8 = 0x24;
pub const AGCCTRL1: u8 = 0x25;
pub const AGCCTRL0: u8 = 0x26;
pub const ANT_SELECT_CONF: u8 = 0x27;

// --- Packet / protocol configuration (Table 44) ---
pub const PCKTCTRL4: u8 = 0x30;
pub const PCKTCTRL3: u8 = 0x31;
pub const PCKTCTRL2: u8 = 0x32;
pub const PCKTCTRL1: u8 = 0x33;
pub const PCKTLEN1: u8 = 0x34;
pub const PCKTLEN0: u8 = 0x35;
pub const SYNC4: u8 = 0x36;
pub const SYNC3: u8 = 0x37;
pub const SYNC2: u8 = 0x38;
pub const SYNC1: u8 = 0x39;
pub const QI: u8 = 0x3A;
pub const FIFO_CONFIG3: u8 = 0x3E;
pub const FIFO_CONFIG2: u8 = 0x3F;
pub const FIFO_CONFIG1: u8 = 0x40;
pub const FIFO_CONFIG0: u8 = 0x41;
pub const PCKT_FLT_OPTIONS: u8 = 0x4F;
pub const PROTOCOL2: u8 = 0x50;
pub const PROTOCOL1: u8 = 0x51;
pub const PROTOCOL0: u8 = 0x52;
pub const TIMERS5_RX_TIMEOUT_PRESCALER: u8 = 0x53;
pub const TIMERS4_RX_TIMEOUT_COUNTER: u8 = 0x54;
pub const TIMERS3_LDC_PRESCALER: u8 = 0x55;
pub const TIMERS2_LDC_COUNTER: u8 = 0x56;
pub const CSMA_CONFIG3: u8 = 0x64;
pub const CSMA_CONFIG2: u8 = 0x65;
pub const CSMA_CONFIG1: u8 = 0x66;
pub const CSMA_CONFIG0: u8 = 0x67;

// --- Frequently used registers (Table 45) ---
pub const CHNUM: u8 = 0x6C;
pub const RCO_VCO_CALIBR_IN2: u8 = 0x6D;
pub const RCO_VCO_CALIBR_IN1: u8 = 0x6E; // VCO_CALIBR_TX[6:0]
pub const RCO_VCO_CALIBR_IN0: u8 = 0x6F; // VCO_CALIBR_RX[6:0]
pub const RCO_VCO_CALIBR_OUT0: u8 = 0xE5; // VCO_CALIBR_DATA[6:0]
pub const IRQ_MASK3: u8 = 0x90;
pub const IRQ_MASK2: u8 = 0x91;
pub const IRQ_MASK1: u8 = 0x92;
pub const IRQ_MASK0: u8 = 0x93;
pub const VCO_CONFIG: u8 = 0xA1;
pub const DEM_CONFIG: u8 = 0xA3;
pub const PM_CONFIG2: u8 = 0xA4;
pub const PM_CONFIG1: u8 = 0xA5;
pub const PM_CONFIG0: u8 = 0xA6;
pub const XO_RCO_CONFIG: u8 = 0xA7;
pub const MC_STATE1: u8 = 0xC0;
pub const MC_STATE0: u8 = 0xC1;
pub const TX_PCKT_INFO: u8 = 0xC2;
pub const RX_PCKT_INFO: u8 = 0xC3;
pub const AFC_CORR: u8 = 0xC4;
pub const LINK_QUALIF2_PQI: u8 = 0xC5;
pub const LINK_QUALIF1_SQI: u8 = 0xC6;
pub const LINK_QUALIF0_AGC: u8 = 0xC7;
pub const RSSI_LEVEL: u8 = 0xC8;
pub const RX_PCKT_LEN1: u8 = 0xC9;
pub const RX_PCKT_LEN0: u8 = 0xCA;
pub const LINEAR_FIFO_STATUS1_TX: u8 = 0xE6;
pub const LINEAR_FIFO_STATUS0_RX: u8 = 0xE7;
pub const IRQ_STATUS3: u8 = 0xFA;
pub const IRQ_STATUS2: u8 = 0xFB;
pub const IRQ_STATUS1: u8 = 0xFC;
pub const IRQ_STATUS0: u8 = 0xFD;

// --- Device information (Table 46) ---
/// PARTNUM register (0xF0). Combined with VERSION it reads 0x0130 = 304.
pub const DEVICE_INFO1_PARTNUM: u8 = 0xF0;
/// VERSION register (0xF1) = 0x30 = 48.
pub const DEVICE_INFO0_VERSION: u8 = 0xF1;
/// Expected PARTNUM byte (0x01) — `(PARTNUM<<8)|VERSION == 304`.
pub const EXPECT_PARTNUM: u8 = 0x01;
/// Expected VERSION byte (0x30 = 48).
pub const EXPECT_VERSION: u8 = 0x30;

// --- IRQ_STATUS / IRQ_MASK bit positions in the 32-bit INT_EVENT word (Table 36) ---
pub const IRQ_RX_DATA_READY: u32 = 1 << 0;
pub const IRQ_RX_DATA_DISC: u32 = 1 << 1;
pub const IRQ_TX_DATA_SENT: u32 = 1 << 2;
pub const IRQ_MAX_RE_TX_REACH: u32 = 1 << 3;
pub const IRQ_CRC_ERROR: u32 = 1 << 4;
pub const IRQ_TX_FIFO_ERROR: u32 = 1 << 5;
pub const IRQ_RX_FIFO_ERROR: u32 = 1 << 6;
pub const IRQ_TX_FIFO_ALMOST_FULL: u32 = 1 << 7;
pub const IRQ_TX_FIFO_ALMOST_EMPTY: u32 = 1 << 8;
pub const IRQ_RX_FIFO_ALMOST_FULL: u32 = 1 << 9;
pub const IRQ_RX_FIFO_ALMOST_EMPTY: u32 = 1 << 10;
pub const IRQ_MAX_BO_CCA_REACH: u32 = 1 << 11;
pub const IRQ_VALID_PREAMBLE: u32 = 1 << 12;
pub const IRQ_VALID_SYNC: u32 = 1 << 13;
pub const IRQ_RSSI_ABOVE_TH: u32 = 1 << 14;
pub const IRQ_WKUP_TOUT_LDC: u32 = 1 << 15;
pub const IRQ_READY: u32 = 1 << 16;
pub const IRQ_STANDBY_DELAYED: u32 = 1 << 17;
pub const IRQ_LOW_BATT_LVL: u32 = 1 << 18;
pub const IRQ_POR: u32 = 1 << 19;
pub const IRQ_BOR: u32 = 1 << 20;
pub const IRQ_LOCK: u32 = 1 << 21;
pub const IRQ_RX_TIMEOUT: u32 = 1 << 29;
pub const IRQ_AES_END: u32 = 1 << 30;

// --- GPIO0_CONF value to route nIRQ (active-low) to GPIO0 ---
// GPIO_SELECT[4:0]=0 (digital-output signal 0 = nIRQ), GPIO_MODE[1:0]=10
// (digital output, low power). Bit layout: [7:3]=select, [2]=reserved, [1:0]=mode.
pub const GPIO_CONF_IRQ: u8 = 0b0000_0010;
