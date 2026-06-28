//! crypto_aes_kat — validate the L0 hardware AES against the FIPS-197 vector.
//!
//! Single board, no radio. Encrypts the canonical AES-128 ECB known-answer test
//! (FIPS-197 App. B) and reports MATCH/MISMATCH — proving the register driver's
//! key load, byte ordering and block compute before AES-CCM is built on it.
//!
//!   just flash crypto_aes_kat    (watch with: tower logs)

#![no_std]
#![no_main]

use log::{error, info};
use tower::radio::aes::Aes;
use tower::{app, board::Board};

// FIPS-197 Appendix B / C.1, AES-128:
const KEY: [u8; 16] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
];
const PLAIN: [u8; 16] = [
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
];
const EXPECT: [u8; 16] = [
    0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a,
];

async fn run(_b: Board) {
    let mut aes = Aes::new();
    aes.set_key(&KEY);

    let mut block = PLAIN;
    aes.encrypt_block(&mut block);

    info!(target: "aes_kat", "expect: {}", Hex(&EXPECT));
    info!(target: "aes_kat", "got:    {}", Hex(&block));
    if block == EXPECT {
        info!(target: "aes_kat", "AES-128 ECB FIPS-197 vector: MATCH ***");
    } else {
        error!(target: "aes_kat", "AES-128 ECB FIPS-197 vector: MISMATCH");
    }

    loop {
        embassy_time::Timer::after_secs(5).await;
    }
}

/// Tiny hex formatter for a byte slice.
struct Hex<'a>(&'a [u8]);
impl core::fmt::Display for Hex<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for b in self.0 {
            write!(f, "{:02x}", b)?;
        }
        Ok(())
    }
}

app!(run);
