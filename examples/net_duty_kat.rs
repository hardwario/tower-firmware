//! net_duty_kat — deterministic check of the EU duty governor (RADIO.md §2.2).
//! Single board, no radio: exercises the token-bucket accounting with known
//! values and reports PASS/FAIL, so the airtime logic is verified independently
//! of timing.
//!
//!   just flash net_duty_kat

#![no_std]
#![no_main]

use log::{error, info};
use tower::radio::duty::{DutyGovernor, frame_toa_ms};
use tower::{app, board::Board};

async fn run(_b: Board) {
    let mut pass = true;

    // ToA: (4+4+1+30+2)·8·1000/19200 = 17 ms for a 30-byte FIFO frame.
    let toa = frame_toa_ms(30);
    info!(target: "duty_kat", "ToA(30B) = {} ms (expect 17)", toa);
    pass &= toa == 17;
    // Max non-bulk frame (96 B) ≈ 44 ms (§2.6).
    let toa_max = frame_toa_ms(96);
    info!(target: "duty_kat", "ToA(96B) = {} ms (expect 44)", toa_max);
    pass &= toa_max == 44;

    // Bucket cap 100 ms, 1 % refill. Consume 17 ms frames until refused.
    let mut g = DutyGovernor::new(100, 10);
    let mut allowed = 0;
    for _ in 0..10 {
        if g.try_consume(17) {
            allowed += 1;
        } else {
            break;
        }
    }
    info!(target: "duty_kat", "allowed {} x17ms from 100ms (expect 5), budget={}", allowed, g.budget_ms());
    pass &= allowed == 5 && g.budget_ms() == 15;
    pass &= !g.try_consume(17); // 15 < 17 → refused

    // Refill 1000 ms wall → +10 ms airtime (1 %); then one more 17 ms fits.
    g.refill_ms(1000);
    info!(target: "duty_kat", "after 1s refill budget={} (expect 25)", g.budget_ms());
    pass &= g.budget_ms() == 25;
    pass &= g.try_consume(17); // 25 → 8
    pass &= !g.try_consume(17); // 8 < 17

    // Huge refill is capped at the bucket size.
    g.refill_ms(1_000_000);
    pass &= g.budget_ms() == 100;

    if pass {
        info!(target: "duty_kat", "duty governor KAT: ALL PASS ***");
    } else {
        error!(target: "duty_kat", "duty governor KAT: FAIL");
    }
    loop {
        embassy_time::Timer::after_secs(5).await;
    }
}

app!(run);
