//! Boot-time stack painting + a runtime high-water read — a *measured* answer to "how deep does
//! the stack actually go?" on this 20 KB part, replacing the ~7.5 KB estimate the gateway budget
//! carries (docs/gateway.md) with a bench number.
//!
//! flip-link places the stack at the **bottom** of RAM (`.cargo/config.toml`), so it grows DOWN
//! from [`_stack_start`] toward the RAM origin and an overflow faults on the underflow instead of
//! silently corrupting `.bss`. That leaves a clean region `[RAM_BASE, _stack_start)` to paint.
//!
//! [`paint`] fills the free stem of that region with a sentinel word once, from the shallow boot
//! frame; as execution deepens — including ISR frames, since every task polls on the one executor
//! stack — the sentinels get overwritten. [`used`] then scans from the bottom for the first
//! surviving sentinel: the lowest address the stack ever reached. Because `used = _stack_start −
//! low_water`, the never-painted top region (live at paint time) is still counted, so there is no
//! systematic undercount. Read it over the console with `/system stack print` after driving the
//! deepest paths (the HIL gateway stack test does exactly that).

use core::ptr;

/// STM32L0 SRAM origin — the flip-link stack's low bound (it grows down to here; a further push
/// faults rather than underflowing RAM). Fixed for the L0 family; the top comes from the linker.
const RAM_BASE: usize = 0x2000_0000;

/// Sentinel word painted into the free stack. `0xAAAA_AAAA` (alternating bits) is vanishingly
/// unlikely as a genuine stack value, so a surviving one reliably marks never-touched space.
const SENTINEL: u32 = 0xAAAA_AAAA;

/// Bytes left unpainted just below the live frame at [`paint`] time, so the paint loop can never
/// clobber its own return address / locals.
const PAINT_GUARD: usize = 128;

unsafe extern "C" {
    /// Linker symbol (cortex-m-rt, flip-link-relocated): its ADDRESS is the initial SP = top of
    /// the stack region. The stack occupies `[RAM_BASE, _stack_start)` and grows down.
    static _stack_start: u8;
}

#[inline]
fn stack_top() -> usize {
    &raw const _stack_start as usize
}

/// Total stack region size in bytes (`_stack_start − RAM_BASE`).
#[must_use]
pub fn total() -> usize {
    stack_top() - RAM_BASE
}

/// Paint the free stack below the current frame with [`SENTINEL`]. Call **once**, as early as
/// possible from a shallow frame (the `app!` entry does, before `Board::take`), so the paint
/// covers everything the deeper call graph + ISR frames will later use.
///
/// Safe in practice: at the boot entry the stack is nearly empty, so `[RAM_BASE, sp − guard)` is
/// all genuinely-free RAM below the live frame (task futures live in statics, not on this stack).
pub fn paint() {
    let sp = cortex_m::register::msp::read() as usize;
    let hi = sp.saturating_sub(PAINT_GUARD);
    let mut p = RAM_BASE;
    while p + 4 <= hi {
        // SAFETY: [RAM_BASE, hi) ⊂ the free stack region below the live frame at boot; word-aligned.
        unsafe { ptr::write_volatile(p as *mut u32, SENTINEL) };
        p += 4;
    }
}

/// Lowest address the stack ever reached (its deepest point): the first word, scanning UP from
/// `RAM_BASE`, that is no longer the sentinel. Returns `RAM_BASE` if the whole paint was consumed
/// (a near-overflow — flip-link would have faulted before a true overflow).
fn low_water() -> usize {
    let top = stack_top();
    let mut p = RAM_BASE;
    while p + 4 <= top {
        // SAFETY: reading within the stack region; word-aligned and < top.
        if unsafe { ptr::read_volatile(p as *const u32) } != SENTINEL {
            break;
        }
        p += 4;
    }
    p
}

/// Peak stack **used** in bytes (`_stack_start − low_water`) — the high-water mark to weigh against
/// the 8 KB budget floor. `0` if [`paint`] was never called (the region is unpainted, so the very
/// first word differs from the sentinel only by chance — treat a suspiciously-low read as "not
/// painted").
#[must_use]
pub fn used() -> usize {
    stack_top() - low_water()
}

/// Stack still **free** in bytes at the deepest point reached (`low_water − RAM_BASE`). Near zero
/// means the paint was almost fully consumed — a near-overflow warning.
#[must_use]
pub fn free() -> usize {
    low_water() - RAM_BASE
}
