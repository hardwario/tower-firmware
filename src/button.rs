//! Debounced push-button driver with click/hold detection.
//!
//! Runs as its own task that owns the button's GPIO and produces high-level
//! [`Event`]s, consumed through a cheap, copyable [`Button`] handle:
//! `button.next_event().await`. Two constructors share the *same* debounce/
//! click/hold state machine, config, and events — they differ only in how they
//! idle:
//!
//!   * [`init_exti`] — **low power.** Owns an [`ExtiInput`]; while the button is
//!     released it sleeps on the pin's EXTI interrupt, so the MCU can enter STOP
//!     and only wakes on a press edge. It polls at `scan_interval` *only* while a
//!     press is being processed. Prefer this.
//!   * [`init_polled`] — owns a plain [`Input`] and polls at `scan_interval`
//!     continuously (no EXTI). Use it when the EXTI line is unavailable — STM32
//!     shares EXTI line *N* across all ports' pin *N*, so e.g. PA8 and PB8 both
//!     map to line 8 and cannot both be EXTI-driven; put one on `init_exti` and
//!     the other on `init_polled`. Costs periodic wake-ups (no STOP while idle).
//!
//! Events (mirrors the HARDWARIO SDK button semantics):
//!   * [`Event::Press`]   — debounced press edge.
//!   * [`Event::Release`] — debounced release edge.
//!   * [`Event::Click`]   — a press+release shorter than `click_timeout` (and not a hold).
//!   * [`Event::Hold`]    — fired once when held continuously for `hold_time`.
//!
//! Pin-agnostic: both constructors take embassy's type-erased input, and the
//! pressed level is declared with [`Polarity`].

// Reusable SDK driver surface: full event/config API exposed even if the app
// only uses a subset.

use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Input;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};

/// A debounced button event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// Debounced press edge.
    Press,
    /// Debounced release edge.
    Release,
    /// Short press: pressed and released within `click_timeout`, no hold fired.
    Click,
    /// Long press: held continuously for `hold_time` (fires once per press).
    Hold,
}

/// Button timing configuration.
#[derive(Clone, Copy)]
pub struct Config {
    /// How often the input is sampled *while a press is being processed*. In
    /// EXTI mode idle costs no polling; in polled mode this is the steady rate.
    pub scan_interval: Duration,
    /// Input must read pressed this long before a press is accepted.
    pub debounce_press: Duration,
    /// Input must read released this long before a release is accepted.
    pub debounce_release: Duration,
    /// Max press→release duration that still counts as a [`Event::Click`].
    pub click_timeout: Duration,
    /// Continuous-hold duration that triggers [`Event::Hold`].
    pub hold_time: Duration,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            scan_interval: Duration::from_millis(5),
            debounce_press: Duration::from_millis(20),
            debounce_release: Duration::from_millis(20),
            click_timeout: Duration::from_millis(500),
            hold_time: Duration::from_millis(1000),
        }
    }
}

/// Depth of the event queue between the task and the [`Button`] handle.
const QUEUE_DEPTH: usize = 8;

/// Event mailbox tying a [`Button`] handle to its task. Declare one `static`
/// per button: `static BTN: ButtonChannel = ButtonChannel::new();`.
pub struct ButtonChannel {
    inner: Channel<CriticalSectionRawMutex, Event, QUEUE_DEPTH>,
}

impl ButtonChannel {
    pub const fn new() -> Self {
        Self {
            inner: Channel::new(),
        }
    }
}

impl Default for ButtonChannel {
    fn default() -> Self {
        Self::new()
    }
}

/// Cheap, copyable handle for consuming button events.
#[derive(Clone, Copy)]
pub struct Button {
    ch: &'static ButtonChannel,
}

impl Button {
    /// Await the next debounced event. If the app falls behind, the oldest
    /// events past the queue depth are dropped.
    pub async fn next_event(&self) -> Event {
        self.ch.inner.receive().await
    }
}

/// Which logic level the line reads while the button is pressed.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Polarity {
    /// Pressed drives the line high (e.g. the Core Module's PA8: externally
    /// pulled down, a press drives it high — idle reads low, a press reads high).
    ActiveHigh,
    /// Pressed drives the line low (a button to GND with a pull-up).
    ActiveLow,
}

impl Polarity {
    const fn is_active_high(self) -> bool {
        matches!(self, Polarity::ActiveHigh)
    }
}

/// Spawn an **EXTI-gated** (low-power) button task; returns its event handle.
///
/// While released, the task sleeps on the pin's edge (the MCU can STOP); it
/// polls only while a press is in progress.
pub fn init_exti(
    spawner: Spawner,
    input: ExtiInput<'static, Async>,
    polarity: Polarity,
    config: Config,
    channel: &'static ButtonChannel,
) -> Button {
    spawner.spawn(
        scan_task(
            Source::Exti(input),
            polarity.is_active_high(),
            config,
            channel,
        )
        .unwrap(),
    );
    Button { ch: channel }
}

/// Spawn a **polled** button task; returns its event handle.
///
/// Samples the input every `scan_interval` with no EXTI — use when the pin's
/// EXTI line is already taken (see the module docs). Keeps the MCU waking while
/// idle, so prefer [`init_exti`] when the EXTI line is free.
pub fn init_polled(
    spawner: Spawner,
    input: Input<'static>,
    polarity: Polarity,
    config: Config,
    channel: &'static ButtonChannel,
) -> Button {
    spawner.spawn(
        scan_task(
            Source::Polled(input),
            polarity.is_active_high(),
            config,
            channel,
        )
        .unwrap(),
    );
    Button { ch: channel }
}

/// Input behind the one task: either EXTI-wakeable or plain polled. Lets a
/// single (non-generic) task serve both modes.
enum Source {
    Exti(ExtiInput<'static, Async>),
    Polled(Input<'static>),
}

impl Source {
    fn is_high(&self) -> bool {
        match self {
            Source::Exti(i) => i.is_high(),
            Source::Polled(i) => i.is_high(),
        }
    }

    /// Wait while the button is idle: EXTI sleeps until the pressed level (the
    /// MCU may STOP); polled just waits one scan interval.
    async fn idle_wait(&mut self, active_high: bool, scan_interval: Duration) {
        match self {
            Source::Exti(i) => {
                if active_high {
                    i.wait_for_high().await;
                } else {
                    i.wait_for_low().await;
                }
            }
            Source::Polled(_) => Timer::after(scan_interval).await,
        }
    }
}

/// Up to this many buttons (EXTI + polled combined) can run concurrently.
#[embassy_executor::task(pool_size = 2)]
async fn scan_task(
    mut input: Source,
    active_high: bool,
    config: Config,
    ch: &'static ButtonChannel,
) {
    let mut pressed = false; // debounced state
    let mut candidate_since: Option<Instant> = None; // when raw first differed
    let mut press_instant = Instant::now(); // start of the debounced press
    let mut hold_fired = false;

    loop {
        let now = Instant::now();
        let raw = input.is_high() == active_high;

        if raw != pressed {
            // Candidate transition — require the new level to hold long enough.
            let threshold = if raw {
                config.debounce_press
            } else {
                config.debounce_release
            };
            let since = *candidate_since.get_or_insert(now);
            if now.saturating_duration_since(since) >= threshold {
                pressed = raw;
                candidate_since = None;
                if pressed {
                    press_instant = now;
                    hold_fired = false;
                    let _ = ch.inner.try_send(Event::Press);
                } else {
                    let _ = ch.inner.try_send(Event::Release);
                    if !hold_fired
                        && now.saturating_duration_since(press_instant) <= config.click_timeout
                    {
                        let _ = ch.inner.try_send(Event::Click);
                    }
                }
            }
        } else {
            // Stable: drop any pending candidate, and check the hold threshold.
            candidate_since = None;
            if pressed
                && !hold_fired
                && now.saturating_duration_since(press_instant) >= config.hold_time
            {
                hold_fired = true;
                let _ = ch.inner.try_send(Event::Hold);
            }
        }

        if !pressed && !raw && candidate_since.is_none() {
            // Idle (released, stable): EXTI sleeps on the next press edge; polled
            // waits one scan interval. Either way we resume the loop on return.
            input.idle_wait(active_high, config.scan_interval).await;
        } else {
            // A press is in progress — keep polling.
            Timer::after(config.scan_interval).await;
        }
    }
}
