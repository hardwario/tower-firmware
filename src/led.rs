//! Non-blocking LED blink dispatcher.
//!
//! The LED runs in its own task that owns the pin and plays patterns over time,
//! so the application never blocks on `Timer`s just to flash a LED. The app
//! drives it through a cheap, copyable [`Led`] handle whose methods are
//! fire-and-forget (they never block or await).
//!
//! Two layers, with a clear priority:
//!   * a **background** pattern that loops forever (a status/heartbeat
//!     indication), and
//!   * **instant** sequences that take priority — each preempts the background,
//!     plays once, and then the background resumes.
//!
//! When an instant sequence interrupts a *running* background, the LED is first
//! held off for a short, API-settable gap ([`Led::set_switch_delay`]) so the
//! instant indication reads as distinct from the background blinking rather than
//! blending into it.
//!
//! Pin-agnostic: pass any GPIO via embassy's type-erased [`Output`] (so it is
//! not tied to the board LED), and declare the polarity with [`Polarity`].
//!
//! ```ignore
//! static STATUS: led::LedChannel = led::LedChannel::new();
//! static HEARTBEAT: led::Pattern = &[led::Step::on(30), led::Step::off(2970)];
//! static OK: led::Pattern = &[led::Step::on(40), led::Step::off(60), led::Step::on(40)];
//!
//! let led = led::init(spawner, Output::new(p.PH1, Level::Low, Speed::Low), &STATUS, true);
//! led.set_background(Some(HEARTBEAT));
//! led.play(OK); // preempts the heartbeat after the switch gap, then it resumes
//! ```

// Reusable SDK driver surface: full API exposed even if the app uses a subset
// (e.g. `Polarity::ActiveLow` for off-board LEDs).

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

/// One step of a blink pattern: hold the LED on/off for `ms` milliseconds.
#[derive(Clone, Copy)]
pub struct Step {
    pub on: bool,
    pub ms: u32,
}

impl Step {
    /// LED on for `ms` milliseconds.
    pub const fn on(ms: u32) -> Self {
        Self { on: true, ms }
    }
    /// LED off for `ms` milliseconds.
    pub const fn off(ms: u32) -> Self {
        Self { on: false, ms }
    }
}

/// A blink pattern: a `'static` slice of [`Step`]s. As a background it loops; as
/// an instant sequence it plays once. Define patterns as `static` items.
pub type Pattern = &'static [Step];

/// Default background→instant switch gap; override with [`Led::set_switch_delay`].
const DEFAULT_SWITCH_DELAY: Duration = Duration::from_millis(250);

/// Depth of the command queue between an [`Led`] handle and its task.
const QUEUE_DEPTH: usize = 8;

enum Command {
    /// Set (or clear, with `None`) the looping background pattern.
    Background(Option<Pattern>),
    /// Play a one-shot sequence at priority over the background.
    Play(Pattern),
    /// Change the background→instant switch gap.
    SwitchDelay(Duration),
}

/// Command mailbox tying an [`Led`] handle to its dispatcher task. Declare one
/// `static` per LED: `static STATUS: LedChannel = LedChannel::new();`.
pub struct LedChannel {
    inner: Channel<CriticalSectionRawMutex, Command, QUEUE_DEPTH>,
}

impl LedChannel {
    pub const fn new() -> Self {
        Self {
            inner: Channel::new(),
        }
    }
}

impl Default for LedChannel {
    fn default() -> Self {
        Self::new()
    }
}

/// Cheap, copyable handle the application uses to drive a LED. Every method is
/// best-effort fire-and-forget: it enqueues a command and returns immediately,
/// dropping the command only if the queue is somehow full.
#[derive(Clone, Copy)]
pub struct Led {
    ch: &'static LedChannel,
}

impl Led {
    /// Play `seq` once, at priority over the background pattern.
    pub fn play(&self, seq: Pattern) {
        let _ = self.ch.inner.try_send(Command::Play(seq));
    }

    /// Set the looping background pattern, or `None` to turn the background off.
    pub fn set_background(&self, pattern: Option<Pattern>) {
        let _ = self.ch.inner.try_send(Command::Background(pattern));
    }

    /// Set the off-gap inserted when an instant sequence preempts a running
    /// background (visual separation). Default is 250 ms.
    pub fn set_switch_delay(&self, delay: Duration) {
        let _ = self.ch.inner.try_send(Command::SwitchDelay(delay));
    }
}

/// Which logic level lights the LED.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Polarity {
    /// Driving the pin high lights the LED (e.g. the Core Module's PH1).
    ActiveHigh,
    /// Driving the pin low lights the LED.
    ActiveLow,
}

impl Polarity {
    const fn is_active_high(self) -> bool {
        matches!(self, Polarity::ActiveHigh)
    }
}

/// Spawn the dispatcher for `pin` and return a handle to drive it.
pub fn init(
    spawner: Spawner,
    pin: Output<'static>,
    channel: &'static LedChannel,
    polarity: Polarity,
) -> Led {
    // The size-bounded task pool is allocated here; see `dispatcher`'s pool_size.
    spawner.spawn(dispatcher(pin, channel, polarity.is_active_high()).unwrap());
    Led { ch: channel }
}

/// Up to this many LEDs can be driven concurrently (one task per LED).
#[embassy_executor::task(pool_size = 4)]
async fn dispatcher(mut pin: Output<'static>, ch: &'static LedChannel, active_high: bool) {
    let mut background: Option<Pattern> = None;
    let mut switch_delay = DEFAULT_SWITCH_DELAY;
    apply(&mut pin, false, active_high);

    loop {
        // Run the background (preemptible by any command) or idle until one.
        let cmd = match background {
            Some(pattern) => run_background(&mut pin, pattern, active_high, ch).await,
            None => {
                apply(&mut pin, false, active_high);
                ch.inner.receive().await
            }
        };

        match cmd {
            // Ignore an empty pattern so the background loop always makes progress.
            Command::Background(p) => background = p.filter(|pat| !pat.is_empty()),
            Command::SwitchDelay(d) => switch_delay = d,
            Command::Play(seq) => {
                // Separate the instant sequence from a *running* background only.
                if background.is_some() {
                    apply(&mut pin, false, active_high);
                    Timer::after(switch_delay).await;
                }
                play(&mut pin, seq, active_high).await;
                apply(&mut pin, false, active_high);
            }
        }
    }
}

/// Drive the pin to the requested logical level, honoring polarity.
fn apply(pin: &mut Output<'static>, on: bool, active_high: bool) {
    if on == active_high {
        pin.set_high();
    } else {
        pin.set_low();
    }
}

/// Play a sequence once, start to finish.
async fn play(pin: &mut Output<'static>, seq: Pattern, active_high: bool) {
    for step in seq {
        apply(pin, step.on, active_high);
        Timer::after_millis(step.ms as u64).await;
    }
}

/// Loop `pattern` (assumed non-empty) until a command arrives; return it. The
/// command preempts the current step immediately rather than waiting it out.
async fn run_background(
    pin: &mut Output<'static>,
    pattern: Pattern,
    active_high: bool,
    ch: &'static LedChannel,
) -> Command {
    loop {
        for step in pattern {
            apply(pin, step.on, active_high);
            match select(Timer::after_millis(step.ms as u64), ch.inner.receive()).await {
                Either::First(()) => {}
                Either::Second(cmd) => return cmd,
            }
        }
    }
}
