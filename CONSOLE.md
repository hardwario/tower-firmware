# TOWER Console

We need to build a proper console system between the target (MCU) and the host (PC). We already have “jolt” program for generic STM32L0x flashing and raw text serial output monitoring.

However, we need a more sophisticated and reliable link between the two parties. The goal is to send/receive the following entities:

* Device logs (on the target side, by utilizing the standard Rust macros).
* Shell commands and responses (command is always initiated by the host).
  * For one command, there is only one response (coming at once) together with a result code (0 = success). So it will be convenient for processing.
* Event messages (unsolicited) from the target.

The log messages are always sent on the UART (regardless of the USB detection). Make sure the PA12 pin, where we are now detecting the connected USB, has a pull-down resistor always enabled.

Everything is text-based from the user’s perspective, but that is not the case on the transport level.

Transport level

As we are transmitting data over the UART, we have to handle data with caution, as “anything can happen”. I want to use Postcard for data serialization. Also, there should be a COBS frame synchronization, and at least a 16-bit CRC. We will also need to use the message type field to differentiate between the transport entities and a sequence field.

## Host side

On the host side, we will build a CLI tool “tower” that will serve multiple purposes, but now the key commands are:

* tower logs - keeps printing device logs to stdout (parameter --no-colors) disables color printing
  * Each log message has a log level, and this is colorized.
  * Each log message arrives with the timestamp and originating module name.
  * Local time from computer is prepended
* tower events - keeps printing device logs to stdout.
  * The events, opposed to slots, are more structured - like shell commands and responses, are designed to be processed programmatically.
* tower shell - opens an interactive shell.
  * The command waits for commands from the user on stdin and outputs responses from the target to stdout.

Next, we will have the command tower console, which is a TUI frontend of the CLI tool. This console opens two panes (left and right) with four major elements:

1. Left pane, about 25 % of the height: Device Events.
2. Left pane, one line: User’s text input for the Shell Command.
3. Left pane, remaining height: Shell Responses.
4. Right pane, full height: Device Logs.

These elements are in the framed group boxes with the titles:

* Device Events (allows zoom + paging)
* Shell Responses (allows zoom + paging) - the responses appear from the bottom, so they are visually close to the command input.
* Shell Command (supports up/down history + Ctrl-R search)
* Device Logs (allows zoom + paging)

The TOWER Console has a header line (with “HARDWARIO TOWER Console” title + version + device path in it) and a footer line, with the following functional keys:

* Shift-Tab: Cycles focus between (shell command input - implicit, shell responses, device events, device logs).
* F3 = Zoom (show the focused element in full screen) - works like a toggle (yellow when on). In zoomed view, the borders disappear.
* F5 = Pause (yellow when paused) - works like a toggle.
* F8 = Clear (works for Device Events, Shell Responses, Device Logs)
* F10 = Quit

Naturally, Page Up / Down works if the focused element is events / logs / shell responses.

Also, the footer displays the date and time in the bottom-right corner.

The header and footer have a gray background with black text in it.

Implement the CLI program here: /Users/pavel/hardwario/github/tower-cli

## Shell Implementation

I really like what RouterOS does with the slash commands and “/export” functionality. I want to get inspiration from its syntax. It would be very cool to even have the TAB completion functionality in the same way.

Implement commands such as:

* /system reboot
* /system/resource print

We do not have to follow the exact output as RouterOS, but giving people some configuration interface they might be already familiar with is worth trying.

I do not want to support starting from a subpath, e.g., split “/system/resource” and then “print”. It always has to start with the slash command from the root, so “/export” works like “/export terse”.

In terms of settings, we always print everything (not just deviation from factory defaults).

Also, implement these commands:
/system identity set name=tower-01

/system identity print

Allow at most 32 characters of identity. Store the identity in the KV storage.

Turn the shell and settings into a generic framework, so we won’t have much of boilerplate for each new command / settings / query.

## Host Implementation

* Use the Rust 2024 edition.
* Use the “clap” crate for CLI processing.
* Use the “ratatui” crate for TUI implementation.
* Check implementation of “jolt” as we are cooking a software from one kitchen: https://github.com/hardwario/jolt
* We also have a command “tower devices” to list available serial ports. Like what “jolt list” does.
