//! Put this crate's hand-written `memory.x` on the linker search path.
//!
//! Identical pattern to the embassy boot examples: copy `memory.x` into `OUT_DIR`,
//! add `OUT_DIR` to the search path so `cortex-m-rt`'s `link.x` (`INCLUDE memory.x`)
//! finds it. This crate does not enable embassy-stm32's `memory-x` feature, so this is
//! the only `memory.x` on the path — FLASH = the BOOTLOADER region.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");

    // --nmagic avoids excessive section padding on the small L0 flash (matches the
    // SDK's own build.rs). Use `-arg-bins` so it applies to this binary.
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
}
