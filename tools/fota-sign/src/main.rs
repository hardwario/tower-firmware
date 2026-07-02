//! fota-sign — sign a firmware image into a signed FOTA manifest (docs/fota.md).
//!
//!   just fota-sign pubkey [--hex] [--key <seed>]
//!       Print the vendor public key — as a Rust array to paste into the firmware
//!       (`tower::fota::VENDOR_PUBKEY`), or as 64-char hex with `--hex` (the form
//!       tower-protocol's `TOWER_VENDOR_PUBKEY` env var takes for a production build).
//!
//!   just fota-sign sign --version <N> --in <image.bin> --out <image.fmanifest> [--hw-id <H>] [--key <seed>]
//!       Hash the image (SHA-512 truncated to 256 bits — see `image_digest`), build a
//!       Manifest {version, size, sha256, hw_id}, sign its canonical bytes with the vendor
//!       Ed25519 key, and write the 116-byte signed manifest blob. Deliver it alongside the
//!       image; the device verifies it (signature + image hash + rollback) before a swap.
//!
//! # Signing key
//!
//! `--key <file>` reads the 32-byte Ed25519 **seed** (private key) from a file — either 32 raw
//! bytes or 64 hex chars — which is how a **production** signer supplies the real vendor key
//! (keep it secret / on an HSM export). With no `--key`, this falls back to a built-in **DEV**
//! seed, but only when compiled with the default `dev-key` feature; that seed is public in this
//! file, so images it signs are NOT production-trustworthy. A `--no-default-features` build has
//! no built-in key and requires `--key`, mirroring the compile-time guard on the verify side
//! (`tower_protocol::fota::VENDOR_PUBKEY`).

use std::process::exit;

use ed25519_dalek::{Signer, SigningKey};
use sha2::{Digest, Sha512};
use tower_protocol::fota::Manifest;

/// DEV signing seed (32 bytes) — bring-up only; see the module note. The matching public
/// key (`fota-sign pubkey`) is baked into the firmware. Present only with the `dev-key` feature.
#[cfg(feature = "dev-key")]
const DEV_SEED: [u8; 32] = *b"tower-fota dev signing key v1!!!";

/// The image digest carried in `Manifest::sha256`: **SHA-512 truncated to 256 bits**.
/// Truncating a 512-bit hash to 256 is standard practice (cf. SHA-512/256) and lets the
/// device bootloader reuse salty's SHA-512 — the hash its Ed25519 verify already links —
/// instead of a second hash engine. `crates/bootloader` computes the identical value over the
/// staged image, so the two MUST agree (see the `host_and_device_image_digest_agree` test).
fn image_digest(image: &[u8]) -> [u8; 32] {
    let mut h = Sha512::new();
    h.update(image);
    let full: [u8; 64] = h.finalize().into();
    let mut out = [0u8; 32];
    out.copy_from_slice(&full[..32]);
    out
}

/// The signing key: the seed read from `--key <file>` if given, else the built-in DEV seed
/// (only if compiled with the `dev-key` feature — otherwise this is a fatal error, so a
/// production build cannot silently sign with a missing/forgeable key).
fn signing_key(key_path: Option<&str>) -> SigningKey {
    match key_path {
        Some(p) => SigningKey::from_bytes(&read_seed(p)),
        None => dev_signing_key(),
    }
}

#[cfg(feature = "dev-key")]
fn dev_signing_key() -> SigningKey {
    SigningKey::from_bytes(&DEV_SEED)
}

#[cfg(not(feature = "dev-key"))]
fn dev_signing_key() -> SigningKey {
    die("no signing key: pass --key <file> (this build has the dev key disabled)")
}

/// Read a 32-byte Ed25519 seed from `path`: accept either 32 raw bytes or 64 hex characters
/// (optionally with surrounding whitespace / a trailing newline).
fn read_seed(path: &str) -> [u8; 32] {
    let raw = std::fs::read(path).unwrap_or_else(|e| die(&format!("read key {path}: {e}")));
    if raw.len() == 32 {
        let mut seed = [0u8; 32];
        seed.copy_from_slice(&raw);
        return seed;
    }
    // Otherwise treat it as hex text (trim whitespace/newline).
    let text: String = String::from_utf8_lossy(&raw)
        .chars()
        .filter(|c| !c.is_whitespace())
        .collect();
    if text.len() != 64 {
        die(&format!(
            "key {path}: expected 32 raw bytes or 64 hex chars, got {} bytes / {} hex chars",
            raw.len(),
            text.len()
        ));
    }
    let mut seed = [0u8; 32];
    for (i, byte) in seed.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&text[2 * i..2 * i + 2], 16)
            .unwrap_or_else(|_| die(&format!("key {path}: invalid hex")));
    }
    seed
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("pubkey") => cmd_pubkey(&args[2..]),
        Some("sign") => cmd_sign(&args[2..]),
        _ => {
            eprintln!(
                "usage:\n  fota-sign pubkey [--hex] [--key <seed>]\n  fota-sign sign --version <N> --in <image.bin> --out <out.fmanifest> [--hw-id <H>] [--key <seed>]"
            );
            exit(2);
        }
    }
}

fn cmd_pubkey(args: &[String]) {
    let mut hex = false;
    let mut key_path: Option<String> = None;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--hex" => {
                hex = true;
                i += 1;
            }
            "--key" => {
                key_path = Some(args.get(i + 1).cloned().unwrap_or_else(|| die("missing value")));
                i += 2;
            }
            other => die(&format!("unknown arg: {other}")),
        }
    }
    let pk = signing_key(key_path.as_deref()).verifying_key().to_bytes();
    if hex {
        // 64-char hex — paste into TOWER_VENDOR_PUBKEY for a production tower-protocol build.
        let mut s = String::with_capacity(64);
        for b in pk {
            s.push_str(&format!("{b:02x}"));
        }
        println!("{s}");
        return;
    }
    println!("// FOTA vendor public key — generated by `fota-sign pubkey`.");
    print!("pub const VENDOR_PUBKEY: [u8; 32] = [");
    for (i, b) in pk.iter().enumerate() {
        if i % 8 == 0 {
            print!("\n   ");
        }
        print!(" 0x{b:02x},");
    }
    println!("\n];");
}

fn cmd_sign(args: &[String]) {
    let mut version: Option<u32> = None;
    let mut hw_id: u32 = 0;
    let mut in_path: Option<String> = None;
    let mut out_path: Option<String> = None;
    let mut key_path: Option<String> = None;

    let mut i = 0;
    while i < args.len() {
        let val = || args.get(i + 1).cloned().unwrap_or_else(|| die("missing value"));
        match args[i].as_str() {
            "--version" => version = Some(val().parse().unwrap_or_else(|_| die("bad --version"))),
            "--hw-id" => hw_id = val().parse().unwrap_or_else(|_| die("bad --hw-id")),
            "--in" => in_path = Some(val()),
            "--out" => out_path = Some(val()),
            "--key" => key_path = Some(val()),
            other => die(&format!("unknown arg: {other}")),
        }
        i += 2;
    }

    let version = version.unwrap_or_else(|| die("--version is required"));
    let in_path = in_path.unwrap_or_else(|| die("--in is required"));
    let out_path = out_path.unwrap_or_else(|| die("--out is required"));

    // hw_id 0 is a *wildcard* image that installs on every product (bring-up only). Warn when it
    // is used implicitly so a multi-product fleet doesn't ship a universal image by omission.
    if hw_id == 0 {
        eprintln!(
            "warning: --hw-id not set (0 = wildcard: installs on ANY product). Pass --hw-id <H> for a product-locked image."
        );
    }

    let image = std::fs::read(&in_path).unwrap_or_else(|e| die(&format!("read {in_path}: {e}")));
    let sha256 = image_digest(&image);

    let manifest = Manifest {
        flags: 0,
        hw_id,
        version,
        size: image.len() as u32,
        sha256,
    };
    let sig = signing_key(key_path.as_deref()).sign(&manifest.encode());
    let signed = manifest.encode_signed(&sig.to_bytes());
    std::fs::write(&out_path, signed).unwrap_or_else(|e| die(&format!("write {out_path}: {e}")));

    eprintln!(
        "signed v{} hw_id={} ({} B image, sha256 {:02x}{:02x}{:02x}{:02x}..) -> {} ({} B)",
        version,
        hw_id,
        image.len(),
        sha256[0],
        sha256[1],
        sha256[2],
        sha256[3],
        out_path,
        signed.len(),
    );
}

fn die(msg: &str) -> ! {
    eprintln!("error: {msg}");
    exit(1);
}

#[cfg(test)]
mod tests {
    use super::*;
    use tower_protocol::fota::{VENDOR_PUBKEY, verify_signed};

    /// The public key derived from this tool's `DEV_SEED` MUST equal the `VENDOR_PUBKEY`
    /// baked into the bootloader (the trust anchor it verifies against). The seed lives here
    /// and the baked key lives in `tower-protocol`, so they *can* drift — and if they do,
    /// every image this tool signs would be silently rejected by the bootloader. Pinning them
    /// together makes that drift a failed test, not a field-only mystery. (Regenerate the
    /// baked key with `fota-sign pubkey` whenever the seed changes.)
    #[test]
    #[cfg(feature = "dev-key")]
    fn dev_seed_matches_baked_vendor_pubkey() {
        assert_eq!(
            signing_key(None).verifying_key().to_bytes(),
            VENDOR_PUBKEY,
            "DEV_SEED-derived pubkey != tower_protocol::fota::VENDOR_PUBKEY — \
             run `fota-sign pubkey` and update the baked key"
        );
    }

    /// The whole point of the signing path: a manifest signed on the host with `ed25519-dalek`
    /// verifies on the device with `salty` (`tower_protocol::verify_signed`). Both follow
    /// RFC 8032, so the same seed yields the same key and interoperable signatures.
    #[test]
    #[cfg(feature = "dev-key")]
    fn dalek_signed_verifies_with_salty() {
        let key = signing_key(None);
        let pubkey = key.verifying_key().to_bytes();

        let image = b"a pretend firmware image of some length";
        let m = Manifest {
            flags: 0,
            hw_id: 0,
            version: 7,
            size: image.len() as u32,
            sha256: image_digest(image),
        };

        let sig = key.sign(&m.encode());
        let signed = m.encode_signed(&sig.to_bytes());

        // Device-side salty verification accepts the host's dalek signature.
        assert_eq!(verify_signed(&pubkey, &signed), Some(m));

        // And rejects a tampered image hash (flip a byte of the sha in the manifest body).
        let mut bad = signed;
        bad[20] ^= 0x01;
        assert!(verify_signed(&pubkey, &bad).is_none());
    }

    /// The signer's image digest (sha2's SHA-512, truncated) MUST equal the device's (salty's
    /// SHA-512, truncated — fed in 128-byte chunks exactly as the bootloader reads flash). Both
    /// are standard SHA-512, but the digest crosses two independent implementations, so — like
    /// the dalek↔salty signature pin above — this guards against either drifting (a mismatch
    /// would make every signed image fail the device's hash check). The image spans multiple
    /// 128-byte blocks plus a partial tail to exercise salty's block buffering.
    #[test]
    fn host_and_device_image_digest_agree() {
        let image: std::vec::Vec<u8> = (0..1000u32).map(|i| i as u8).collect();
        let host = image_digest(&image);
        let mut dev = salty::Sha512::new();
        for chunk in image.chunks(128) {
            dev.update(chunk);
        }
        let dev_full = dev.finalize();
        assert_eq!(
            host,
            dev_full[..32],
            "host sha2 vs device salty SHA-512 digest drift"
        );
    }
}
