//! FOTA firmware-image **manifest** — the signed metadata that gates an over-the-air
//! update (see `docs/fota.md`, "Security model").
//!
//! This lives in the shared crate for the same reason the console schema does: the
//! **host signing tool** (which builds + signs the manifest) and the **on-device
//! bootloader verifier** (which checks the signature before swapping) must agree
//! **byte-for-byte** on exactly which bytes are signed. A drift here is not a parse
//! error — it is a silently-rejected (or, worse, forged-and-accepted) update.
//!
//! Unlike the console messages, the manifest is **not** postcard: it is a fixed
//! little-endian layout. A signed artifact must be reproducible by a signing tool in
//! any language with zero serializer ambiguity, so the layout is spelled out in bytes:
//!
//! ```text
//! manifest (signed bytes, MANIFEST_LEN = 52):
//!   off  size  field
//!   0    4     magic   = b"TWFM"   (Tower FOTA Manifest)
//!   4    1     format  = FORMAT
//!   5    1     flags   (reserved feature bits; 0 today)
//!   6    2     _rsv    (zero)
//!   8    4     hw_id   (product id; an image for one product must not install on
//!                       another. 0 = "any", for bring-up only)
//!   12   4     version (monotonic; rollback protection rejects version <= installed)
//!   16   4     size    (image length in bytes)
//!   20   32    sha256  (SHA-256 of the exact image bytes)
//!
//! signed manifest (SIGNED_LEN = 116):
//!   0    52    manifest (the bytes above — exactly what is signed)
//!   52   64    sig      (Ed25519 signature over those 52 bytes)
//! ```
//!
//! The signature scheme is **Ed25519** (docs/fota.md): the signer signs the
//! 52 manifest bytes; the verifier checks the signature over the **received** bytes
//! (never a re-encoding) with the vendor public key baked into the bootloader. One
//! signature over the manifest — which carries `sha256(image)` — covers both image
//! integrity and authenticity (docs/fota.md).

/// Manifest magic, `b"TWFM"` (Tower FOTA Manifest). First 4 bytes of the manifest.
pub const MAGIC: [u8; 4] = *b"TWFM";
/// Manifest format version. Bump on any layout change so an old verifier rejects a
/// newer layout rather than misreading it.
pub const FORMAT: u8 = 1;

/// SHA-256 digest length (the image hash carried in the manifest).
pub const SHA256_LEN: usize = 32;
/// Ed25519 signature length.
pub const SIG_LEN: usize = 64;

/// Length of the encoded manifest — i.e. the exact bytes that get signed.
pub const MANIFEST_LEN: usize = 52;
/// Length of the encoded **signed** manifest: the manifest followed by its signature.
pub const SIGNED_LEN: usize = MANIFEST_LEN + SIG_LEN; // 116

/// Host-proxy sentinel offset ([`MsgType::FotaReq`](crate::MsgType::FotaReq)): a request
/// with this offset asks the host for the [`SIGNED_LEN`]-byte **signed manifest** rather
/// than image bytes (real image offsets are well below `u32::MAX`). The reply
/// ([`MsgType::FotaData`](crate::MsgType::FotaData)) echoes it.
pub const FOTA_MANIFEST_OFFSET: u32 = u32::MAX;

/// Vendor Ed25519 **public** key that gates FOTA image installs (docs/fota.md): an image
/// is only swapped in if its signed [`Manifest`] verifies against this key with
/// [`verify_signed`]. The matching private key signs releases on the host (`tools/fota-sign`).
/// Lives here (the shared crate) because the **bootloader** verifies (it owns the trust
/// anchor), and the host signer must match.
///
/// **This is the DEV key** (`fota-sign pubkey`) — anyone can sign with the published dev
/// seed, so **replace it (and the host key) before shipping**.
pub const VENDOR_PUBKEY: [u8; 32] = [
    0x88, 0xd1, 0x15, 0xc9, 0x74, 0x21, 0x96, 0xd8, //
    0x74, 0x39, 0xf8, 0xe6, 0xd8, 0x52, 0x5c, 0x0b, //
    0xf0, 0x0f, 0x76, 0x16, 0xed, 0x62, 0x9a, 0xaa, //
    0x79, 0x0b, 0x5d, 0x34, 0x89, 0x39, 0x73, 0xbf, //
];

/// Parsed firmware-image manifest (the signed metadata; see the module docs for the
/// byte layout). `Copy` and fixed-size — no allocation, no serializer.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Manifest {
    /// Reserved feature flags (0 today). Carried in the signed bytes so future flags
    /// are covered by the signature.
    pub flags: u8,
    /// Product / hardware id. An image built for one product must not install on
    /// another; `0` means "any" (bring-up only — production images set a real id).
    pub hw_id: u32,
    /// Monotonic firmware version. Rollback protection rejects an image whose
    /// `version <= installed_version`, even if it is validly signed (docs/fota.md).
    pub version: u32,
    /// Image length in bytes (the staged image must match this before swap).
    pub size: u32,
    /// SHA-256 of the exact image bytes. The bootloader recomputes the hash over the
    /// staged image and checks it against this before swapping (docs/fota.md).
    pub sha256: [u8; SHA256_LEN],
}

impl Manifest {
    /// Encode the manifest into its canonical signed bytes (`MANIFEST_LEN`). These —
    /// and only these — are what the signature covers.
    #[must_use]
    pub fn encode(&self) -> [u8; MANIFEST_LEN] {
        let mut out = [0u8; MANIFEST_LEN];
        out[0..4].copy_from_slice(&MAGIC);
        out[4] = FORMAT;
        out[5] = self.flags;
        // out[6..8] reserved (zero)
        out[8..12].copy_from_slice(&self.hw_id.to_le_bytes());
        out[12..16].copy_from_slice(&self.version.to_le_bytes());
        out[16..20].copy_from_slice(&self.size.to_le_bytes());
        out[20..52].copy_from_slice(&self.sha256);
        out
    }

    /// Decode a manifest from its signed bytes. Returns `None` unless the buffer is at
    /// least `MANIFEST_LEN`, starts with [`MAGIC`], and carries format [`FORMAT`] —
    /// so a corrupt, foreign, or future-format blob is rejected, not misread.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Manifest> {
        if buf.len() < MANIFEST_LEN || buf[0..4] != MAGIC || buf[4] != FORMAT {
            return None;
        }
        let mut sha256 = [0u8; SHA256_LEN];
        sha256.copy_from_slice(&buf[20..52]);
        Some(Manifest {
            flags: buf[5],
            hw_id: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            version: u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]),
            size: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
            sha256,
        })
    }

    /// Rollback gate (docs/fota.md): is this image strictly newer than what's installed?
    /// A validly-signed *older* image must be refused, so the version lives **inside** the
    /// signed bytes — checking it only matters once the signature has been verified.
    #[must_use]
    pub fn supersedes(&self, installed_version: u32) -> bool {
        self.version > installed_version
    }

    /// The full post-signature acceptance check: the staged image must match the
    /// manifest's `size` and `sha256` (integrity) **and** be strictly newer than
    /// `installed_version` (rollback). Call only after [`verify_signed`] has
    /// authenticated the manifest. `staged_sha` should be computed over the bytes actually
    /// in the DFU slot (re-read, so a flash write fault is caught too).
    #[must_use]
    pub fn accepts_image(&self, staged_len: u32, staged_sha: &[u8; SHA256_LEN], installed_version: u32) -> bool {
        self.size == staged_len && &self.sha256 == staged_sha && self.supersedes(installed_version)
    }

    /// Encode this manifest plus its signature into a `SIGNED_LEN` blob (the on-wire /
    /// on-disk signed artifact). The signature must have been produced over
    /// [`encode`](Self::encode)'s output.
    #[must_use]
    pub fn encode_signed(&self, sig: &[u8; SIG_LEN]) -> [u8; SIGNED_LEN] {
        let mut out = [0u8; SIGNED_LEN];
        out[..MANIFEST_LEN].copy_from_slice(&self.encode());
        out[MANIFEST_LEN..].copy_from_slice(sig);
        out
    }
}

/// Split a signed-manifest blob into `(manifest_bytes, signature)` for verification.
///
/// `manifest_bytes` is the first [`MANIFEST_LEN`] bytes **as received** — verify the
/// Ed25519 signature over exactly these (never a re-encoding), then [`Manifest::decode`]
/// them. Returns `None` if the blob is too short or the manifest header is invalid.
#[must_use]
pub fn split_signed(buf: &[u8]) -> Option<(&[u8], [u8; SIG_LEN])> {
    if buf.len() < SIGNED_LEN {
        return None;
    }
    let manifest_bytes = &buf[..MANIFEST_LEN];
    // Reject early if the header is wrong (cheap; avoids verifying junk).
    if manifest_bytes[0..4] != MAGIC || manifest_bytes[4] != FORMAT {
        return None;
    }
    let mut sig = [0u8; SIG_LEN];
    sig.copy_from_slice(&buf[MANIFEST_LEN..SIGNED_LEN]);
    Some((manifest_bytes, sig))
}

/// Verify a signed-manifest blob against the vendor public key and return the parsed
/// [`Manifest`] only if the Ed25519 signature is valid (docs/fota.md).
///
/// This is the **authenticity gate**: the device must not arm a swap unless the manifest
/// — which carries `sha256(image)` and `version` — is signed by the vendor key. CCM on
/// the wire is *not* sufficient (the link key is recoverable by a sniffer); only this
/// signature is. After this returns `Some(m)`, the caller still must check that the staged
/// image hashes to `m.sha256` and that `m.supersedes(installed_version)` (rollback).
///
/// The 52 manifest bytes are signed directly (small enough — no pre-hash); Ed25519 hashes
/// them with SHA-512 internally. Interops with a host signer (e.g. `ed25519-dalek`) that
/// signs the same `Manifest::encode()` bytes. Available with the `verify` feature.
#[cfg(feature = "verify")]
#[must_use]
pub fn verify_signed(vendor_pubkey: &[u8; 32], signed: &[u8]) -> Option<Manifest> {
    let (manifest_bytes, sig) = split_signed(signed)?;
    let public_key = salty::PublicKey::try_from(vendor_pubkey).ok()?;
    let signature = salty::Signature::from(&sig);
    public_key.verify(manifest_bytes, &signature).ok()?;
    Manifest::decode(manifest_bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Manifest {
        let mut sha256 = [0u8; SHA256_LEN];
        for (i, b) in sha256.iter_mut().enumerate() {
            *b = i as u8;
        }
        Manifest {
            flags: 0,
            hw_id: 0xDEAD_BEEF,
            version: 42,
            size: 65_536,
            sha256,
        }
    }

    #[test]
    fn roundtrip() {
        let m = sample();
        let bytes = m.encode();
        assert_eq!(bytes.len(), MANIFEST_LEN);
        assert_eq!(&bytes[0..4], &MAGIC);
        assert_eq!(bytes[4], FORMAT);
        assert_eq!(Manifest::decode(&bytes), Some(m));
    }

    #[test]
    fn fields_land_at_documented_offsets() {
        let m = sample();
        let b = m.encode();
        assert_eq!(b[5], 0); // flags
        assert_eq!(&b[8..12], &0xDEAD_BEEFu32.to_le_bytes());
        assert_eq!(&b[12..16], &42u32.to_le_bytes());
        assert_eq!(&b[16..20], &65_536u32.to_le_bytes());
        assert_eq!(&b[20..52], &m.sha256);
    }

    #[test]
    fn rejects_bad_magic_format_and_length() {
        let mut b = sample().encode();
        assert!(Manifest::decode(&b[..MANIFEST_LEN - 1]).is_none()); // too short
        b[0] = b'X';
        assert!(Manifest::decode(&b).is_none()); // bad magic
        let mut b = sample().encode();
        b[4] = FORMAT + 1;
        assert!(Manifest::decode(&b).is_none()); // future format
    }

    #[test]
    fn signed_roundtrip_and_split() {
        let m = sample();
        let mut sig = [0u8; SIG_LEN];
        for (i, s) in sig.iter_mut().enumerate() {
            *s = 0x80 ^ i as u8;
        }
        let blob = m.encode_signed(&sig);
        assert_eq!(blob.len(), SIGNED_LEN);

        let (manifest_bytes, got_sig) = split_signed(&blob).unwrap();
        assert_eq!(manifest_bytes, &m.encode()[..]); // verify over exactly these bytes
        assert_eq!(got_sig, sig);
        assert_eq!(Manifest::decode(manifest_bytes), Some(m));
    }

    #[test]
    fn split_rejects_short_and_malformed() {
        let m = sample();
        let sig = [0u8; SIG_LEN];
        let blob = m.encode_signed(&sig);
        assert!(split_signed(&blob[..SIGNED_LEN - 1]).is_none()); // too short
        let mut bad = blob;
        bad[4] = FORMAT + 1; // future format in the manifest part
        assert!(split_signed(&bad).is_none());
    }

    #[test]
    fn supersedes_is_strict_newer() {
        let mut m = sample();
        m.version = 5;
        assert!(m.supersedes(4));
        assert!(!m.supersedes(5)); // equal → reject (rollback)
        assert!(!m.supersedes(6));
    }

    #[test]
    fn accepts_image_checks_size_hash_and_version() {
        let m = sample(); // version 42, size 65_536, sha = 0,1,2,..,31
        assert!(m.accepts_image(65_536, &m.sha256, 41));
        assert!(!m.accepts_image(65_535, &m.sha256, 41)); // wrong size
        let mut wrong = m.sha256;
        wrong[0] ^= 0x01;
        assert!(!m.accepts_image(65_536, &wrong, 41)); // wrong hash
        assert!(!m.accepts_image(65_536, &m.sha256, 42)); // not newer (rollback)
    }

    #[cfg(feature = "verify")]
    #[test]
    fn verify_accepts_valid_and_rejects_tampered() {
        let keypair = salty::Keypair::from(&[7u8; 32]);
        let pubkey = keypair.public.to_bytes();
        let m = sample();
        let sig = keypair.sign(&m.encode());
        let signed = m.encode_signed(&sig.to_bytes());

        // A correctly-signed manifest verifies and round-trips.
        assert_eq!(verify_signed(&pubkey, &signed), Some(m));

        // Flip a byte of the signed manifest body → signature no longer matches.
        let mut bad_body = signed;
        bad_body[12] ^= 0x01; // a version byte
        assert!(verify_signed(&pubkey, &bad_body).is_none());

        // Flip a byte of the signature → rejected.
        let mut bad_sig = signed;
        bad_sig[MANIFEST_LEN] ^= 0x01;
        assert!(verify_signed(&pubkey, &bad_sig).is_none());

        // A different (wrong) vendor key → rejected.
        let other = salty::Keypair::from(&[8u8; 32]).public.to_bytes();
        assert!(verify_signed(&other, &signed).is_none());
    }

    /// The baked [`VENDOR_PUBKEY`] is a valid Ed25519 public key — the bootloader's trust
    /// anchor (and must match the host signer `fota-sign pubkey`).
    #[cfg(feature = "verify")]
    #[test]
    fn vendor_pubkey_is_valid() {
        assert!(salty::PublicKey::try_from(&VENDOR_PUBKEY).is_ok());
    }
}
