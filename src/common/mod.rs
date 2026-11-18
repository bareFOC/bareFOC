//! # common — leaf utility modules
//!
//! Pure-logic helpers consumed by the rest of the FOC engine. Each
//! submodule is self-contained and unit-tested on the host — no
//! hardware, no timing. See `PORTING_STATUS.md` Core Library table for
//! the per-submodule status.

pub mod defaults;
pub mod filter;
pub mod pid;
pub mod time;
