//! Models shipped with the server.
//!
//! A model is registered by name in [`crate::model::ModelRegistry`] and named by
//! a scenario manifest's `model` key.

pub mod drones;
pub mod walkers;

/// The same model on krABMaga's `SparseA5Grid`, registered as `walkers-krabmaga`.
#[cfg(feature = "krabmaga")]
pub mod krabmaga_walkers;
