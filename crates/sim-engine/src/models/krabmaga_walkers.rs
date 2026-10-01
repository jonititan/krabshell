//! The reference walkers, with krABMaga's `SparseA5Grid` as occupancy of record.
//!
//! Same rules and parameters as [`super::walkers`], so the two can be run
//! side by side from one pack (`simctl bench --model walkers-krabmaga`) to see
//! what the real field costs against the built-in one.
//!
//! Two properties of the upstream grid shape this file:
//!
//! - `lazy_update` *clears* the write buffer. An agent that is not placed on a
//!   tick is gone after the commit, so every step re-places every agent,
//!   including the ones that hold still.
//! - The single-threaded grid has no object-to-cell index (`get_location` scans
//!   every bag), so the model keeps each agent's committed cell itself and
//!   uses the grid for what it is good at: bags by cell.
//!
//! Intents are buffered and applied at the start of the next step, which is
//! exactly the double-buffer contract of spec §4.2: nothing an intent does is
//! visible until the tick that applies it commits.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};

use krabmaga::bevy_a5::prelude::GeoCell as A5Cell;
use krabmaga::engine::fields::field::Field;
use krabmaga::engine::fields::sparse_a5_grid::SparseA5Grid;
use sim_core::config::Param;
use sim_core::rng::Rng;
use sim_core::{
    AgentId, CellBag, GeoCell, Intent, IntentVerb, Profile, RejectReason, ScenarioBlob, WorldObject,
};

use super::walkers::{gold_draw, seed_area};
use crate::model::{Model, Truncation};

const NEIGHBOR_CACHE_CAP: usize = 1 << 20;

#[derive(Debug, Clone)]
struct Walker {
    kind: String,
    steps: u64,
    anchored: bool,
    /// Committed cell. Mirrors the grid's read side.
    cell: GeoCell,
}

/// A change waiting for the next step.
#[derive(Debug, Clone)]
enum Pending {
    Spawn {
        id: AgentId,
        cell: GeoCell,
        kind: String,
        anchored: bool,
    },
    Despawn {
        id: AgentId,
    },
    ClearCell {
        cell: GeoCell,
    },
    Move {
        id: AgentId,
        cell: GeoCell,
    },
}

pub struct KrabmagaWalkers {
    grid: SparseA5Grid<AgentId>,
    walkers: HashMap<AgentId, Walker>,
    pending: Vec<Pending>,
    neighbors: HashMap<GeoCell, Vec<GeoCell>>,
    occupied: HashSet<GeoCell>,
    /// Agents an intent placed for the coming tick; they hold that cell.
    placed: HashSet<AgentId>,
    rng: Rng,
    seed: u64,
    next_id: AgentId,
    stay_probability: f64,
    gold: bool,
}

pub fn build(blob: &ScenarioBlob) -> anyhow::Result<Box<dyn Model>> {
    let mut model = KrabmagaWalkers {
        grid: new_grid(blob),
        walkers: HashMap::new(),
        pending: Vec::new(),
        neighbors: HashMap::new(),
        occupied: HashSet::new(),
        placed: HashSet::new(),
        rng: Rng::default(),
        seed: 0,
        next_id: 1,
        stay_probability: 0.0,
        gold: false,
    };
    model.reset(blob)?;
    Ok(Box::new(model))
}

fn new_grid(blob: &ScenarioBlob) -> SparseA5Grid<AgentId> {
    let resolution = blob.config.grid.resolution;
    match blob.config.grid.root {
        Some(root) => SparseA5Grid::new_with_root(to_a5(root), resolution),
        None => SparseA5Grid::new(resolution),
    }
}

fn to_a5(cell: GeoCell) -> A5Cell {
    A5Cell::new(cell.raw())
}

fn from_a5(cell: &A5Cell) -> GeoCell {
    GeoCell::from_raw(cell.raw())
}

impl KrabmagaWalkers {
    fn accepts(&self, cell: GeoCell) -> bool {
        self.grid.contains(&to_a5(cell))
    }

    fn neighbors_of(&mut self, cell: GeoCell) -> &[GeoCell] {
        if !self.neighbors.contains_key(&cell) {
            if self.neighbors.len() >= NEIGHBOR_CACHE_CAP {
                self.neighbors.clear();
            }
            let mut options: Vec<GeoCell> = self
                .grid
                .cell_neighbors(&to_a5(cell))
                .unwrap_or_default()
                .iter()
                .map(from_a5)
                .filter(|c| self.grid.contains(&to_a5(*c)))
                .collect();
            options.sort_unstable();
            self.neighbors.insert(cell, options);
        }
        &self.neighbors[&cell]
    }

    /// Place every agent on the write side and commit. Used after anything
    /// that replaces the world wholesale.
    fn commit_all(&mut self) {
        self.occupied.clear();
        for (id, walker) in &self.walkers {
            self.grid.set_object_location(*id, &to_a5(walker.cell));
            self.occupied.insert(walker.cell);
        }
        self.grid.lazy_update();
    }

    fn apply_pending(&mut self) {
        for change in std::mem::take(&mut self.pending) {
            match change {
                Pending::Spawn {
                    id,
                    cell,
                    kind,
                    anchored,
                } => {
                    self.walkers.insert(
                        id,
                        Walker {
                            kind,
                            steps: 0,
                            anchored,
                            cell,
                        },
                    );
                    self.placed.insert(id);
                }
                Pending::Despawn { id } => {
                    self.walkers.remove(&id);
                }
                Pending::ClearCell { cell } => {
                    self.walkers.retain(|_, w| w.cell != cell);
                }
                Pending::Move { id, cell } => {
                    if let Some(walker) = self.walkers.get_mut(&id) {
                        walker.cell = cell;
                        self.placed.insert(id);
                    }
                }
            }
        }
    }

    fn next_cell(&mut self, agent: AgentId, from: GeoCell, roll: f64, pick: u64) -> GeoCell {
        if roll < self.stay_probability {
            return from;
        }
        let options = self.neighbors_of(from);
        if options.is_empty() {
            return from;
        }
        let target = options[(pick % options.len() as u64) as usize];
        if let Some(walker) = self.walkers.get_mut(&agent) {
            walker.steps += 1;
        }
        target
    }
}

impl Model for KrabmagaWalkers {
    fn kind(&self) -> &'static str {
        "walkers-krabmaga"
    }

    fn reset(&mut self, blob: &ScenarioBlob) -> anyhow::Result<()> {
        let params = &blob.config.model;
        self.grid = new_grid(blob);
        self.walkers.clear();
        self.pending.clear();
        self.neighbors.clear();
        self.placed.clear();
        self.next_id = 1;
        self.gold = blob.config.run.profile == Profile::Gold;
        self.seed = params
            .get("seed")
            .and_then(Param::as_i64)
            .map(|v| v as u64)
            .unwrap_or(0x5eed_1234_5678_9abc);
        self.rng = Rng::new(self.seed);
        self.stay_probability = params
            .get("stay_probability")
            .and_then(Param::as_f64)
            .unwrap_or(0.0)
            .clamp(0.0, 1.0);

        let resolution = blob.config.grid.resolution;
        for seed in &blob.agents {
            let cell = seed.resolve_cell(resolution)?;
            if !self.accepts(cell) {
                anyhow::bail!("seed agent at {cell} is outside the scenario's world");
            }
            let id = seed.id.unwrap_or(self.next_id);
            let anchored = seed
                .state
                .get("anchored")
                .and_then(serde_json::Value::as_bool)
                .unwrap_or(false);
            self.walkers.insert(
                id,
                Walker {
                    kind: seed.kind.clone(),
                    steps: 0,
                    anchored,
                    cell,
                },
            );
            self.next_id = self.next_id.max(id + 1);
        }

        let count = params
            .get("walkers")
            .and_then(Param::as_i64)
            .unwrap_or(0)
            .max(0) as u64;
        if count > 0 {
            let area = seed_area(blob, |c| self.accepts(c))?;
            for i in 0..count {
                let id = self.next_id;
                self.next_id += 1;
                let cell = area[(i as usize) % area.len()];
                self.walkers.insert(
                    id,
                    Walker {
                        kind: "walker".to_owned(),
                        steps: 0,
                        anchored: false,
                        cell,
                    },
                );
            }
        }

        self.commit_all();
        Ok(())
    }

    fn apply_intent(&mut self, intent: &Intent) -> Result<(), RejectReason> {
        if !self.accepts(intent.cell) {
            return Err(RejectReason::OutOfBounds);
        }
        match intent.verb {
            IntentVerb::Spawn => {
                let count = intent
                    .payload
                    .get("count")
                    .and_then(serde_json::Value::as_u64)
                    .unwrap_or(1)
                    .clamp(1, 10_000);
                let kind = intent
                    .payload
                    .get("kind")
                    .and_then(serde_json::Value::as_str)
                    .unwrap_or("walker")
                    .to_owned();
                let anchored = intent
                    .payload
                    .get("anchored")
                    .and_then(serde_json::Value::as_bool)
                    .unwrap_or(false);
                for _ in 0..count {
                    let id = self.next_id;
                    self.next_id += 1;
                    self.pending.push(Pending::Spawn {
                        id,
                        cell: intent.cell,
                        kind: kind.clone(),
                        anchored,
                    });
                }
                Ok(())
            }
            IntentVerb::Despawn => {
                match intent.agent {
                    Some(id) if self.walkers.contains_key(&id) => {
                        self.pending.push(Pending::Despawn { id })
                    }
                    Some(id) => {
                        return Err(RejectReason::Rejected {
                            message: format!("no agent {id}"),
                        })
                    }
                    None => self.pending.push(Pending::ClearCell { cell: intent.cell }),
                }
                Ok(())
            }
            IntentVerb::Move => {
                let id = intent.agent.ok_or(RejectReason::Rejected {
                    message: "move needs an agent id".to_owned(),
                })?;
                if !self.walkers.contains_key(&id) {
                    return Err(RejectReason::Rejected {
                        message: format!("no agent {id}"),
                    });
                }
                self.pending.push(Pending::Move {
                    id,
                    cell: intent.cell,
                });
                Ok(())
            }
            IntentVerb::SetField | IntentVerb::Custom(_) => Err(RejectReason::Rejected {
                message: format!("the walkers model has no handler for {:?}", intent.verb),
            }),
        }
    }

    fn step(&mut self, tick: u64) {
        self.apply_pending();

        let mut agents: Vec<AgentId> = self.walkers.keys().copied().collect();
        if self.gold {
            agents.sort_unstable();
        }

        self.occupied.clear();
        for agent in agents {
            let Some(walker) = self.walkers.get(&agent) else {
                continue;
            };
            let from = walker.cell;
            let target = if walker.anchored || self.placed.contains(&agent) {
                from
            } else {
                let (roll, pick) = if self.gold {
                    gold_draw(self.seed, agent, tick)
                } else {
                    (self.rng.unit(), self.rng.next_u64())
                };
                self.next_cell(agent, from, roll, pick)
            };
            // Every agent is written every tick: the upstream commit clears
            // the write side, so a skipped agent would vanish.
            self.grid.set_object_location(agent, &to_a5(target));
            self.occupied.insert(target);
            if let Some(walker) = self.walkers.get_mut(&agent) {
                walker.cell = target;
            }
        }

        self.placed.clear();
        self.grid.lazy_update();
    }

    fn snapshot(&self, out: &mut Vec<CellBag>, limit: usize) -> Truncation {
        out.clear();
        // Read bags straight from the committed side of the krABMaga grid.
        let collected: RefCell<HashMap<GeoCell, Vec<AgentId>>> = RefCell::new(HashMap::new());
        self.grid.iter_objects(|cell, id| {
            collected
                .borrow_mut()
                .entry(from_a5(cell))
                .or_default()
                .push(*id);
        });
        let collected = collected.into_inner();

        let mut cells: Vec<GeoCell> = collected.keys().copied().collect();
        cells.sort_unstable();

        let mut truncation = Truncation::default();
        for cell in cells {
            if out.len() >= limit {
                truncation.cells_omitted += 1;
                continue;
            }
            let mut objects: Vec<WorldObject> = collected[&cell]
                .iter()
                .filter_map(|id| {
                    let walker = self.walkers.get(id)?;
                    Some(WorldObject {
                        id: *id,
                        kind: walker.kind.clone(),
                        state: if walker.anchored {
                            serde_json::json!({ "steps": walker.steps, "anchored": true })
                        } else {
                            serde_json::json!({ "steps": walker.steps })
                        },
                    })
                })
                .collect();
            objects.sort_by_key(|o| o.id);
            objects.dedup_by_key(|o| o.id);
            out.push(CellBag { cell, objects });
            truncation.cells_written += 1;
        }
        truncation
    }

    fn agent_count(&self) -> u64 {
        self.grid.num_objects() as u64
    }

    fn occupied_cells(&self) -> u64 {
        self.occupied.len() as u64
    }

    fn fields(&self) -> serde_json::Value {
        serde_json::json!({
            "next_id": self.next_id,
            "seed": self.seed,
            "stay_probability": self.stay_probability,
            "gold": self.gold,
        })
    }

    fn restore_fields(&mut self, fields: &serde_json::Value) -> anyhow::Result<()> {
        if let Some(v) = fields.get("next_id").and_then(serde_json::Value::as_u64) {
            self.next_id = self.next_id.max(v);
        }
        if let Some(v) = fields.get("seed").and_then(serde_json::Value::as_u64) {
            self.seed = v;
        }
        if let Some(v) = fields
            .get("stay_probability")
            .and_then(serde_json::Value::as_f64)
        {
            self.stay_probability = v.clamp(0.0, 1.0);
        }
        if let Some(v) = fields.get("gold").and_then(serde_json::Value::as_bool) {
            self.gold = v;
        }
        Ok(())
    }

    fn rng_state(&self) -> u64 {
        self.rng.state()
    }

    fn set_rng_state(&mut self, state: u64) {
        self.rng = Rng::from_state(state);
    }

    fn load_cells(&mut self, cells: Vec<CellBag>) -> anyhow::Result<()> {
        self.walkers.clear();
        self.pending.clear();
        for bag in cells {
            if !self.accepts(bag.cell) {
                anyhow::bail!("checkpoint cell {} does not belong to this world", bag.cell);
            }
            for object in bag.objects {
                let get_bool = |k| object.state.get(k).and_then(serde_json::Value::as_bool);
                let walker = Walker {
                    steps: object
                        .state
                        .get("steps")
                        .and_then(serde_json::Value::as_u64)
                        .unwrap_or(0),
                    anchored: get_bool("anchored").unwrap_or(false),
                    kind: object.kind,
                    cell: bag.cell,
                };
                self.next_id = self.next_id.max(object.id + 1);
                self.walkers.insert(object.id, walker);
            }
        }
        self.commit_all();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{ActorId, Config, ScenarioManifest};

    fn blob(profile: Profile, walkers: i64) -> ScenarioBlob {
        let manifest = ScenarioManifest {
            id: "kw".into(),
            name: "kw".into(),
            description: String::new(),
            model: "walkers-krabmaga".into(),
            engine_rev: None,
            resolution: 6,
            root: None,
            profile,
        };
        let mut config = Config::default();
        manifest.apply_to(&mut config);
        config.model.insert("walkers".into(), Param::Int(walkers));
        config.model.insert("spread_k".into(), Param::Int(2));
        config
            .model
            .insert("origin_lon".into(), Param::Float(-0.1276));
        config
            .model
            .insert("origin_lat".into(), Param::Float(51.5072));
        ScenarioBlob {
            manifest,
            config,
            agents: Vec::new(),
            fields: serde_json::Value::Null,
        }
    }

    fn cell() -> GeoCell {
        GeoCell::from_lonlat(-0.1276, 51.5072, 6).unwrap()
    }

    fn total(view: &[CellBag]) -> usize {
        view.iter().map(|b| b.objects.len()).sum()
    }

    #[test]
    fn agents_are_conserved_even_though_the_upstream_commit_clears_the_write_side() {
        let mut model = build(&blob(Profile::Fast, 300)).unwrap();
        let mut view = Vec::new();
        for tick in 1..=100 {
            model.step(tick);
            assert_eq!(model.agent_count(), 300, "tick {tick}");
        }
        model.snapshot(&mut view, usize::MAX);
        assert_eq!(total(&view), 300);
        assert_eq!(view.len() as u64, model.occupied_cells());
    }

    #[test]
    fn gold_runs_hash_the_same() {
        let scenario = blob(Profile::Gold, 150);
        let (mut a, mut b) = (build(&scenario).unwrap(), build(&scenario).unwrap());
        for tick in 1..=40 {
            a.step(tick);
            b.step(tick);
        }
        let (mut va, mut vb) = (Vec::new(), Vec::new());
        a.snapshot(&mut va, usize::MAX);
        b.snapshot(&mut vb, usize::MAX);
        assert_eq!(va, vb);
    }

    #[test]
    fn gold_matches_the_built_in_model_move_for_move() {
        // Same pack, same rules, same draws: the two occupancy backends must
        // agree exactly. This is what makes the krABMaga numbers comparable.
        let mut scenario = blob(Profile::Gold, 200);
        let mut ours = build(&scenario).unwrap();
        scenario.manifest.model = "walkers".into();
        let mut builtin = crate::models::walkers::build(&scenario).unwrap();
        for tick in 1..=30 {
            ours.step(tick);
            builtin.step(tick);
        }
        let (mut a, mut b) = (Vec::new(), Vec::new());
        ours.snapshot(&mut a, usize::MAX);
        builtin.snapshot(&mut b, usize::MAX);
        assert_eq!(
            sim_core::snapshot::stable_hash(&a),
            sim_core::snapshot::stable_hash(&b)
        );
    }

    #[test]
    fn intents_mean_the_same_thing_on_both_backends() {
        let mut scenario = blob(Profile::Gold, 40);
        let mut ours = build(&scenario).unwrap();
        scenario.manifest.model = "walkers".into();
        let mut builtin = crate::models::walkers::build(&scenario).unwrap();

        let spawn = Intent::operator(cell(), IntentVerb::Spawn, serde_json::json!({"count": 5}));
        let mut moved = Intent::operator(cell(), IntentVerb::Move, serde_json::Value::Null);
        moved.agent = Some(7);

        for tick in 1..=12 {
            if tick == 3 {
                ours.apply_intent(&spawn).unwrap();
                builtin.apply_intent(&spawn).unwrap();
            }
            if tick == 6 {
                ours.apply_intent(&moved).unwrap();
                builtin.apply_intent(&moved).unwrap();
            }
            ours.step(tick);
            builtin.step(tick);
            let (mut a, mut b) = (Vec::new(), Vec::new());
            ours.snapshot(&mut a, usize::MAX);
            builtin.snapshot(&mut b, usize::MAX);
            assert_eq!(a, b, "backends diverged on tick {tick}");
            if tick == 6 {
                assert!(
                    a.iter()
                        .any(|bag| bag.cell == cell() && bag.objects.iter().any(|o| o.id == 7)),
                    "a moved agent ends the tick where it was sent"
                );
            }
        }
    }

    #[test]
    fn an_intent_is_invisible_until_the_step_that_applies_it() {
        let mut model = build(&blob(Profile::Fast, 0)).unwrap();
        let spawn = Intent {
            actor: ActorId::operator(),
            ..Intent::operator(cell(), IntentVerb::Spawn, serde_json::json!({"count": 3}))
        };
        model.apply_intent(&spawn).unwrap();
        assert_eq!(model.agent_count(), 0);
        model.step(1);
        assert_eq!(model.agent_count(), 3);

        model
            .apply_intent(&Intent::operator(
                cell(),
                IntentVerb::Despawn,
                serde_json::Value::Null,
            ))
            .unwrap();
        model.step(2);
        assert_eq!(model.agent_count(), 0);
    }

    #[test]
    fn a_snapshot_round_trips_through_load_cells() {
        let scenario = blob(Profile::Gold, 80);
        let mut model = build(&scenario).unwrap();
        for tick in 1..=5 {
            model.step(tick);
        }
        let mut before = Vec::new();
        model.snapshot(&mut before, usize::MAX);

        let mut restored = build(&scenario).unwrap();
        restored.load_cells(before.clone()).unwrap();
        let mut after = Vec::new();
        restored.snapshot(&mut after, usize::MAX);
        assert_eq!(before, after);
    }
}
