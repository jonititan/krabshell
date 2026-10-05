//! The reference model: agents that random-walk across A5 cells.
//!
//! It exists to exercise every part of the server — occupancy, intents,
//! snapshots, checkpoints, both determinism profiles — with rules simple enough
//! that a wrong answer is obvious. Phase 1's speed baseline is measured on it.

use std::collections::{BTreeMap, HashMap, HashSet};

use sim_core::config::Param;
pub(crate) use sim_core::rng::gold_draw;
use sim_core::rng::Rng;
use sim_core::{
    AgentId, CellBag, GeoCell, Intent, IntentVerb, Profile, RejectReason, ScenarioBlob, WorldObject,
};

use crate::grid::SparseCellGrid;
use crate::model::{Model, Truncation};

/// Neighbour lookups are pure maths on the cell id, so they can be cached for
/// the life of a run. The cap stops an unbounded world from turning the cache
/// into a leak; it is generous enough that a normal island never trips it.
const NEIGHBOR_CACHE_CAP: usize = 1 << 20;

#[derive(Debug, Clone, PartialEq)]
struct Walker {
    kind: String,
    steps: u64,
    /// Anchored agents never move. Packs use them for fixed features, and they
    /// double as proof that per-agent state survives a checkpoint round trip.
    anchored: bool,
}

pub struct Walkers {
    grid: SparseCellGrid,
    walkers: HashMap<AgentId, Walker>,
    neighbors: HashMap<GeoCell, Vec<GeoCell>>,
    rng: Rng,
    seed: u64,
    next_id: AgentId,
    /// Chance an agent holds its cell on a given tick.
    stay_probability: f64,
    /// `gold` runs iterate agents in id order and draw from a per-agent stream,
    /// so the result does not depend on hash iteration order (spec §6.5).
    gold: bool,
    /// Agents an intent placed for the coming tick. They hold that cell rather
    /// than walking, so "move to X" and "spawn at X" mean X after the tick.
    placed: HashSet<AgentId>,
}

/// Registered as `walkers` in the model registry.
pub fn build(blob: &ScenarioBlob) -> anyhow::Result<Box<dyn Model>> {
    let mut model = Walkers {
        grid: SparseCellGrid::new(blob.config.grid.resolution, blob.config.grid.root),
        walkers: HashMap::new(),
        neighbors: HashMap::new(),
        rng: Rng::default(),
        seed: 0,
        next_id: 1,
        stay_probability: 0.0,
        gold: false,
        placed: HashSet::new(),
    };
    model.reset(blob)?;
    Ok(Box::new(model))
}

/// Where a population seeded by count starts: a disk of `spread_k` cells around
/// `origin_lon`/`origin_lat`, or around the island root's centre, filtered to
/// cells the world accepts. Shared with the krABMaga-backed variant so both
/// models start from the same world given the same pack.
pub(crate) fn seed_area(
    blob: &ScenarioBlob,
    accepts: impl Fn(GeoCell) -> bool,
) -> anyhow::Result<Vec<GeoCell>> {
    let params = &blob.config.model;
    let resolution = blob.config.grid.resolution;
    let spread = params
        .get("spread_k")
        .and_then(Param::as_i64)
        .unwrap_or(3)
        .clamp(0, 24) as usize;

    let origin = match (
        params.get("origin_lon").and_then(Param::as_f64),
        params.get("origin_lat").and_then(Param::as_f64),
    ) {
        (Some(lon), Some(lat)) => GeoCell::from_lonlat(lon, lat, resolution)?,
        _ => match blob.config.grid.root {
            Some(root) => {
                let (lon, lat) = root.centroid()?;
                GeoCell::from_lonlat(lon, lat, resolution)?
            }
            None => GeoCell::from_lonlat(0.0, 0.0, resolution)?,
        },
    };

    let area: Vec<GeoCell> = origin
        .grid_disk(spread)?
        .into_iter()
        .filter(|c| accepts(*c))
        .collect();
    if area.is_empty() {
        anyhow::bail!(
            "seed area around {origin} is empty at resolution {resolution}; \
             check origin_lon/origin_lat against the scenario root"
        );
    }
    Ok(area)
}

impl Walkers {
    fn params(blob: &ScenarioBlob) -> &BTreeMap<String, Param> {
        &blob.config.model
    }

    /// Cached, root-filtered neighbours of a cell.
    fn neighbors_of(&mut self, cell: GeoCell) -> &[GeoCell] {
        if !self.neighbors.contains_key(&cell) {
            if self.neighbors.len() >= NEIGHBOR_CACHE_CAP {
                self.neighbors.clear();
            }
            let mut options = cell.neighbors().unwrap_or_default();
            options.retain(|c| self.grid.accepts(*c));
            options.sort_unstable();
            self.neighbors.insert(cell, options);
        }
        &self.neighbors[&cell]
    }

    fn spawn_at(&mut self, cell: GeoCell, kind: String, anchored: bool) -> AgentId {
        let id = self.next_id;
        self.next_id += 1;
        self.walkers.insert(
            id,
            Walker {
                kind,
                steps: 0,
                anchored,
            },
        );
        self.grid.set_location(id, cell);
        id
    }

    fn walks_this_tick(&self, agent: AgentId) -> bool {
        !self.placed.contains(&agent)
    }

    fn move_one(&mut self, agent: AgentId, from: GeoCell, roll: f64, pick: u64) {
        if roll < self.stay_probability {
            return;
        }
        if self.walkers.get(&agent).is_some_and(|w| w.anchored) {
            return;
        }
        let options = self.neighbors_of(from);
        if options.is_empty() {
            return;
        }
        let target = options[(pick % options.len() as u64) as usize];
        self.grid.set_location(agent, target);
        if let Some(walker) = self.walkers.get_mut(&agent) {
            walker.steps += 1;
        }
    }
}

impl Model for Walkers {
    fn kind(&self) -> &'static str {
        "walkers"
    }

    fn reset(&mut self, blob: &ScenarioBlob) -> anyhow::Result<()> {
        let params = Self::params(blob);
        let resolution = blob.config.grid.resolution;

        self.grid.reconfigure(resolution, blob.config.grid.root);
        self.walkers.clear();
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

        // Authored agents and a bare population count are additive: a pack may
        // pin a handful of named agents *and* ask for a crowd around them.
        // Named ids are placed first so the allocator can step over them.
        for seed in &blob.agents {
            let cell = seed.resolve_cell(resolution)?;
            if !self.grid.accepts(cell) {
                anyhow::bail!("seed agent at {cell} is outside the scenario's world");
            }
            let anchored = seed
                .state
                .get("anchored")
                .and_then(serde_json::Value::as_bool)
                .unwrap_or(false);
            let id = seed.id.unwrap_or(self.next_id);
            self.walkers.insert(
                id,
                Walker {
                    kind: seed.kind.clone(),
                    steps: 0,
                    anchored,
                },
            );
            self.grid.set_location(id, cell);
            self.next_id = self.next_id.max(id + 1);
        }

        let count = params
            .get("walkers")
            .and_then(Param::as_i64)
            .unwrap_or(0)
            .max(0) as u64;
        if count > 0 {
            let area = seed_area(blob, |c| self.grid.accepts(c))?;
            for i in 0..count {
                let cell = area[(i as usize) % area.len()];
                self.spawn_at(cell, "walker".to_owned(), false);
            }
        }

        self.grid.commit_now();
        Ok(())
    }

    fn apply_intent(&mut self, intent: &Intent) -> Result<(), RejectReason> {
        if !self.grid.accepts(intent.cell) {
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
                    let id = self.spawn_at(intent.cell, kind.clone(), anchored);
                    self.placed.insert(id);
                }
                Ok(())
            }
            IntentVerb::Despawn => match intent.agent {
                Some(agent) => {
                    if self.walkers.remove(&agent).is_none() {
                        return Err(RejectReason::Rejected {
                            message: format!("no agent {agent}"),
                        });
                    }
                    self.grid.remove(agent);
                    Ok(())
                }
                None => {
                    // No agent named: clear the cell.
                    for agent in self.grid.occupants(intent.cell).to_vec() {
                        self.walkers.remove(&agent);
                        self.grid.remove(agent);
                    }
                    Ok(())
                }
            },
            IntentVerb::Move => {
                let agent = intent.agent.ok_or(RejectReason::Rejected {
                    message: "move needs an agent id".to_owned(),
                })?;
                if !self.walkers.contains_key(&agent) {
                    return Err(RejectReason::Rejected {
                        message: format!("no agent {agent}"),
                    });
                }
                self.grid.set_location(agent, intent.cell);
                self.placed.insert(agent);
                Ok(())
            }
            IntentVerb::SetField | IntentVerb::Custom(_) => Err(RejectReason::Rejected {
                message: format!("the walkers model has no handler for {:?}", intent.verb),
            }),
        }
    }

    fn step(&mut self, tick: u64) {
        if self.gold {
            // Deterministic: fixed agent order, and each agent's draw depends
            // only on (seed, agent, tick), never on where it sits in a HashMap.
            let mut agents: Vec<AgentId> = self.walkers.keys().copied().collect();
            agents.sort_unstable();
            for agent in agents {
                let Some(from) = self.grid.location(agent) else {
                    continue;
                };
                if !self.walks_this_tick(agent) {
                    continue;
                }
                let (roll, pick) = gold_draw(self.seed, agent, tick);
                self.move_one(agent, from, roll, pick);
            }
        } else {
            let agents: Vec<AgentId> = self.walkers.keys().copied().collect();
            for agent in agents {
                let Some(from) = self.grid.location(agent) else {
                    continue;
                };
                if !self.walks_this_tick(agent) {
                    continue;
                }
                let roll = self.rng.unit();
                let pick = self.rng.next_u64();
                self.move_one(agent, from, roll, pick);
            }
        }
        self.placed.clear();
        self.grid.lazy_update();
    }

    fn snapshot(&self, out: &mut Vec<CellBag>, limit: usize) -> Truncation {
        out.clear();
        let cells = self.grid.sorted_cells();
        let mut truncation = Truncation::default();
        for cell in cells {
            if out.len() >= limit {
                truncation.cells_omitted += 1;
                continue;
            }
            let mut objects: Vec<WorldObject> = self
                .grid
                .occupants(cell)
                .iter()
                .filter_map(|agent| {
                    let walker = self.walkers.get(agent)?;
                    Some(WorldObject {
                        id: *agent,
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
            out.push(CellBag { cell, objects });
            truncation.cells_written += 1;
        }
        truncation
    }

    fn agent_count(&self) -> u64 {
        self.grid.agent_count()
    }

    fn occupied_cells(&self) -> u64 {
        self.grid.occupied_cells()
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
        if let Some(next) = fields.get("next_id").and_then(serde_json::Value::as_u64) {
            self.next_id = next;
        }
        if let Some(seed) = fields.get("seed").and_then(serde_json::Value::as_u64) {
            self.seed = seed;
        }
        if let Some(p) = fields
            .get("stay_probability")
            .and_then(serde_json::Value::as_f64)
        {
            self.stay_probability = p.clamp(0.0, 1.0);
        }
        if let Some(gold) = fields.get("gold").and_then(serde_json::Value::as_bool) {
            self.gold = gold;
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
        self.grid.clear();
        self.walkers.clear();
        for bag in cells {
            if !self.grid.accepts(bag.cell) {
                anyhow::bail!(
                    "checkpoint cell {} does not belong to this world (resolution {}, root {:?})",
                    bag.cell,
                    self.grid.resolution(),
                    self.grid.root()
                );
            }
            for object in bag.objects {
                let steps = object
                    .state
                    .get("steps")
                    .and_then(serde_json::Value::as_u64)
                    .unwrap_or(0);
                let anchored = object
                    .state
                    .get("anchored")
                    .and_then(serde_json::Value::as_bool)
                    .unwrap_or(false);
                self.walkers.insert(
                    object.id,
                    Walker {
                        kind: object.kind,
                        steps,
                        anchored,
                    },
                );
                self.grid.set_location(object.id, bag.cell);
                self.next_id = self.next_id.max(object.id + 1);
            }
        }
        self.grid.commit_now();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{ActorId, Config, ScenarioManifest};
    use uuid::Uuid;

    fn blob(profile: Profile, walkers: i64) -> ScenarioBlob {
        let manifest = ScenarioManifest {
            id: "test".into(),
            name: "test".into(),
            description: String::new(),
            model: "walkers".into(),
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

    fn intent(cell: GeoCell, verb: IntentVerb, payload: serde_json::Value) -> Intent {
        Intent {
            id: Uuid::new_v4(),
            tick: None,
            cell,
            agent: None,
            actor: ActorId::operator(),
            verb,
            payload,
            issued_at: 0,
        }
    }

    #[test]
    fn a_seeded_world_conserves_its_agents_across_ticks() {
        let mut model = build(&blob(Profile::Fast, 500)).unwrap();
        assert_eq!(model.agent_count(), 500);
        for tick in 1..=100 {
            model.step(tick);
            assert_eq!(model.agent_count(), 500, "agents vanished on tick {tick}");
        }
        assert!(model.occupied_cells() > 1, "the walk should spread out");
    }

    #[test]
    fn gold_runs_are_reproducible_tick_for_tick() {
        let scenario = blob(Profile::Gold, 200);
        let mut left = build(&scenario).unwrap();
        let mut right = build(&scenario).unwrap();

        let (mut a, mut b) = (Vec::new(), Vec::new());
        for tick in 1..=50 {
            left.step(tick);
            right.step(tick);
        }
        left.snapshot(&mut a, usize::MAX);
        right.snapshot(&mut b, usize::MAX);
        assert_eq!(
            sim_core::snapshot::stable_hash(&a),
            sim_core::snapshot::stable_hash(&b)
        );
        assert_eq!(a, b);
    }

    #[test]
    fn spawn_and_despawn_intents_change_the_world_on_the_next_tick() {
        let mut model = build(&blob(Profile::Fast, 0)).unwrap();
        let cell = GeoCell::from_lonlat(-0.1276, 51.5072, 6).unwrap();

        model
            .apply_intent(&intent(
                cell,
                IntentVerb::Spawn,
                serde_json::json!({"count": 3, "kind": "scout"}),
            ))
            .unwrap();
        assert_eq!(model.agent_count(), 0, "not committed yet");

        model.step(1);
        assert_eq!(model.agent_count(), 3);

        let mut view = Vec::new();
        model.snapshot(&mut view, usize::MAX);
        assert!(view
            .iter()
            .flat_map(|b| &b.objects)
            .all(|o| o.kind == "scout"));

        model
            .apply_intent(&intent(cell, IntentVerb::Despawn, serde_json::Value::Null))
            .unwrap();
        model.step(2);
        assert_eq!(model.agent_count(), 0);
    }

    #[test]
    fn an_intent_outside_the_world_is_rejected_by_the_model() {
        let mut scenario = blob(Profile::Fast, 0);
        let root = GeoCell::from_lonlat(-0.1276, 51.5072, 2).unwrap();
        scenario.manifest.root = Some(root);
        scenario.config.grid.root = Some(root);
        let mut model = build(&scenario).unwrap();

        let elsewhere = GeoCell::from_lonlat(151.2093, -33.8688, 6).unwrap();
        assert_eq!(
            model.apply_intent(&intent(
                elsewhere,
                IntentVerb::Spawn,
                serde_json::Value::Null
            )),
            Err(RejectReason::OutOfBounds)
        );
    }

    #[test]
    fn unhandled_verbs_are_rejected_with_a_message_not_a_panic() {
        let mut model = build(&blob(Profile::Fast, 0)).unwrap();
        let cell = GeoCell::from_lonlat(-0.1276, 51.5072, 6).unwrap();
        let err = model
            .apply_intent(&intent(
                cell,
                IntentVerb::Custom(42),
                serde_json::Value::Null,
            ))
            .unwrap_err();
        assert!(matches!(err, RejectReason::Rejected { .. }));
    }

    #[test]
    fn a_snapshot_round_trips_through_load_cells() {
        let scenario = blob(Profile::Gold, 120);
        let mut model = build(&scenario).unwrap();
        for tick in 1..=10 {
            model.step(tick);
        }
        let mut before = Vec::new();
        model.snapshot(&mut before, usize::MAX);

        let mut restored = build(&scenario).unwrap();
        restored.load_cells(before.clone()).unwrap();
        restored.restore_fields(&model.fields()).unwrap();

        let mut after = Vec::new();
        restored.snapshot(&mut after, usize::MAX);
        assert_eq!(before, after);
        assert_eq!(restored.agent_count(), model.agent_count());
    }

    #[test]
    fn a_capped_snapshot_reports_what_it_left_out() {
        let mut model = build(&blob(Profile::Fast, 400)).unwrap();
        for tick in 1..=20 {
            model.step(tick);
        }
        let mut view = Vec::new();
        let full = model.snapshot(&mut view, usize::MAX);
        assert!(!full.is_truncated());

        let capped = model.snapshot(&mut view, 2);
        assert_eq!(view.len(), 2);
        assert!(capped.is_truncated());
        assert_eq!(
            capped.cells_written + capped.cells_omitted,
            full.cells_written
        );
    }
}
