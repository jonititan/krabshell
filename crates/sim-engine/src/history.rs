//! Rewind: the recent past, rebuilt on request (gold profile only).
//!
//! The tick thread hands this thread a keyframe of the world every
//! `run.history_every_ticks` ticks, and every intent it applies. To show tick
//! `T`, this thread restores the last keyframe at or before `T` into a model
//! of its own, applies the logged intents tick by tick exactly as the driver
//! did, and steps up to `T`. A gold run replays to the bit, so the view it
//! builds is the world as it was, not a recording of it: any tick in the
//! window, with everything the live view carries.
//!
//! Taking a keyframe on the tick thread is a frame handover (the same one the
//! publisher uses), so it costs the live run next to nothing; the frame is
//! written out and kept here as JSON. The model it replays on is kept between
//! requests, so stepping forward is one step and stepping back replays from
//! the keyframe before.
//!
//! A new world (a load, a reset, a restore) starts the history afresh. The
//! live run never waits on this thread.

use std::collections::VecDeque;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use crossbeam_channel::{bounded, unbounded, Receiver, Sender};
use sim_core::intent::LoggedIntent;
use sim_core::{CellBag, Profile, ScenarioBlob};

use crate::driver::Published;
use crate::model::{Frame, Model, ModelRegistry};

/// Replayed views kept for repeat requests: a dashboard asks for the same
/// tick several ways at once (entities, overlays, the control panel).
const VIEW_CACHE: usize = 8;

/// How long a request waits for its view.
const VIEW_TIMEOUT: Duration = Duration::from_secs(30);

/// What the recent past holds, for the API.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HistoryRange {
    /// The oldest tick that can be shown, once there is one.
    pub first_tick: Option<u64>,
    pub every_ticks: u64,
    /// Why there is no rewind, when there is none.
    pub off: Option<&'static str>,
}

/// What the tick thread and the handles share with the history thread.
pub(crate) struct Shared {
    first_tick: AtomicU64,
    every_ticks: AtomicU64,
    on: AtomicBool,
    off_why: std::sync::Mutex<&'static str>,
}

impl Shared {
    fn set_off(&self, why: &'static str) {
        self.on.store(false, Ordering::Release);
        self.first_tick.store(u64::MAX, Ordering::Release);
        *self.off_why.lock().unwrap_or_else(|e| e.into_inner()) = why;
    }
}

enum Msg {
    /// A new world: forget the old one's past.
    Begin {
        epoch: u64,
        /// The scenario, without its seed agents: a keyframe supplies them.
        blob: Box<ScenarioBlob>,
        keep: usize,
    },
    Keyframe {
        epoch: u64,
        tick: u64,
        frame: Box<dyn Frame>,
        fields: serde_json::Value,
        rng: u64,
    },
    Applied {
        epoch: u64,
        logged: LoggedIntent,
    },
    ViewAt {
        tick: u64,
        live: Arc<Published>,
        reply: Sender<Result<Arc<Published>, String>>,
    },
    Stop,
}

/// The tick thread's end.
pub(crate) struct History {
    tx: Sender<Msg>,
    shared: Arc<Shared>,
    join: Option<JoinHandle<()>>,
    epoch: u64,
    every: u64,
    on: bool,
}

/// A handle for asking about the past, cheap to clone.
#[derive(Clone)]
pub struct HistoryHandle {
    tx: Sender<Msg>,
    shared: Arc<Shared>,
}

impl History {
    pub(crate) fn spawn(registry: ModelRegistry) -> std::io::Result<History> {
        let (tx, rx) = unbounded();
        let shared = Arc::new(Shared {
            first_tick: AtomicU64::new(u64::MAX),
            every_ticks: AtomicU64::new(0),
            on: AtomicBool::new(false),
            off_why: std::sync::Mutex::new("no world yet"),
        });
        let join = std::thread::Builder::new()
            .name("sim-history".into())
            .spawn({
                let shared = Arc::clone(&shared);
                move || Replayer::new(registry, shared).run(rx)
            })?;
        Ok(History {
            tx,
            shared,
            join: Some(join),
            epoch: 0,
            every: 0,
            on: false,
        })
    }

    pub(crate) fn handle(&self) -> HistoryHandle {
        HistoryHandle {
            tx: self.tx.clone(),
            shared: Arc::clone(&self.shared),
        }
    }

    /// A new world at its first tick: start the history from it.
    pub(crate) fn begin(&mut self, blob: &ScenarioBlob, model: &dyn Model, tick: u64) {
        self.epoch += 1;
        let run = &blob.config.run;
        self.every = run.history_every_ticks.max(1);
        self.on = run.profile == Profile::Gold && run.history_keyframes > 0;
        let why = if run.profile != Profile::Gold {
            Some("rewind needs the gold profile, which replays exactly")
        } else if run.history_keyframes == 0 {
            Some("run.history_keyframes is 0")
        } else if model.frame().is_none() {
            // Without a frame there is no cheap handover, and rewind is not
            // worth slowing the tick thread down for.
            Some("this model cannot hand its world over cheaply")
        } else {
            None
        };
        if let Some(why) = why {
            self.on = false;
            self.shared.set_off(why);
            return;
        }
        self.shared.every_ticks.store(self.every, Ordering::Release);
        let mut empty = blob.clone();
        empty.agents.clear();
        let _ = self.tx.send(Msg::Begin {
            epoch: self.epoch,
            blob: Box::new(empty),
            keep: run.history_keyframes,
        });
        self.keyframe(model, tick);
        self.shared.on.store(true, Ordering::Release);
    }

    /// After a tick: a keyframe, if one is due.
    pub(crate) fn ticked(&mut self, model: &dyn Model, tick: u64) {
        if self.on && tick.is_multiple_of(self.every) {
            self.keyframe(model, tick);
        }
    }

    /// An intent the driver applied before stepping to `tick`.
    pub(crate) fn applied(&self, tick: u64, intent: &sim_core::Intent) {
        if self.on {
            let _ = self.tx.send(Msg::Applied {
                epoch: self.epoch,
                logged: LoggedIntent {
                    tick,
                    intent: intent.clone(),
                },
            });
        }
    }

    fn keyframe(&self, model: &dyn Model, tick: u64) {
        let Some(frame) = model.frame() else {
            return;
        };
        let _ = self.tx.send(Msg::Keyframe {
            epoch: self.epoch,
            tick,
            frame,
            fields: model.fields(),
            rng: model.rng_state(),
        });
    }

    pub(crate) fn stop(&mut self) {
        let _ = self.tx.send(Msg::Stop);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

impl Drop for History {
    fn drop(&mut self) {
        self.stop();
    }
}

impl HistoryHandle {
    pub fn range(&self) -> HistoryRange {
        let first = self.shared.first_tick.load(Ordering::Acquire);
        let on = self.shared.on.load(Ordering::Acquire);
        HistoryRange {
            first_tick: (on && first != u64::MAX).then_some(first),
            every_ticks: self.shared.every_ticks.load(Ordering::Acquire),
            off: (!on).then(|| {
                *self
                    .shared
                    .off_why
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
            }),
        }
    }

    /// The world at `tick`, rebuilt from the history. `live` is the current
    /// view: `tick` may not be past it, and the view borrows its config,
    /// names and recent reports.
    pub(crate) fn view_at(
        &self,
        tick: u64,
        live: Arc<Published>,
    ) -> Result<Arc<Published>, String> {
        let (reply, answer) = bounded(1);
        self.tx
            .send(Msg::ViewAt { tick, live, reply })
            .map_err(|_| "the history thread has stopped".to_owned())?;
        answer
            .recv_timeout(VIEW_TIMEOUT)
            .map_err(|_| "the history did not answer in time".to_owned())?
    }
}

struct Keyframe {
    tick: u64,
    /// The cells, written out as JSON: a fraction of their size as values.
    cells: Vec<u8>,
    /// Shared with the keyframe before when unchanged, as fields mostly are.
    fields: Arc<serde_json::Value>,
    rng: u64,
}

/// A model replaying the past, at `tick`.
struct Replay {
    model: Box<dyn Model>,
    tick: u64,
}

struct Replayer {
    registry: ModelRegistry,
    shared: Arc<Shared>,
    epoch: u64,
    blob: Option<ScenarioBlob>,
    keep: usize,
    keyframes: VecDeque<Keyframe>,
    /// Intents applied after the oldest keyframe, in the order applied.
    log: VecDeque<LoggedIntent>,
    replay: Option<Replay>,
    views: VecDeque<Arc<Published>>,
}

impl Replayer {
    fn new(registry: ModelRegistry, shared: Arc<Shared>) -> Replayer {
        Replayer {
            registry,
            shared,
            epoch: 0,
            blob: None,
            keep: 0,
            keyframes: VecDeque::new(),
            log: VecDeque::new(),
            replay: None,
            views: VecDeque::new(),
        }
    }

    fn run(mut self, rx: Receiver<Msg>) {
        while let Ok(msg) = rx.recv() {
            match msg {
                Msg::Begin { epoch, blob, keep } => {
                    self.epoch = epoch;
                    self.blob = Some(*blob);
                    self.keep = keep.max(1);
                    self.keyframes.clear();
                    self.log.clear();
                    self.replay = None;
                    self.views.clear();
                }
                Msg::Keyframe {
                    epoch,
                    tick,
                    frame,
                    fields,
                    rng,
                } if epoch == self.epoch => self.keep_keyframe(tick, frame, fields, rng),
                Msg::Applied { epoch, logged } if epoch == self.epoch => self.log.push_back(logged),
                Msg::ViewAt { tick, live, reply } => {
                    let view = catch_unwind(AssertUnwindSafe(|| self.view_at(tick, &live)))
                        .unwrap_or_else(|_| {
                            // A replay that panicked leaves its model unusable.
                            self.replay = None;
                            Err(format!("replaying to tick {tick} panicked"))
                        });
                    let _ = reply.send(view);
                }
                Msg::Stop => return,
                // From a world already replaced.
                Msg::Keyframe { .. } | Msg::Applied { .. } => {}
            }
        }
    }

    fn keep_keyframe(
        &mut self,
        tick: u64,
        frame: Box<dyn Frame>,
        fields: serde_json::Value,
        rng: u64,
    ) {
        let mut cells = Vec::new();
        frame.snapshot(&mut cells, usize::MAX);
        let Ok(cells) = serde_json::to_vec(&cells) else {
            return;
        };
        let fields = match self.keyframes.back() {
            Some(last) if *last.fields == fields => Arc::clone(&last.fields),
            _ => Arc::new(fields),
        };
        self.keyframes.push_back(Keyframe {
            tick,
            cells,
            fields,
            rng,
        });
        while self.keyframes.len() > self.keep {
            self.keyframes.pop_front();
        }
        // An intent logged at tick t was applied before stepping to t, so a
        // keyframe taken after tick k already holds every one up to k.
        let first = self.keyframes.front().map_or(tick, |k| k.tick);
        while self.log.front().is_some_and(|l| l.tick <= first) {
            self.log.pop_front();
        }
        self.shared.first_tick.store(first, Ordering::Release);
    }

    fn view_at(&mut self, tick: u64, live: &Published) -> Result<Arc<Published>, String> {
        if live.meta.scenario_id != self.blob.as_ref().map_or("", |b| b.manifest.id.as_str()) {
            return Err("the history is for another world".into());
        }
        if tick > live.meta.tick {
            return Err(format!(
                "tick {tick} has not happened yet (now {})",
                live.meta.tick
            ));
        }
        let Some(first) = self.keyframes.front().map(|k| k.tick) else {
            return Err("no history yet".into());
        };
        if tick < first {
            return Err(format!(
                "tick {tick} is older than the history, which starts at {first}"
            ));
        }
        if let Some(view) = self.views.iter().find(|v| v.meta.tick == tick) {
            return Ok(Arc::clone(view));
        }

        let key = self
            .keyframes
            .iter()
            .rev()
            .find(|k| k.tick <= tick)
            .expect("the first keyframe is at or before tick");
        // Carry on from where the last replay stopped if it is on the way;
        // otherwise start again from the keyframe.
        let reusable = self
            .replay
            .as_ref()
            .is_some_and(|r| r.tick >= key.tick && r.tick <= tick);
        if !reusable {
            let blob = self.blob.as_ref().ok_or("no world yet")?;
            let mut model = self.registry.build(blob).map_err(|e| e.to_string())?;
            let cells: Vec<CellBag> =
                serde_json::from_slice(&key.cells).map_err(|e| e.to_string())?;
            model.load_cells(cells).map_err(|e| e.to_string())?;
            model
                .restore_fields(&key.fields)
                .map_err(|e| e.to_string())?;
            model.set_rng_state(key.rng);
            model.set_tick(key.tick);
            self.replay = Some(Replay {
                model,
                tick: key.tick,
            });
        }
        let replay = self.replay.as_mut().expect("set above");
        let mut reports = Vec::new();
        let start = self.log.partition_point(|l| l.tick <= replay.tick);
        let mut next = start;
        while replay.tick < tick {
            let t = replay.tick + 1;
            // As the driver does: that tick's intents, in the order applied,
            // then the step.
            while let Some(l) = self.log.get(next).filter(|l| l.tick == t) {
                replay.model.apply_intent(&l.intent).map_err(|e| {
                    format!("intent {} did not replay at tick {t}: {e}", l.intent.id)
                })?;
                next += 1;
            }
            replay.model.step(t);
            replay.model.drain_reports(&mut reports);
            reports.clear();
            replay.tick = t;
        }

        let view = Arc::new(build_view(replay.model.as_ref(), tick, live));
        if self.views.len() >= VIEW_CACHE {
            self.views.pop_front();
        }
        self.views.push_back(Arc::clone(&view));
        Ok(view)
    }
}

/// A published view of a replayed world, shaped like the live one.
fn build_view(model: &dyn Model, tick: u64, live: &Published) -> Published {
    let limit = live.config.run.max_published_cells;
    let mut cells = Vec::new();
    let truncation = model.snapshot(&mut cells, limit);
    let mut view = Published {
        meta: live.meta.clone(),
        metrics: live.metrics,
        health: live.health.clone(),
        config: live.config.clone(),
        scenario_id: live.scenario_id.clone(),
        scenario_name: live.scenario_name.clone(),
        model_kind: live.model_kind.clone(),
        cells: Vec::new(),
        truncated: truncation.is_truncated(),
        published_at_ms: live.published_at_ms,
        recent_intents: live
            .recent_intents
            .iter()
            .filter(|r| r.tick <= tick)
            .cloned()
            .collect(),
        reports: live
            .reports
            .iter()
            .filter(|r| r.tick <= tick)
            .cloned()
            .collect(),
        overlays: model.overlays(),
    };
    view.meta.tick = tick;
    view.meta.agent_count = model.agent_count();
    view.meta.occupied_cells = model.occupied_cells();
    view.meta.hash = (view.meta.profile == Profile::Gold && !view.truncated)
        .then(|| sim_core::snapshot::stable_hash(&cells));
    view.metrics.tick = tick;
    view.metrics.agent_count = view.meta.agent_count;
    view.metrics.occupied_cells = view.meta.occupied_cells;
    view.metrics.truncated_cells = truncation.cells_omitted;
    view.cells = cells;
    view
}
