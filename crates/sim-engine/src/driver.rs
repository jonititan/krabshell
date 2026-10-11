//! The tick thread (spec §6.1).
//!
//! One `std::thread` owns the world. Nothing else touches it. The HTTP edge
//! talks to it through two bounded channels in, and one published snapshot out:
//!
//! - control commands (start, pause, step, load, checkpoint),
//! - intent submissions,
//! - an [`ArcSwap`] holding the most recent committed view.
//!
//! The publish side is a pointer swap, so a slow reader can never stall a tick,
//! and a fast tick loop can never queue unbounded frames for a slow socket
//! (spec §9.3, §12). Readers may miss intermediate frames; that is the design.
//! Views are built on a second thread (see [`crate::publisher`]) when the
//! model can hand over a frame of its committed world.

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use arc_swap::ArcSwap;
use crossbeam_channel::{bounded, Receiver, RecvTimeoutError, Sender, TrySendError};
use sim_core::{
    CellBag, Checkpoint, Config, DriverHealth, GeoCell, Intent, Metrics, Profile, RejectReason,
    Report, RunMode, ScenarioBlob, SnapshotMeta,
};
use uuid::Uuid;

use crate::admission::{Admission, AdmissionOutcome};
use crate::history::{History, HistoryHandle, HistoryRange};
use crate::model::{Model, ModelRegistry};
use crate::publisher::{Cells, Publisher};

/// Control channel depth. Control messages are operator-scale, not agent-scale.
const CONTROL_CAPACITY: usize = 64;

/// How long the driver sleeps between wake-ups while paused.
const IDLE_POLL: Duration = Duration::from_millis(50);

/// Ring of recent intent outcomes kept for the dashboard.
const RECENT_INTENTS: usize = 32;

/// Reports kept in the published view. A consumer polling at the publish rate
/// sees every one; a slower one sees a gap in `seq` and can say so.
const RECENT_REPORTS: usize = 512;

/// Upper bound on the replay log held in memory for a `gold` run.
const INTENT_LOG_CAP: usize = 100_000;

/// What the operator asked the driver to do.
#[derive(Debug)]
pub enum Control {
    Start {
        target_tps: Option<f64>,
    },
    Pause,
    /// Advance `n` ticks, then pause.
    Step {
        n: u64,
    },
    /// Rebuild the world from the scenario currently loaded.
    Reset,
    Load(Box<ScenarioBlob>),
    SetRate(Option<f64>),
    /// Adopt a new config. Fields that need a reset are applied by reloading.
    SetConfig(Box<Config>),
    Restore(Box<Checkpoint>),
    /// Copy the whole committed world into an owned buffer for the caller to
    /// write to disk off-thread (spec §12).
    Snapshot(Sender<Box<Checkpoint>>),
    Shutdown,
}

/// A control message and an optional place to report the result.
#[derive(Debug)]
pub struct Command {
    pub control: Control,
    pub ack: Option<Sender<Result<(), String>>>,
}

impl Command {
    pub fn new(control: Control) -> Self {
        Command { control, ack: None }
    }

    fn reply(&self, result: Result<(), String>) {
        if let Some(ack) = &self.ack {
            let _ = ack.try_send(result);
        }
    }
}

/// The fate of a submitted intent, sent back on the submitting connection.
#[derive(Debug, Clone, PartialEq)]
pub enum IntentAck {
    /// Queued and applied on this tick.
    Applied {
        tick: u64,
    },
    Rejected(RejectReason),
}

/// An intent plus somewhere to report its fate.
#[derive(Debug)]
pub struct Submission {
    pub intent: Intent,
    pub ack: Option<Sender<IntentAck>>,
}

impl Submission {
    pub fn fire_and_forget(intent: Intent) -> Self {
        Submission { intent, ack: None }
    }

    fn resolve(&self, ack: IntentAck) {
        if let Some(tx) = &self.ack {
            let _ = tx.try_send(ack);
        }
    }
}

/// One line of the dashboard's intent activity list.
#[derive(Debug, Clone, serde::Serialize)]
pub struct IntentRecord {
    pub id: Uuid,
    pub actor: String,
    pub verb: String,
    pub cell: GeoCell,
    pub tick: u64,
    /// `None` when applied; the rejection code otherwise.
    pub rejected: Option<String>,
    pub detail: String,
}

/// The committed world as the HTTP edge sees it.
///
/// Rebuilt at most `publish_hz` times a second, and after operator actions,
/// and swapped in atomically. Readers clone the `Arc`, never the contents.
#[derive(Debug, Clone)]
pub struct Published {
    pub meta: SnapshotMeta,
    pub metrics: Metrics,
    pub health: DriverHealth,
    pub config: Config,
    pub scenario_id: String,
    pub scenario_name: String,
    pub model_kind: String,
    /// Committed occupancy, sorted by cell. Capped at `max_published_cells`.
    pub cells: Vec<CellBag>,
    /// True when the cap dropped cells from `cells`.
    pub truncated: bool,
    pub published_at_ms: u64,
    pub recent_intents: Vec<IntentRecord>,
    /// The most recent model reports, oldest first, by ascending `seq`.
    pub reports: Vec<Report>,
    /// What the model publishes beside its agents; see [`Model::overlays`].
    pub overlays: serde_json::Value,
}

impl Published {
    fn empty(config: Config, blob: &ScenarioBlob) -> Self {
        Published {
            meta: SnapshotMeta {
                scenario_id: blob.manifest.id.clone(),
                tick: 0,
                profile: config.run.profile,
                root: config.grid.root,
                resolution: config.grid.resolution,
                agent_count: 0,
                occupied_cells: 0,
                hash: None,
            },
            metrics: Metrics::default(),
            health: DriverHealth::default(),
            config,
            scenario_id: blob.manifest.id.clone(),
            scenario_name: blob.manifest.name.clone(),
            model_kind: blob.manifest.model.clone(),
            cells: Vec::new(),
            truncated: false,
            published_at_ms: now_ms(),
            recent_intents: Vec::new(),
            reports: Vec::new(),
            overlays: serde_json::Value::Null,
        }
    }

    /// Reports newer than `seq`, oldest first.
    pub fn reports_since(&self, seq: u64) -> &[Report] {
        let start = self.reports.partition_point(|r| r.seq <= seq);
        &self.reports[start..]
    }

    /// The bag at one cell, answered from the published view.
    ///
    /// Cells are in ascending order, so this is a binary search. Returns `None`
    /// for an empty cell — and also for a cell the cap dropped, which is why
    /// callers should surface [`Published::truncated`].
    pub fn bag(&self, cell: GeoCell) -> Option<&CellBag> {
        self.cells
            .binary_search_by_key(&cell, |b| b.cell)
            .ok()
            .map(|i| &self.cells[i])
    }

    /// Every occupied bag that is `cell` or lies inside it, in cell order.
    ///
    /// At the occupancy resolution this is at most one bag. At a coarser one it
    /// is what the dashboard shows when an operator clicks a map polygon: the
    /// agents the polygon's count was made of.
    pub fn bags_within(&self, cell: GeoCell) -> Vec<&CellBag> {
        if cell.resolution() >= self.meta.resolution {
            return self.bag(cell).into_iter().collect();
        }
        self.cells
            .iter()
            .filter(|b| b.cell.is_within(cell))
            .collect()
    }

    /// Occupancy rolled up to a coarser resolution, for the dashboard map.
    pub fn aggregate(&self, resolution: i32) -> Vec<(GeoCell, u64)> {
        let mut counts: std::collections::HashMap<GeoCell, u64> = std::collections::HashMap::new();
        for bag in &self.cells {
            let parent = if bag.cell.resolution() <= resolution {
                bag.cell
            } else {
                match bag.cell.parent(Some(resolution)) {
                    Ok(parent) => parent,
                    Err(_) => continue,
                }
            };
            *counts.entry(parent).or_default() += bag.objects.len() as u64;
        }
        let mut rolled: Vec<(GeoCell, u64)> = counts.into_iter().collect();
        rolled.sort_unstable_by_key(|(cell, _)| *cell);
        rolled
    }
}

/// The handle the rest of the process holds. Cloneable and `Send`.
#[derive(Clone)]
pub struct DriverHandle {
    control: Sender<Command>,
    intents: Sender<Submission>,
    published: Arc<ArcSwap<Published>>,
    history: HistoryHandle,
}

impl DriverHandle {
    /// Latest committed view. Cheap: one atomic load and an `Arc` clone.
    pub fn view(&self) -> Arc<Published> {
        self.published.load_full()
    }

    /// The ticks rewind can show: from the oldest keyframe up to the live
    /// tick. See [`crate::history`].
    pub fn history_range(&self) -> HistoryRange {
        self.history.range()
    }

    /// The world as it was at `tick`, rebuilt from the history; the live view
    /// if `tick` is now. Blocking (it may replay up to
    /// `run.history_every_ticks` ticks), so callers on Tokio must wrap it in
    /// `spawn_blocking`.
    pub fn view_at(&self, tick: u64) -> Result<Arc<Published>, DriverError> {
        let live = self.view();
        if tick == live.meta.tick {
            return Ok(live);
        }
        self.history
            .view_at(tick, live)
            .map_err(DriverError::Refused)
    }

    /// Liveness, with the publish age recomputed against the current clock.
    pub fn health(&self) -> DriverHealth {
        let view = self.view();
        let mut health = view.health.clone();
        health.last_publish_age_ms = now_ms().saturating_sub(view.published_at_ms);
        health
    }

    /// Send a control command and wait for the driver to report the result.
    ///
    /// Blocking, so callers on Tokio must wrap it in `spawn_blocking`. Waiting
    /// is what makes "load this scenario" able to fail with a reason.
    pub fn command(&self, control: Control) -> Result<(), DriverError> {
        let (tx, rx) = bounded(1);
        self.control
            .send(Command {
                control,
                ack: Some(tx),
            })
            .map_err(|_| DriverError::Gone)?;
        match rx.recv_timeout(Duration::from_secs(30)) {
            Ok(Ok(())) => Ok(()),
            Ok(Err(message)) => Err(DriverError::Refused(message)),
            Err(RecvTimeoutError::Timeout) => Err(DriverError::Timeout),
            Err(RecvTimeoutError::Disconnected) => Err(DriverError::Gone),
        }
    }

    /// Fire a control command without waiting.
    pub fn send(&self, control: Control) -> Result<(), DriverError> {
        self.control
            .send(Command::new(control))
            .map_err(|_| DriverError::Gone)
    }

    /// Copy the committed world out for a checkpoint download.
    pub fn checkpoint(&self) -> Result<Box<Checkpoint>, DriverError> {
        let (tx, rx) = bounded(1);
        self.control
            .send(Command::new(Control::Snapshot(tx)))
            .map_err(|_| DriverError::Gone)?;
        rx.recv_timeout(Duration::from_secs(60))
            .map_err(|err| match err {
                RecvTimeoutError::Timeout => DriverError::Timeout,
                RecvTimeoutError::Disconnected => DriverError::Gone,
            })
    }

    /// Offer an intent. Never blocks the caller and never blocks the tick loop:
    /// a full queue is a rejection, not a wait (spec §8.2 rule 4).
    pub fn submit(&self, submission: Submission) -> Result<(), RejectReason> {
        match self.intents.try_send(submission) {
            Ok(()) => Ok(()),
            Err(TrySendError::Full(submission)) => {
                let reason = RejectReason::Backpressured {
                    pending: self.intents.len(),
                };
                submission.resolve(IntentAck::Rejected(reason.clone()));
                Err(reason)
            }
            Err(TrySendError::Disconnected(_)) => Err(RejectReason::Rejected {
                message: "driver is not running".to_owned(),
            }),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum DriverError {
    #[error("the driver thread is gone")]
    Gone,
    #[error("the driver did not answer in time")]
    Timeout,
    #[error("{0}")]
    Refused(String),
}

/// Owns the thread. Dropping it asks the driver to stop and waits for it.
pub struct Driver {
    handle: DriverHandle,
    join: Option<JoinHandle<()>>,
}

impl Driver {
    /// Start a driver on the given scenario. The world is built before this
    /// returns, so a bad pack fails here rather than on the first tick.
    pub fn spawn(registry: ModelRegistry, blob: ScenarioBlob) -> anyhow::Result<Self> {
        blob.config.validate()?;
        let model = registry.build(&blob)?;

        let (control_tx, control_rx) = bounded(CONTROL_CAPACITY);
        let (intent_tx, intent_rx) = bounded(blob.config.run.max_pending_intents);
        let published = Arc::new(ArcSwap::from_pointee(Published::empty(
            blob.config.clone(),
            &blob,
        )));

        let publisher = Publisher::spawn(Arc::clone(&published))?;
        let mut history = History::spawn(registry.clone())?;
        history.begin(&blob, model.as_ref(), 0);
        let handle = DriverHandle {
            control: control_tx,
            intents: intent_tx,
            published: Arc::clone(&published),
            history: history.handle(),
        };
        let mut loop_state = DriverLoop::new(
            registry, blob, model, control_rx, intent_rx, published, publisher, history,
        );
        // Publish before the thread exists, so a caller that reads the view on
        // the line after `spawn` sees the seeded world rather than an empty one.
        loop_state.publish();

        let join = std::thread::Builder::new()
            .name("sim-driver".to_owned())
            .spawn(move || loop_state.run())?;

        Ok(Driver {
            handle,
            join: Some(join),
        })
    }

    pub fn handle(&self) -> DriverHandle {
        self.handle.clone()
    }

    /// Ask the driver to stop and wait for the thread.
    pub fn shutdown(mut self) {
        self.stop();
    }

    fn stop(&mut self) {
        let _ = self.handle.send(Control::Shutdown);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

impl Drop for Driver {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Everything that lives on the tick thread.
struct DriverLoop {
    registry: ModelRegistry,
    blob: ScenarioBlob,
    model: Box<dyn Model>,
    control: Receiver<Command>,
    intents: Receiver<Submission>,
    published: Arc<ArcSwap<Published>>,
    publisher: Publisher,
    history: History,

    mode: RunMode,
    tick: u64,
    steps_remaining: u64,
    config: Config,
    fault: Option<String>,
    last_good_tick: u64,

    admission: Admission,
    pending: std::collections::VecDeque<Submission>,
    intent_log: Vec<sim_core::intent::LoggedIntent>,
    intent_log_overflowed: bool,
    recent: std::collections::VecDeque<IntentRecord>,
    reports: std::collections::VecDeque<Report>,
    report_scratch: Vec<Report>,
    /// Never reset, not even by a scenario load, so consumers can tell new
    /// reports from old ones across a reload.
    report_seq: u64,

    metrics: Metrics,
    next_publish: Instant,
    next_tick_at: Option<Instant>,
    window_start: Instant,
    window_ticks: u64,
    window_publishes: u64,
}

impl DriverLoop {
    #[allow(clippy::too_many_arguments)]
    fn new(
        registry: ModelRegistry,
        blob: ScenarioBlob,
        model: Box<dyn Model>,
        control: Receiver<Command>,
        intents: Receiver<Submission>,
        published: Arc<ArcSwap<Published>>,
        publisher: Publisher,
        history: History,
    ) -> Self {
        let config = blob.config.clone();
        DriverLoop {
            admission: Admission::new(&config),
            registry,
            blob,
            model,
            control,
            intents,
            published,
            publisher,
            history,
            mode: RunMode::Paused,
            tick: 0,
            steps_remaining: 0,
            config,
            fault: None,
            last_good_tick: 0,
            pending: std::collections::VecDeque::new(),
            intent_log: Vec::new(),
            intent_log_overflowed: false,
            recent: std::collections::VecDeque::new(),
            reports: std::collections::VecDeque::new(),
            report_scratch: Vec::new(),
            report_seq: 0,
            metrics: Metrics::default(),
            next_publish: Instant::now(),
            next_tick_at: None,
            window_start: Instant::now(),
            window_ticks: 0,
            window_publishes: 0,
        }
    }

    fn run(&mut self) {
        loop {
            if !self.handle_control() {
                break;
            }
            self.drain_intents();

            if self.mode.is_advancing() {
                self.advance();
            }

            if Instant::now() >= self.next_publish {
                self.publish_in_background();
            }

            if !self.mode.is_advancing() && !self.idle() {
                break;
            }
        }
        self.publish_final();
    }

    /// Returns false when the driver has been told to stop.
    fn handle_control(&mut self) -> bool {
        while let Ok(command) = self.control.try_recv() {
            if !self.apply_command(command) {
                return false;
            }
        }
        true
    }

    fn apply_command(&mut self, command: Command) -> bool {
        match &command.control {
            Control::Shutdown => return false,
            Control::Start { target_tps } => {
                if self.mode == RunMode::Faulted {
                    command.reply(Err(
                        "the driver is faulted; reset before starting again".to_owned()
                    ));
                    return true;
                }
                self.config.run.target_tps = *target_tps;
                self.mode = RunMode::Running;
                self.next_tick_at = None;
                self.reset_rate_window();
                // Publish before acking, so that a caller which reads the view
                // straight after a successful command sees the state it just
                // asked for rather than the last frame from before it.
                self.publish();
                command.reply(Ok(()));
            }
            Control::Pause => {
                if self.mode != RunMode::Faulted {
                    self.mode = RunMode::Paused;
                    self.steps_remaining = 0;
                }
                self.publish();
                command.reply(Ok(()));
            }
            Control::Step { n } => {
                if self.mode == RunMode::Faulted {
                    command.reply(Err("the driver is faulted; reset first".to_owned()));
                    return true;
                }
                self.steps_remaining = (*n).max(1);
                self.mode = RunMode::Stepping;
                self.next_tick_at = None;
                self.reset_rate_window();
                self.publish();
                command.reply(Ok(()));
            }
            Control::SetRate(target_tps) => {
                self.config.run.target_tps = *target_tps;
                self.next_tick_at = None;
                self.publish();
                command.reply(Ok(()));
            }
            Control::Reset => {
                let blob = self.blob.clone();
                command.reply(self.load(blob).map_err(|e| e.to_string()));
            }
            Control::Load(blob) => {
                let blob = (**blob).clone();
                command.reply(self.load(blob).map_err(|e| e.to_string()));
            }
            Control::SetConfig(config) => {
                command.reply(
                    self.adopt_config((**config).clone())
                        .map_err(|e| e.to_string()),
                );
            }
            Control::Restore(checkpoint) => {
                let checkpoint = (**checkpoint).clone();
                command.reply(self.restore(checkpoint).map_err(|e| e.to_string()));
            }
            Control::Snapshot(reply) => {
                let checkpoint = self.build_checkpoint();
                let _ = reply.try_send(Box::new(checkpoint));
                command.reply(Ok(()));
            }
        }
        true
    }

    fn load(&mut self, blob: ScenarioBlob) -> anyhow::Result<()> {
        blob.config.validate()?;
        self.mode = RunMode::Loading;
        self.publish();

        let model = self.registry.build(&blob)?;
        self.model = model;
        self.config = blob.config.clone();
        self.blob = blob;
        self.admission = Admission::new(&self.config);
        self.tick = 0;
        self.last_good_tick = 0;
        self.fault = None;
        self.steps_remaining = 0;
        self.discard_pending("the world was rebuilt: a reset, a scenario load or a config change");
        self.intent_log.clear();
        self.intent_log_overflowed = false;
        // The old world's events mean nothing in the new one. `report_seq`
        // keeps counting so consumers never mistake a new report for a seen one.
        self.reports.clear();
        self.metrics = Metrics::default();
        self.publisher.reset_dropped();
        self.history.begin(&self.blob, self.model.as_ref(), 0);
        self.mode = RunMode::Paused;
        self.publish();
        Ok(())
    }

    /// Adopt a config. Structural changes rebuild the world, because they
    /// change what a cell id means (spec §10.2).
    fn adopt_config(&mut self, config: Config) -> anyhow::Result<()> {
        config.validate()?;
        let structural = !self.config.changes_requiring_reset(&config).is_empty();
        if structural {
            let mut blob = self.blob.clone();
            blob.config = config;
            self.load(blob)
        } else {
            self.config = config.clone();
            self.blob.config = config;
            self.admission = Admission::new(&self.config);
            self.publish();
            Ok(())
        }
    }

    fn restore(&mut self, checkpoint: Checkpoint) -> anyhow::Result<()> {
        if self.mode.is_advancing() {
            anyhow::bail!("pause the driver before restoring a checkpoint");
        }
        checkpoint.config.validate()?;
        self.mode = RunMode::Loading;
        self.publish();

        // The checkpoint's cells are the agents, so the world is built without
        // the pack's seed. The stored scenario keeps its seed, though: a reset
        // or a rebuild later starts the scenario over from it, under the
        // restored config.
        let mut blob = self.blob.clone();
        blob.config = checkpoint.config.clone();
        let mut empty = blob.clone();
        empty.agents.clear();
        let mut model = self.registry.build(&empty)?;
        model.load_cells(checkpoint.cells.clone())?;
        model.restore_fields(&checkpoint.fields)?;
        model.set_rng_state(checkpoint.rng);
        model.set_tick(checkpoint.meta.tick);

        self.model = model;
        self.config = checkpoint.config;
        self.blob = blob;
        self.admission = Admission::new(&self.config);
        self.tick = checkpoint.meta.tick;
        self.last_good_tick = self.tick;
        self.fault = None;
        self.discard_pending("a checkpoint was restored");
        self.intent_log = checkpoint.intent_log.unwrap_or_default();
        self.reports.clear();
        self.history
            .begin(&self.blob, self.model.as_ref(), self.tick);
        self.mode = RunMode::Paused;
        self.publish();
        Ok(())
    }

    /// The world is being replaced: answer every queued intent, and every one
    /// still on the channel, as discarded. Left alone they would vanish
    /// unanswered (a ground station would think its mission was taken) or,
    /// still on the channel, land in a world they were not meant for.
    fn discard_pending(&mut self, why: &str) {
        let reason = RejectReason::Discarded {
            why: why.to_owned(),
        };
        let mut gone: Vec<Submission> = self.pending.drain(..).collect();
        while let Ok(submission) = self.intents.try_recv() {
            gone.push(submission);
        }
        for submission in gone {
            self.note_intent(&submission.intent, Err(&reason));
            submission.resolve(IntentAck::Rejected(reason.clone()));
        }
        self.metrics.intent_queue_depth = 0;
    }

    /// Move submissions off the channel and into the pending queue, applying
    /// the admission rules that do not need the model (spec §8.2).
    fn drain_intents(&mut self) {
        while self.pending.len() < self.config.run.max_pending_intents {
            let Ok(submission) = self.intents.try_recv() else {
                break;
            };
            match self.admission.admit(&submission.intent, self.tick) {
                AdmissionOutcome::Accept => self.pending.push_back(submission),
                AdmissionOutcome::Reject(reason) => {
                    self.note_intent(&submission.intent, Err(&reason));
                    submission.resolve(IntentAck::Rejected(reason));
                }
            }
        }
        self.metrics.intent_queue_depth = self.pending.len() as u64;
    }

    fn advance(&mut self) {
        if let Some(due) = self.next_tick_at {
            if Instant::now() < due {
                // Rate-limited: give the time back rather than spinning.
                std::thread::sleep(due.saturating_duration_since(Instant::now()).min(IDLE_POLL));
                return;
            }
        }

        let target_tick = self.tick + 1;
        self.apply_due_intents(target_tick);

        let started = Instant::now();
        let model = &mut self.model;
        let outcome = catch_unwind(AssertUnwindSafe(|| model.step(target_tick)));
        self.metrics.last_step_ns = started.elapsed().as_nanos() as u64;

        match outcome {
            Ok(()) => {
                self.tick = target_tick;
                self.last_good_tick = target_tick;
                self.history.ticked(self.model.as_ref(), target_tick);
                self.collect_reports();
                self.window_ticks += 1;
                if self.mode == RunMode::Stepping {
                    self.steps_remaining -= 1;
                    if self.steps_remaining == 0 {
                        self.mode = RunMode::Paused;
                        // A single step is an operator action: show its result
                        // now rather than at the next publish deadline.
                        self.publish();
                    }
                }
                if let Some(interval) = self.config.tick_interval() {
                    // Anchor on the previous deadline so the rate does not drift.
                    let base = self.next_tick_at.unwrap_or(started);
                    let next = base + interval;
                    self.next_tick_at = Some(next.max(Instant::now() - interval));
                } else {
                    self.next_tick_at = None;
                }
            }
            Err(payload) => {
                self.mode = RunMode::Faulted;
                self.steps_remaining = 0;
                self.fault = Some(describe_panic(payload));
                tracing::error!(
                    tick = target_tick,
                    fault = self.fault.as_deref().unwrap_or(""),
                    "sim step panicked; driver is frozen at the last good tick"
                );
                self.publish();
                if self.config.run.auto_recover {
                    let blob = self.blob.clone();
                    if let Err(err) = self.load(blob) {
                        tracing::error!(%err, "auto-recovery failed; operator reset required");
                    }
                }
            }
        }
    }

    /// Hand the model every intent that is due on `tick`, up to the per-tick cap.
    fn apply_due_intents(&mut self, tick: u64) {
        let mut applied = 0usize;
        let mut deferred = std::collections::VecDeque::new();

        while let Some(submission) = self.pending.pop_front() {
            if applied >= self.config.run.max_applied_per_tick {
                deferred.push_back(submission);
                continue;
            }
            if submission.intent.tick.is_some_and(|t| t > tick) {
                deferred.push_back(submission);
                continue;
            }
            match self.model.apply_intent(&submission.intent) {
                Ok(()) => {
                    applied += 1;
                    self.metrics.intents_applied += 1;
                    if self.config.run.profile == Profile::Gold {
                        self.log_intent(tick, &submission.intent);
                    }
                    self.history.applied(tick, &submission.intent);
                    self.note_intent(&submission.intent, Ok(tick));
                    submission.resolve(IntentAck::Applied { tick });
                }
                Err(reason) => {
                    self.metrics.intents_rejected += 1;
                    self.note_intent(&submission.intent, Err(&reason));
                    submission.resolve(IntentAck::Rejected(reason));
                }
            }
        }

        self.pending = deferred;
        self.metrics.intent_queue_depth = self.pending.len() as u64;
    }

    fn log_intent(&mut self, tick: u64, intent: &Intent) {
        if self.intent_log.len() >= INTENT_LOG_CAP {
            if !self.intent_log_overflowed {
                self.intent_log_overflowed = true;
                tracing::warn!(
                    cap = INTENT_LOG_CAP,
                    "intent log is full; gold replay from this run will be incomplete"
                );
            }
            return;
        }
        self.intent_log.push(sim_core::intent::LoggedIntent {
            tick,
            intent: intent.clone(),
        });
    }

    fn collect_reports(&mut self) {
        self.model.drain_reports(&mut self.report_scratch);
        for mut report in self.report_scratch.drain(..) {
            self.report_seq += 1;
            report.seq = self.report_seq;
            if self.reports.len() >= RECENT_REPORTS {
                self.reports.pop_front();
            }
            self.reports.push_back(report);
        }
    }

    fn note_intent(&mut self, intent: &Intent, result: Result<u64, &RejectReason>) {
        if self.recent.len() >= RECENT_INTENTS {
            self.recent.pop_front();
        }
        let (rejected, detail, tick) = match result {
            Ok(tick) => (None, "applied".to_owned(), tick),
            Err(reason) => (
                Some(reason.code().to_owned()),
                reason.to_string(),
                self.tick,
            ),
        };
        self.recent.push_back(IntentRecord {
            id: intent.id,
            actor: intent.actor.to_string(),
            verb: format!("{:?}", intent.verb),
            cell: intent.cell,
            tick,
            rejected,
            detail,
        });
    }

    fn build_checkpoint(&mut self) -> Checkpoint {
        let mut cells = Vec::new();
        self.model.snapshot(&mut cells, usize::MAX);
        let gold = self.config.run.profile == Profile::Gold;
        Checkpoint {
            meta: SnapshotMeta {
                scenario_id: self.blob.manifest.id.clone(),
                tick: self.tick,
                profile: self.config.run.profile,
                root: self.config.grid.root,
                resolution: self.config.grid.resolution,
                agent_count: self.model.agent_count(),
                occupied_cells: self.model.occupied_cells(),
                hash: gold.then(|| sim_core::snapshot::stable_hash(&cells)),
            },
            config: self.config.clone(),
            cells,
            fields: self.model.fields(),
            rng: self.model.rng_state(),
            intent_log: gold.then(|| self.intent_log.clone()),
        }
    }

    /// Publish the world as it is now, and return once readers can see it.
    /// Operator actions publish this way before they ack.
    fn publish(&mut self) {
        let (view, cells, limit) = self.next_view();
        self.publisher.publish_now(view, cells, limit);
    }

    /// Publish the world as it is now without waiting for the view to be
    /// built. The regular publish while running.
    fn publish_in_background(&mut self) {
        let (view, cells, limit) = self.next_view();
        self.publisher.submit(view, cells, limit);
    }

    /// Everything a view needs from the tick thread. The cells come as a frame
    /// to build elsewhere when the model has one, and are written here when
    /// it does not.
    fn next_view(&mut self) -> (Published, Cells, usize) {
        let limit = self.config.run.max_published_cells;
        let cells = match self.model.frame() {
            Some(frame) => Cells::Frame(frame),
            None => {
                let mut cells = Vec::new();
                let truncation = self.model.snapshot(&mut cells, limit);
                Cells::Ready(cells, truncation)
            }
        };

        // Rates are recomputed on every publish so a freshly started run shows
        // a real number immediately, and the window is rolled once a second has
        // half passed so the figure keeps tracking the present.
        self.window_publishes += 1;
        let elapsed = self.window_start.elapsed().as_secs_f64();
        if elapsed >= 0.001 {
            self.metrics.sim_tps = self.window_ticks as f64 / elapsed;
            self.metrics.publish_hz = self.window_publishes as f64 / elapsed;
        }
        if elapsed >= 0.5 {
            self.reset_rate_window();
        }

        self.metrics.tick = self.tick;
        self.metrics.agent_count = self.model.agent_count();
        self.metrics.occupied_cells = self.model.occupied_cells();
        self.next_publish = Instant::now() + self.config.publish_interval();

        let view = Published {
            meta: SnapshotMeta {
                scenario_id: self.blob.manifest.id.clone(),
                tick: self.tick,
                profile: self.config.run.profile,
                root: self.config.grid.root,
                resolution: self.config.grid.resolution,
                agent_count: self.metrics.agent_count,
                occupied_cells: self.metrics.occupied_cells,
                // The publisher hashes the cells once they are written.
                hash: None,
            },
            metrics: self.metrics,
            health: DriverHealth {
                alive: true,
                mode: self.mode,
                last_publish_age_ms: 0,
                fault: self.fault.clone(),
                last_good_tick: self.last_good_tick,
            },
            config: self.config.clone(),
            scenario_id: self.blob.manifest.id.clone(),
            scenario_name: self.blob.manifest.name.clone(),
            model_kind: self.blob.manifest.model.clone(),
            cells: Vec::new(),
            truncated: false,
            published_at_ms: now_ms(),
            recent_intents: self.recent.iter().cloned().collect(),
            reports: self.reports.iter().cloned().collect(),
            overlays: self.model.overlays(),
        };
        (view, cells, limit)
    }

    /// Start measuring the tick rate afresh. Called when a run starts, so the
    /// first reading is not diluted by however long the driver sat paused.
    fn reset_rate_window(&mut self) {
        self.window_start = Instant::now();
        self.window_ticks = 0;
        self.window_publishes = 0;
    }

    /// Mark the view dead so the HTTP edge answers 503 rather than serving a
    /// snapshot that will never advance again (spec §12).
    fn publish_final(&mut self) {
        // Let the last view in flight land first, so this one is not overwritten.
        self.publisher.stop();
        self.history.stop();
        let mut view = (**self.published.load()).clone();
        view.health.alive = false;
        view.health.mode = RunMode::Paused;
        view.published_at_ms = now_ms();
        self.published.store(Arc::new(view));
    }

    /// Wait for something to happen. A paused driver must not burn a core.
    ///
    /// Returns false once shutdown has been requested. The wait is capped at
    /// the next publish deadline so `/health` keeps ticking over while paused.
    fn idle(&mut self) -> bool {
        let wait = self
            .next_publish
            .saturating_duration_since(Instant::now())
            .min(IDLE_POLL);
        // Stop taking intents once the pending queue is full, so the channel
        // fills and `submit` pushes back instead of the queue growing unbounded.
        let never = crossbeam_channel::never();
        let intents = if self.pending.len() < self.config.run.max_pending_intents {
            &self.intents
        } else {
            &never
        };
        crossbeam_channel::select! {
            recv(self.control) -> command => {
                match command {
                    // Route through the single handler so acks and faults
                    // behave the same whether the driver was busy or idle.
                    Ok(command) => self.apply_command(command),
                    Err(_) => false,
                }
            }
            recv(intents) -> submission => {
                if let Ok(submission) = submission {
                    match self.admission.admit(&submission.intent, self.tick) {
                        AdmissionOutcome::Accept => self.pending.push_back(submission),
                        AdmissionOutcome::Reject(reason) => {
                            self.note_intent(&submission.intent, Err(&reason));
                            submission.resolve(IntentAck::Rejected(reason));
                        }
                    }
                }
                true
            }
            default(wait) => true,
        }
    }
}

fn describe_panic(payload: Box<dyn std::any::Any + Send>) -> String {
    if let Some(message) = payload.downcast_ref::<&'static str>() {
        (*message).to_owned()
    } else if let Some(message) = payload.downcast_ref::<String>() {
        message.clone()
    } else {
        "panic with a non-string payload".to_owned()
    }
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}
