//! Building published views off the tick thread.
//!
//! Turning the committed world into a [`Published`] means writing every
//! agent's state out as JSON and, for a gold run, hashing the lot:
//! milliseconds for a few thousand agents, twenty times a second. The tick
//! thread hands that to this thread and goes straight back to ticking.
//!
//! There is one slot, not a queue. A view replaces one still waiting, so a
//! publisher that falls behind skips views (they count as dropped) rather
//! than queueing them, and views are stored in the order they were taken. A
//! view an operator action waits for is never replaced: the tick thread does
//! nothing else until it is stored.

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;

use arc_swap::ArcSwap;
use crossbeam_channel::{bounded, Sender};
use sim_core::{CellBag, Profile};

use crate::driver::Published;
use crate::model::{Frame, Truncation};

/// Where a view's cells come from.
pub(crate) enum Cells {
    /// Already written, on the tick thread.
    Ready(Vec<CellBag>, Truncation),
    /// Written here, from a frame the model handed over.
    Frame(Box<dyn Frame>),
}

struct Job {
    /// Everything but the cells, what the cap left out, and the hash.
    view: Published,
    cells: Cells,
    limit: usize,
    done: Option<Sender<()>>,
}

#[derive(Default)]
struct Slot {
    job: Option<Job>,
    stopping: bool,
    /// Views nobody read: replaced in the slot before they were built, or
    /// stored and replaced before any reader loaded them.
    dropped: u64,
}

struct Shared {
    slot: Mutex<Slot>,
    wake: Condvar,
}

fn lock(shared: &Shared) -> MutexGuard<'_, Slot> {
    shared.slot.lock().unwrap_or_else(PoisonError::into_inner)
}

pub(crate) struct Publisher {
    shared: Arc<Shared>,
    join: Option<JoinHandle<()>>,
}

impl Publisher {
    pub(crate) fn spawn(published: Arc<ArcSwap<Published>>) -> std::io::Result<Publisher> {
        let shared = Arc::new(Shared {
            slot: Mutex::new(Slot::default()),
            wake: Condvar::new(),
        });
        let theirs = Arc::clone(&shared);
        let join = std::thread::Builder::new()
            .name("sim-publisher".to_owned())
            .spawn(move || run(&theirs, &published))?;
        Ok(Publisher {
            shared,
            join: Some(join),
        })
    }

    /// Hand a view over and carry on.
    pub(crate) fn submit(&self, view: Published, cells: Cells, limit: usize) {
        self.put(Job {
            view,
            cells,
            limit,
            done: None,
        });
    }

    /// Hand a view over and wait until readers can load it.
    pub(crate) fn publish_now(&self, view: Published, cells: Cells, limit: usize) {
        let (done, stored) = bounded(1);
        self.put(Job {
            view,
            cells,
            limit,
            done: Some(done),
        });
        // An error means the publisher thread is gone and nothing will be
        // stored; waiting longer would not change that.
        let _ = stored.recv();
    }

    fn put(&self, job: Job) {
        let mut slot = lock(&self.shared);
        if slot.job.replace(job).is_some() {
            slot.dropped += 1;
        }
        self.shared.wake.notify_one();
    }

    /// Count dropped views from zero again, as a fresh world does.
    pub(crate) fn reset_dropped(&self) {
        lock(&self.shared).dropped = 0;
    }

    /// Store whatever is waiting, then end the thread.
    pub(crate) fn stop(&mut self) {
        lock(&self.shared).stopping = true;
        self.shared.wake.notify_one();
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

impl Drop for Publisher {
    fn drop(&mut self) {
        self.stop();
    }
}

fn run(shared: &Shared, published: &ArcSwap<Published>) {
    loop {
        let job = {
            let mut slot = lock(shared);
            loop {
                if let Some(job) = slot.job.take() {
                    break job;
                }
                if slot.stopping {
                    return;
                }
                slot = shared
                    .wake
                    .wait(slot)
                    .unwrap_or_else(PoisonError::into_inner);
            }
        };
        let Job {
            mut view,
            cells,
            limit,
            done,
        } = job;

        let built = catch_unwind(AssertUnwindSafe(|| match cells {
            Cells::Ready(cells, truncation) => (cells, truncation),
            Cells::Frame(frame) => {
                let mut cells = Vec::new();
                let truncation = frame.snapshot(&mut cells, limit);
                (cells, truncation)
            }
        }));
        match built {
            Ok((cells, truncation)) => {
                let unread = Arc::strong_count(&published.load()) <= 1;
                let dropped = {
                    let mut slot = lock(shared);
                    slot.dropped += u64::from(unread);
                    slot.dropped
                };
                view.metrics.dropped_views = dropped;
                view.metrics.truncated_cells = truncation.cells_omitted;
                view.truncated = truncation.is_truncated();
                view.meta.hash = (view.meta.profile == Profile::Gold && !truncation.is_truncated())
                    .then(|| sim_core::snapshot::stable_hash(&cells));
                view.cells = cells;
                published.store(Arc::new(view));
            }
            Err(_) => {
                // Keep the last good view rather than publish a broken one; the
                // next frame gets another go.
                tracing::error!(tick = view.meta.tick, "building a published view panicked");
            }
        }
        if let Some(done) = done {
            let _ = done.send(());
        }
    }
}
