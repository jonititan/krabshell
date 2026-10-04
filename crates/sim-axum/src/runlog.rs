//! Logs each run of a drones pack to a folder: where the drones flew and
//! what they delivered.
//!
//! A pack opts in by setting `heatmap_resolution` or
//! `order_rate_per_address_day` in its `params.toml`. While it is loaded, this
//! samples the published view (it never touches the simulation) into a
//! [`sim_engine::models::drones::runlog::RunLog`] and writes its files to
//! `<dir>/<scenario>-<UTC start>/` every `every`, and once more on shutdown.
//! A reset, a restored checkpoint or a different scenario finishes that
//! folder and starts a new one, so earlier runs are never overwritten.

use std::path::{Path, PathBuf};
use std::time::Duration;

use sim_engine::models::drones::runlog::{self, RunLog};
use sim_engine::DriverHandle;
use tokio::sync::watch;

#[derive(Debug, Clone)]
pub struct RunLogConfig {
    pub dir: PathBuf,
    /// How often the files are rewritten while the run goes on.
    pub every: Duration,
}

struct Active {
    log: RunLog,
    folder: PathBuf,
    /// What was last written, to skip rewriting unchanged files.
    written: Vec<(&'static str, String)>,
}

impl Active {
    async fn write(&mut self) {
        let files = self.log.files();
        if files.is_empty() || files == self.written {
            return;
        }
        if let Err(err) = tokio::fs::create_dir_all(&self.folder).await {
            tracing::warn!(%err, folder = %self.folder.display(), "could not create the run log folder");
            return;
        }
        for (name, contents) in &files {
            let path = self.folder.join(name);
            if let Err(err) = write_atomically(&path, contents.as_bytes()).await {
                tracing::warn!(%err, path = %path.display(), "could not write the run log");
            }
        }
        if self.log.missed_reports > 0 {
            tracing::warn!(
                missed = self.log.missed_reports,
                "the run log missed reports the driver had already dropped; deliveries may be undercounted"
            );
        }
        self.written = files;
    }
}

pub async fn run(
    handle: DriverHandle,
    cfg: RunLogConfig,
    mut shutdown: watch::Receiver<bool>,
) -> anyhow::Result<()> {
    // About as often as the driver publishes; the heatmap interpolates
    // between views, and reports are read from the driver's buffer.
    let mut sample = tokio::time::interval(Duration::from_millis(50));
    let mut flush = tokio::time::interval(cfg.every.max(Duration::from_secs(1)));
    sample.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    flush.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut active: Option<Active> = None;

    loop {
        tokio::select! {
            _ = sample.tick() => {
                let view = handle.view();
                let wanted = runlog::wanted(&view.config);
                let stale = active.as_ref().is_some_and(|a| {
                    !wanted
                        || a.log.scenario != view.scenario_id
                        || a.log.last_tick().is_some_and(|t| view.meta.tick < t)
                });
                if stale {
                    if let Some(mut done) = active.take() {
                        done.write().await;
                    }
                }
                if !wanted {
                    continue;
                }
                let a = active.get_or_insert_with(|| {
                    let folder = cfg.dir.join(format!("{}-{}", view.scenario_id, utc_stamp(now_secs())));
                    tracing::info!(folder = %folder.display(), "logging this run");
                    Active {
                        log: RunLog::new(&view.scenario_id, &view.config, &view),
                        folder,
                        written: Vec::new(),
                    }
                });
                a.log.observe(&view);
            }
            _ = flush.tick() => {
                if let Some(a) = active.as_mut() {
                    a.write().await;
                }
            }
            changed = shutdown.changed() => {
                if changed.is_err() || *shutdown.borrow() {
                    break;
                }
            }
        }
    }
    if let Some(mut a) = active {
        a.write().await;
    }
    Ok(())
}

async fn write_atomically(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    let tmp = path.with_extension("tmp");
    tokio::fs::write(&tmp, bytes).await?;
    tokio::fs::rename(&tmp, path).await
}

fn now_secs() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

/// `20261003T142233Z` for a Unix time, without a date library.
pub fn utc_stamp(secs: u64) -> String {
    let days = (secs / 86_400) as i64;
    let rem = secs % 86_400;
    // Howard Hinnant's civil-from-days.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}{month:02}{day:02}T{:02}{:02}{:02}Z",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60
    )
}

#[cfg(test)]
mod tests {
    use super::utc_stamp;

    #[test]
    fn utc_stamps_are_calendar_dates() {
        assert_eq!(utc_stamp(0), "19700101T000000Z");
        assert_eq!(utc_stamp(951_782_400), "20000229T000000Z");
        assert_eq!(utc_stamp(1_791_037_353), "20261003T142233Z");
    }
}
