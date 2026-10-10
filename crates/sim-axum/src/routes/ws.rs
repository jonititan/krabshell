//! Live subscriptions (spec §9.3).
//!
//! One task per connection. It owns that client's area of interest and the last
//! frame it sent, so deltas are computed per client, off the tick thread. The
//! driver is never aware of how many sockets are open.
//!
//! The publish budget is enforced by construction: the task wakes on a timer at
//! `publish_hz` and reads whatever the driver has published by then. A world
//! running at 400 000 ticks a second and a browser reading at 20 Hz simply see
//! different frames; neither waits for the other.

use std::collections::BTreeSet;

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::State;
use axum::response::Response;
use futures_util::{SinkExt, StreamExt};
use sim_core::wire::{diff, ClientMsg, ServerMsg, PROTOCOL_VERSION};
use sim_core::{CellBag, ErrorCode, GeoCell, RejectReason};
use sim_engine::{IntentAck, Submission};

use crate::AppState;

/// Largest area of interest one connection may hold.
const MAX_AOI_CELLS: usize = 20_000;

/// Largest `k` accepted for a disk AOI.
const MAX_AOI_K: u32 = 64;

/// Most intents one connection may have waiting for the driver's answer.
/// More are refused as backpressured until some are answered: each waits
/// for a tick, and a paused world has none.
const MAX_WAITING: usize = 1_000;

/// Intents sent to the driver and not yet answered, with the channel each
/// answer will come on.
type Waiting = Vec<(uuid::Uuid, crossbeam_channel::Receiver<IntentAck>)>;

/// Largest message a client may send. A full 20 000-cell subscription is
/// under half of this; an intent is capped far lower below.
const MAX_MESSAGE_BYTES: usize = 1024 * 1024;

pub async fn upgrade(ws: WebSocketUpgrade, State(state): State<AppState>) -> Response {
    ws.max_message_size(MAX_MESSAGE_BYTES)
        .max_frame_size(MAX_MESSAGE_BYTES)
        .on_upgrade(move |socket| serve(socket, state))
}

async fn serve(socket: WebSocket, state: AppState) {
    let (mut sink, mut stream) = socket.split();

    let view = state.driver.view();
    if send(
        &mut sink,
        ServerMsg::Hello {
            tick: view.meta.tick,
            meta: view.meta.clone(),
            protocol: PROTOCOL_VERSION,
        },
    )
    .await
    .is_err()
    {
        return;
    }

    // Intent acknowledgements come back on the driver's channel type, which is
    // synchronous. Polling the channels on the publish timer answers within a
    // frame without parking a thread per intent: a paused world answers none,
    // and a thread each would soon use up the blocking pool every control
    // route needs.
    let mut waiting: Waiting = Vec::new();

    let mut aoi: BTreeSet<GeoCell> = BTreeSet::new();
    let mut last: Vec<CellBag> = Vec::new();
    let mut resend_everything = false;

    let period = state.driver.view().config.publish_interval();
    let mut ticker = tokio::time::interval(period);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    loop {
        tokio::select! {
            incoming = stream.next() => {
                let Some(Ok(message)) = incoming else { break };
                match message {
                    Message::Text(text) => {
                        match serde_json::from_str::<ClientMsg>(&text) {
                            Ok(msg) => {
                                if handle(msg, &state, &mut aoi, &mut resend_everything, &mut waiting, &mut sink)
                                    .await
                                    .is_err()
                                {
                                    break;
                                }
                            }
                            Err(err) => {
                                let _ = send(
                                    &mut sink,
                                    ServerMsg::Error {
                                        code: ErrorCode::BadRequest,
                                        message: format!("could not parse message: {err}"),
                                    },
                                )
                                .await;
                            }
                        }
                    }
                    Message::Close(_) => break,
                    // `Bytes` clones by reference count, not by copying the payload.
                    Message::Ping(payload) if sink.send(Message::Pong(payload.clone())).await.is_err() => break,
                    _ => {}
                }
            }

            _ = ticker.tick() => {
                // Answer every intent the driver has decided.
                for msg in answered(&mut waiting) {
                    if send(&mut sink, msg).await.is_err() {
                        return;
                    }
                }

                let view = state.driver.view();

                if !aoi.is_empty() {
                    let current: Vec<CellBag> = aoi
                        .iter()
                        .filter_map(|cell| view.bag(*cell).cloned())
                        .collect();

                    let frame = if resend_everything {
                        resend_everything = false;
                        Some(ServerMsg::Snapshot {
                            tick: view.meta.tick,
                            cells: current.clone(),
                        })
                    } else {
                        let (upsert, remove) = diff(&last, &current);
                        (!upsert.is_empty() || !remove.is_empty()).then(|| ServerMsg::Delta {
                            tick: view.meta.tick,
                            upsert,
                            remove,
                        })
                    };

                    last = current;
                    if let Some(frame) = frame {
                        if send(&mut sink, frame).await.is_err() {
                            return;
                        }
                    }
                }

                let heartbeat = ServerMsg::Tick {
                    metrics: view.metrics,
                };
                if send(&mut sink, heartbeat).await.is_err() {
                    return;
                }
            }
        }
    }
}

type Sink = futures_util::stream::SplitSink<WebSocket, Message>;

fn too_many_cells() -> ServerMsg {
    ServerMsg::Error {
        code: ErrorCode::BadRequest,
        message: format!(
            "an area of interest is capped at {MAX_AOI_CELLS} cells; \
             subscribe to a coarser area or fewer cells"
        ),
    }
}

/// Take the decided intents out of `waiting`, as the messages to send. An
/// answer channel the driver dropped means the intent was thrown away
/// unanswered, which the client hears as a rejection rather than nothing.
fn answered(waiting: &mut Waiting) -> Vec<ServerMsg> {
    let mut out = Vec::new();
    waiting.retain(|(intent_id, rx)| {
        let intent_id = *intent_id;
        match rx.try_recv() {
            Ok(IntentAck::Applied { .. }) => out.push(ServerMsg::Accepted { intent_id }),
            Ok(IntentAck::Rejected(reason)) => out.push(ServerMsg::Rejected { intent_id, reason }),
            Err(crossbeam_channel::TryRecvError::Empty) => return true,
            Err(crossbeam_channel::TryRecvError::Disconnected) => out.push(ServerMsg::Rejected {
                intent_id,
                reason: RejectReason::Discarded {
                    why: "the driver dropped it unanswered".into(),
                },
            }),
        }
        false
    });
    out
}

async fn handle(
    msg: ClientMsg,
    state: &AppState,
    aoi: &mut BTreeSet<GeoCell>,
    resend_everything: &mut bool,
    waiting: &mut Waiting,
    sink: &mut Sink,
) -> Result<(), axum::Error> {
    match msg {
        ClientMsg::Subscribe { cells, res } => {
            let world_resolution = state.driver.view().meta.resolution;
            // Count before expanding: one resolution-0 cell has about a
            // billion descendants at resolution 15.
            let wanted = cells.iter().fold(0u64, |sum, cell| {
                let n = if res.is_some() {
                    cell.children_count(world_resolution)
                } else {
                    1
                };
                sum.saturating_add(n)
            });
            if (aoi.len() as u64).saturating_add(wanted) > MAX_AOI_CELLS as u64 {
                return send(sink, too_many_cells()).await;
            }
            let mut added = Vec::new();
            for cell in cells {
                // A coarse cell subscribes to everything under it, so a client
                // can say "this parent" without enumerating its children.
                if res.is_some() && cell.resolution() < world_resolution {
                    match cell.children(Some(world_resolution)) {
                        Ok(children) => added.extend(children),
                        Err(err) => {
                            return send(
                                sink,
                                ServerMsg::Error {
                                    code: ErrorCode::BadCell,
                                    message: err.to_string(),
                                },
                            )
                            .await
                        }
                    }
                } else {
                    added.push(cell);
                }
            }
            aoi.extend(added);
            *resend_everything = true;
            Ok(())
        }

        ClientMsg::Unsubscribe { cells } => {
            for cell in cells {
                aoi.remove(&cell);
            }
            *resend_everything = true;
            Ok(())
        }

        ClientMsg::SetAoi { center, k, res: _ } => {
            let k = k.min(MAX_AOI_K) as usize;
            match center.grid_disk(k) {
                Ok(disk) if disk.len() <= MAX_AOI_CELLS => {
                    *aoi = disk.into_iter().collect();
                    *resend_everything = true;
                    Ok(())
                }
                Ok(disk) => {
                    send(
                        sink,
                        ServerMsg::Error {
                            code: ErrorCode::BadRequest,
                            message: format!(
                                "a disk of k={k} is {} cells, over the {MAX_AOI_CELLS} cap",
                                disk.len()
                            ),
                        },
                    )
                    .await
                }
                Err(err) => {
                    send(
                        sink,
                        ServerMsg::Error {
                            code: ErrorCode::BadCell,
                            message: err.to_string(),
                        },
                    )
                    .await
                }
            }
        }

        ClientMsg::Intent { intent } => {
            let intent_id = intent.id;
            // The same cap as `POST /intents`.
            let size = serde_json::to_vec(&intent).map_or(usize::MAX, |b| b.len());
            if size > crate::MAX_INTENT_BYTES {
                return send(
                    sink,
                    ServerMsg::Error {
                        code: ErrorCode::PayloadTooLarge,
                        message: format!(
                            "intent {intent_id} is {size} bytes; the limit is {}",
                            crate::MAX_INTENT_BYTES
                        ),
                    },
                )
                .await;
            }
            let view = state.driver.view();

            // Address checks happen here so a misaddressed intent never takes a
            // queue slot from a valid one (spec §8.2).
            if sim_engine::check_intent_cell(&view.meta, intent.cell).is_err() {
                return send(
                    sink,
                    ServerMsg::Rejected {
                        intent_id,
                        reason: RejectReason::OutOfBounds,
                    },
                )
                .await;
            }

            if !state
                .limiter
                .allow(&intent.actor.0, view.config.run.intent_rate_per_actor)
            {
                return send(
                    sink,
                    ServerMsg::Rejected {
                        intent_id,
                        reason: RejectReason::RateLimited,
                    },
                )
                .await;
            }

            if waiting.len() >= MAX_WAITING {
                return send(
                    sink,
                    ServerMsg::Rejected {
                        intent_id,
                        reason: RejectReason::Backpressured {
                            pending: waiting.len(),
                        },
                    },
                )
                .await;
            }

            let (tx, rx) = crossbeam_channel::bounded(1);
            let outcome = state.driver.submit(Submission {
                intent,
                ack: Some(tx),
            });
            match outcome {
                Ok(()) => {
                    // Answered on the publish timer; see `answered`.
                    waiting.push((intent_id, rx));
                    Ok(())
                }
                Err(reason) => send(sink, ServerMsg::Rejected { intent_id, reason }).await,
            }
        }

        ClientMsg::Ping => send(sink, ServerMsg::Pong).await,
    }
}

async fn send(sink: &mut Sink, msg: ServerMsg) -> Result<(), axum::Error> {
    let text = match serde_json::to_string(&msg) {
        Ok(text) => text,
        Err(err) => {
            tracing::error!(%err, "could not encode a server message");
            return Ok(());
        }
    };
    sink.send(Message::Text(text.into())).await
}
