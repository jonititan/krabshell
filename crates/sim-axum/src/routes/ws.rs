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

pub async fn upgrade(ws: WebSocketUpgrade, State(state): State<AppState>) -> Response {
    ws.on_upgrade(move |socket| serve(socket, state))
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
    // synchronous. Draining it on the publish timer keeps this task free of
    // blocking receives while still answering within one frame.
    let (ack_tx, ack_rx) = crossbeam_channel::unbounded::<(uuid::Uuid, IntentAck)>();

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
                                if handle(msg, &state, &mut aoi, &mut resend_everything, &ack_tx, &mut sink)
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
                // Answer every intent this connection is still waiting on.
                while let Ok((intent_id, ack)) = ack_rx.try_recv() {
                    let msg = match ack {
                        IntentAck::Applied { .. } => ServerMsg::Accepted { intent_id },
                        IntentAck::Rejected(reason) => ServerMsg::Rejected { intent_id, reason },
                    };
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
                    tick: view.meta.tick,
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

async fn handle(
    msg: ClientMsg,
    state: &AppState,
    aoi: &mut BTreeSet<GeoCell>,
    resend_everything: &mut bool,
    ack_tx: &crossbeam_channel::Sender<(uuid::Uuid, IntentAck)>,
    sink: &mut Sink,
) -> Result<(), axum::Error> {
    match msg {
        ClientMsg::Subscribe { cells, res } => {
            let world_resolution = state.driver.view().meta.resolution;
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
            if aoi.len() + added.len() > MAX_AOI_CELLS {
                return send(
                    sink,
                    ServerMsg::Error {
                        code: ErrorCode::BadRequest,
                        message: format!(
                            "an area of interest is capped at {MAX_AOI_CELLS} cells; \
                             subscribe to a coarser area or fewer cells"
                        ),
                    },
                )
                .await;
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

            let (tx, rx) = crossbeam_channel::bounded(1);
            let outcome = state.driver.submit(Submission {
                intent,
                ack: Some(tx),
            });
            match outcome {
                Ok(()) => {
                    // Forward whatever the driver decides onto this connection.
                    let ack_tx = ack_tx.clone();
                    tokio::task::spawn_blocking(move || {
                        if let Ok(ack) = rx.recv() {
                            let _ = ack_tx.send((intent_id, ack));
                        }
                    });
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
