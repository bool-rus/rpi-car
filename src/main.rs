use peripheral::DriverMessage;
use tokio::sync::mpsc::UnboundedSender;
use tower_http::trace::{TraceLayer, DefaultMakeSpan};

use axum::{
    extract::{ws::{Message, WebSocket, WebSocketUpgrade}, State},
    response::IntoResponse,
    routing::get,
    Router
};
use std::net::SocketAddr;

mod peripheral;

async fn handle_connection (mut socket: WebSocket, sender: UnboundedSender<DriverMessage>) {
    while let Some(msg) = socket.recv().await {
        if let Ok(msg) = msg {
            match msg {
                Message::Text(text) => {
                    tracing::error!("Unknown message: {text}");
                },
                Message::Binary(data) => {
                    let moving = i8::from_be_bytes([data[0]]);
                    let turning = i8::from_be_bytes([data[1]]);
                    let msg = DriverMessage {moving, turning};
                    sender.send(msg).unwrap();
                },
                Message::Ping(data) => {
                    socket.send(Message::Pong(data)).await;
                },
                Message::Pong(_) => {},
                Message::Close(_) => {
                    break;
                },
            }
        } else {
            tracing::info!("Client abruptly disconnected");
            return;
        }
    }
}

async fn ws_handler (
    State(sender): State<UnboundedSender<DriverMessage>>,
    ws: WebSocketUpgrade
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_connection(socket, sender))
}
  
#[tokio::main]
async fn main () {
    let sender = peripheral::start().unwrap();
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    tracing::info!("WebSocket started");

    let app = Router::new()
        .route("/websocket", get(ws_handler))
        .layer(
            TraceLayer::new_for_http()
                .make_span_with(DefaultMakeSpan::default().include_headers(true))
        ).with_state(sender);

    let addr = SocketAddr::from(([127, 0, 0, 1], 5000));

    axum::Server::bind(&addr)
        .serve(app.into_make_service())
        .await
        .unwrap();
}
