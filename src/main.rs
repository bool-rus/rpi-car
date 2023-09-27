use clap::Parser;
use peripheral::DriverMessage;
use tokio::sync::mpsc::UnboundedSender;
use tower_http::trace::{TraceLayer, DefaultMakeSpan};

use axum::{
    extract::{ws::{Message, WebSocket, WebSocketUpgrade}, State},
    response::IntoResponse,
    routing::get,
    Router
};
use std::{net::SocketAddr, time::Duration, str::FromStr};

use crate::peripheral::{DriverConfig, OkOrLog};

mod peripheral;


async fn handle_connection (mut socket: WebSocket, sender: UnboundedSender<DriverMessage>) {
    tracing::info!("Websocket started!");
    while let Some(msg) = socket.recv().await {
        if let Ok(msg) = msg {
            match msg {
                Message::Text(text) => {
                    tracing::error!("Unknown message: {text}");
                },
                Message::Binary(data) => {
                    if data.len() != 2 {
                        tracing::error!("wrong data len, data: {:?}", data);
                        continue;
                    }
                    let moving = i8::from_be_bytes([data[0]]);
                    let turning = i8::from_be_bytes([data[1]]);
                    let msg = DriverMessage {moving, turning};
                    sender.send(msg.calibrated()).ok_or_log();
                },
                Message::Ping(data) => {
                    socket.send(Message::Pong(data)).await.ok_or_log();
                },
                Message::Pong(_) => {},
                Message::Close(_) => {
                    break;
                },
            }
        } else {
            tracing::error!("Client abruptly disconnected");
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
    tracing_subscriber::fmt()
    .with_max_level(tracing::Level::INFO)
    .init();

    let Args { listen, servo_center, servo_left, servo_right, motor_frequency, motor_pin, min_duty } = Args::parse();
    let addr = SocketAddr::from_str(&listen).unwrap();
    let mut config = DriverConfig::new(
        Duration::from_micros(servo_left as u64), 
        Duration::from_micros(servo_center as u64), 
        Duration::from_micros(servo_right as u64)
    );
    config.motor_freq = motor_frequency as f64;
    config.motor_pin = motor_pin;
    config.min_duty = min_duty;
    let sender = peripheral::start(config).unwrap();

    let app = Router::new()
        .route("/websocket", get(ws_handler))
        .layer(
            TraceLayer::new_for_http()
                .make_span_with(DefaultMakeSpan::default().include_headers(true))
        ).with_state(sender);


    axum::Server::bind(&addr)
        .serve(app.into_make_service())
        .await
        .unwrap();
}

#[derive(Parser, Debug)]
struct Args {
    /// address to input connections
    #[arg(long, short, default_value="127.0.0.1:5000")]
    listen: String,
    /// servo period (micros) to center
    #[arg(long, default_value="1500")]
    servo_center: u32,
    /// servo period (micros) to max left
    #[arg(long, default_value="1300")]
    servo_left: u32,
    /// servo period (micros) to max right
    #[arg(long, default_value="1800")]
    servo_right:  u32,
    /// motor frequency in Hz
    #[arg(long, default_value="1000")]
    motor_frequency: u32,
    /// motor minimal duty (0..1)
    #[arg(long, default_value="0.5")]
    min_duty: f64, 
    /// motor pin
    #[arg(long, default_value="22")]
    motor_pin: u8,
}

#[test]
fn mytest() {
    println!("max: {}, min: {}", i8::MAX, i8::MIN);
}