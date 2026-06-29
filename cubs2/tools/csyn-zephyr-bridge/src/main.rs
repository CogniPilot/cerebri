use std::net::UdpSocket;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result, anyhow};
use clap::Parser;
use zenoh::Wait;

const CSYN_TOPIC_FLIGHT_SNAPSHOT: u16 = 0;
const CSYN_TOPIC_MOTOR_OUTPUT: u16 = 1;
const CSYN_TOPIC_CONTROL_OUTPUT: u16 = 2;
const CSYN_TOPIC_MOCAP_FRAME: u16 = 3;
const CSYN_TOPIC_SIM_INPUT: u16 = 4;
const CSYN_TOPIC_MANUAL_CONTROL: u16 = 5;

#[derive(Debug, Parser)]
#[command(
    name = "cubs2-csyn-zephyr-bridge",
    about = "Bridge cubs2 zephyr.exe csyn UDP packets to Zenoh topics"
)]
struct Args {
    #[arg(
        long,
        env = "CSYN_CONNECT",
        default_value = "tcp/127.0.0.1:7447",
        help = "Zenoh router endpoint"
    )]
    connect: String,
    #[arg(
        long,
        default_value = "127.0.0.1:4250",
        help = "UDP address where zephyr.exe receives csyn topic packets"
    )]
    zephyr_input: String,
    #[arg(
        long,
        default_value = "127.0.0.1:4251",
        help = "UDP address receiving csyn topic packets from zephyr.exe"
    )]
    output_bind: String,
    #[arg(long, default_value = "synapse/manual_control")]
    manual_control_topic: String,
    #[arg(long, default_value = "synapse/mocap_frame")]
    mocap_topic: String,
    #[arg(long, default_value = "synapse/sim_input")]
    sim_input_topic: String,
    #[arg(long, default_value = "synapse/flight_snapshot")]
    flight_topic: String,
    #[arg(long, default_value = "synapse/motor_output")]
    motor_topic: String,
    #[arg(long, default_value = "synapse/control_output")]
    control_output_topic: String,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let shutdown = shutdown_flag()?;

    let mut config = zenoh::Config::default();
    config
        .insert_json5("connect/endpoints", &format!("[\"{}\"]", args.connect))
        .map_err(|error| anyhow!("failed to configure Zenoh endpoint: {error}"))?;
    let session = zenoh::open(config)
        .wait()
        .map_err(|error| anyhow!("failed to open Zenoh session: {error}"))?;

    let manual_sub = session
        .declare_subscriber(args.manual_control_topic.clone())
        .wait()
        .map_err(|error| anyhow!("failed to subscribe to {}: {error}", args.manual_control_topic))?;
    let mocap_sub = session
        .declare_subscriber(args.mocap_topic.clone())
        .wait()
        .map_err(|error| anyhow!("failed to subscribe to {}: {error}", args.mocap_topic))?;
    let sim_input_sub = session
        .declare_subscriber(args.sim_input_topic.clone())
        .wait()
        .map_err(|error| anyhow!("failed to subscribe to {}: {error}", args.sim_input_topic))?;

    let tx = CsynUdpTx::connect(&args.zephyr_input)?;
    let mut rx = CsynUdpRx::bind(&args.output_bind)?;

    eprintln!("cubs2 csyn bridge via Zenoh {}", args.connect);
    eprintln!(
        "  in:  {}, {}, {} -> UDP {}",
        args.manual_control_topic, args.mocap_topic, args.sim_input_topic, args.zephyr_input
    );
    eprintln!(
        "  out: UDP {} -> {}, {}, {}",
        args.output_bind, args.flight_topic, args.motor_topic, args.control_output_topic
    );

    while !shutdown.load(Ordering::Relaxed) {
        drain_subscriber(&manual_sub, CSYN_TOPIC_MANUAL_CONTROL, &tx, "manual_control")?;
        drain_subscriber(&mocap_sub, CSYN_TOPIC_MOCAP_FRAME, &tx, "mocap_frame")?;
        drain_subscriber(&sim_input_sub, CSYN_TOPIC_SIM_INPUT, &tx, "sim_input")?;
        rx.drain(
            &session,
            &args.flight_topic,
            &args.motor_topic,
            &args.control_output_topic,
        )?;
        thread::sleep(Duration::from_millis(1));
    }

    Ok(())
}

type Subscriber =
    zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>;

fn drain_subscriber(
    subscriber: &Subscriber,
    topic_id: u16,
    tx: &CsynUdpTx,
    name: &str,
) -> Result<()> {
    while let Some(sample) = subscriber
        .recv_timeout(Duration::ZERO)
        .map_err(|error| anyhow!("failed to receive {name} sample: {error}"))?
    {
        let payload = sample.payload().to_bytes();
        tx.send(topic_id, &payload)?;
    }
    Ok(())
}

struct CsynUdpTx {
    socket: UdpSocket,
}

impl CsynUdpTx {
    fn connect(addr: &str) -> Result<Self> {
        let socket = UdpSocket::bind("127.0.0.1:0").context("failed to bind csyn UDP sender")?;
        socket
            .connect(addr)
            .with_context(|| format!("failed to connect csyn UDP sender to {addr}"))?;
        Ok(Self { socket })
    }

    fn send(&self, topic: u16, payload: &[u8]) -> Result<()> {
        if payload.len() > u16::MAX as usize {
            return Err(anyhow!("csyn payload too large: {} bytes", payload.len()));
        }

        let mut frame = Vec::with_capacity(8 + payload.len());
        frame.extend_from_slice(b"CSYN");
        frame.extend_from_slice(&topic.to_le_bytes());
        frame.extend_from_slice(&(payload.len() as u16).to_le_bytes());
        frame.extend_from_slice(payload);
        self.socket
            .send(&frame)
            .context("failed to send csyn UDP frame")?;
        Ok(())
    }
}

struct CsynUdpRx {
    socket: UdpSocket,
    buf: Vec<u8>,
}

impl CsynUdpRx {
    fn bind(bind: &str) -> Result<Self> {
        let socket = UdpSocket::bind(bind)
            .with_context(|| format!("failed to bind csyn UDP receiver at {bind}"))?;
        socket
            .set_nonblocking(true)
            .with_context(|| format!("failed to set {bind} nonblocking"))?;
        Ok(Self {
            socket,
            buf: vec![0_u8; 4096],
        })
    }

    fn drain(
        &mut self,
        session: &zenoh::Session,
        flight_topic: &str,
        motor_topic: &str,
        control_output_topic: &str,
    ) -> Result<()> {
        loop {
            let len = match self.socket.recv(&mut self.buf) {
                Ok(len) => len,
                Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => return Ok(()),
                Err(error) => return Err(error).context("failed to receive csyn UDP frame"),
            };

            if len < 8 || &self.buf[..4] != b"CSYN" {
                continue;
            }

            let topic = u16::from_le_bytes([self.buf[4], self.buf[5]]);
            let payload_len = u16::from_le_bytes([self.buf[6], self.buf[7]]) as usize;
            if payload_len + 8 != len {
                continue;
            }

            let keyexpr = match topic {
                CSYN_TOPIC_FLIGHT_SNAPSHOT => flight_topic,
                CSYN_TOPIC_MOTOR_OUTPUT => motor_topic,
                CSYN_TOPIC_CONTROL_OUTPUT => control_output_topic,
                _ => continue,
            };

            session
                .put(keyexpr.to_owned(), self.buf[8..len].to_vec())
                .wait()
                .map_err(|error| anyhow!("failed to publish {keyexpr}: {error}"))?;
        }
    }
}

fn shutdown_flag() -> Result<Arc<AtomicBool>> {
    let shutdown = Arc::new(AtomicBool::new(false));
    let flag = shutdown.clone();
    ctrlc::set_handler(move || {
        flag.store(true, Ordering::Relaxed);
    })
    .context("failed to install Ctrl-C handler")?;
    Ok(shutdown)
}
