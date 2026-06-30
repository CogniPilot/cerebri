use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::net::UdpSocket;
use std::path::{Path, PathBuf};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

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
        default_value = "udp/192.168.10.2:7447",
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
    #[arg(long, default_value = "synapse/mocap/frame")]
    mocap_topic: String,
    #[arg(long, default_value = "synapse/sim_input")]
    sim_input_topic: String,
    #[arg(long, default_value = "synapse/flight_snapshot")]
    flight_topic: String,
    #[arg(long, default_value = "synapse/motor_output")]
    motor_topic: String,
    #[arg(long, default_value = "synapse/control_output")]
    control_output_topic: String,
    #[arg(
        long,
        env = "CUBS2_BAG",
        value_name = "PATH",
        help = "Record bridged topic payloads to a cubs2 bag file"
    )]
    bag: Option<PathBuf>,
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
        .map_err(|error| {
            anyhow!(
                "failed to subscribe to {}: {error}",
                args.manual_control_topic
            )
        })?;
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
    let mut bag = args.bag.as_deref().map(BagWriter::create).transpose()?;

    eprintln!("cubs2 csyn bridge via Zenoh {}", args.connect);
    eprintln!(
        "  in:  {}, {}, {} -> UDP {}",
        args.manual_control_topic, args.mocap_topic, args.sim_input_topic, args.zephyr_input
    );
    eprintln!(
        "  out: UDP {} -> {}, {}, {}",
        args.output_bind, args.flight_topic, args.motor_topic, args.control_output_topic
    );
    if let Some(path) = &args.bag {
        eprintln!("  bag: {}", path.display());
    }

    while !shutdown.load(Ordering::Relaxed) {
        drain_subscriber(
            &manual_sub,
            CSYN_TOPIC_MANUAL_CONTROL,
            &tx,
            "manual_control",
            &args.manual_control_topic,
            &mut bag,
        )?;
        drain_subscriber(
            &mocap_sub,
            CSYN_TOPIC_MOCAP_FRAME,
            &tx,
            "mocap_frame",
            &args.mocap_topic,
            &mut bag,
        )?;
        drain_subscriber(
            &sim_input_sub,
            CSYN_TOPIC_SIM_INPUT,
            &tx,
            "sim_input",
            &args.sim_input_topic,
            &mut bag,
        )?;
        rx.drain(
            &session,
            &args.flight_topic,
            &args.motor_topic,
            &args.control_output_topic,
            &mut bag,
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
    topic_name: &str,
    bag: &mut Option<BagWriter>,
) -> Result<()> {
    while let Some(sample) = subscriber
        .recv_timeout(Duration::ZERO)
        .map_err(|error| anyhow!("failed to receive {name} sample: {error}"))?
    {
        let payload = sample.payload().to_bytes();
        if let Some(writer) = bag.as_mut() {
            writer.write_record(topic_name, &payload)?;
        }
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
        bag: &mut Option<BagWriter>,
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

            if let Some(writer) = bag.as_mut() {
                writer.write_record(keyexpr, &self.buf[8..len])?;
            }
            session
                .put(keyexpr.to_owned(), self.buf[8..len].to_vec())
                .wait()
                .map_err(|error| anyhow!("failed to publish {keyexpr}: {error}"))?;
        }
    }
}

struct BagWriter {
    writer: BufWriter<File>,
}

impl BagWriter {
    fn create(path: &Path) -> Result<Self> {
        if let Some(parent) = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
        {
            fs::create_dir_all(parent)
                .with_context(|| format!("failed to create bag directory {}", parent.display()))?;
        }

        let mut writer = BufWriter::new(
            File::create(path)
                .with_context(|| format!("failed to create bag {}", path.display()))?,
        );
        writer
            .write_all(b"CUBS2BAG1\n")
            .with_context(|| format!("failed to write bag header {}", path.display()))?;
        writer
            .write_all(b"record=u64_unix_us,u16_topic_len,u32_payload_len,topic,payload\n")
            .with_context(|| format!("failed to write bag schema {}", path.display()))?;
        Ok(Self { writer })
    }

    fn write_record(&mut self, topic: &str, payload: &[u8]) -> Result<()> {
        let topic_bytes = topic.as_bytes();
        if topic_bytes.len() > u16::MAX as usize {
            return Err(anyhow!("bag topic name too long: {topic}"));
        }
        if payload.len() > u32::MAX as usize {
            return Err(anyhow!("bag payload too large: {} bytes", payload.len()));
        }

        let timestamp_us = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .context("system clock is before UNIX epoch")?
            .as_micros() as u64;

        self.writer.write_all(&timestamp_us.to_le_bytes())?;
        self.writer
            .write_all(&(topic_bytes.len() as u16).to_le_bytes())?;
        self.writer
            .write_all(&(payload.len() as u32).to_le_bytes())?;
        self.writer.write_all(topic_bytes)?;
        self.writer.write_all(payload)?;
        self.writer.flush()?;
        Ok(())
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
