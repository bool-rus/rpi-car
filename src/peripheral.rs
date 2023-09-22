use std::time::Duration;

use thiserror::Error;
use tokio::sync::mpsc::{UnboundedSender as Sender, unbounded_channel};
use driver::*;
pub type Result<T> = std::result::Result<T, DriverError>;

const TIMEOUT: Duration = Duration::from_millis(250);

const MAX: f64 = 100.0;

#[derive(Debug, Clone, Copy)]
pub struct DriverMessage {
    pub moving: i8,
    pub turning: i8,
}

impl DriverMessage {
    pub fn is_stopped(&self) -> bool {
        self.moving == 0 && self.turning == 0
    }
    pub fn calibrated(self) -> Self {
        let Self {mut moving, mut turning} = self;
        let max = MAX as i8;
        if moving.abs() > max {
            moving = moving.signum() * max;
        }
        if turning.abs() > max {
            turning = turning.signum() * max;
        }
        Self {moving, turning}
    }
}

#[derive(Debug, Clone, Copy)]
pub struct DriverConfig {
    motor_freq: f64,
    servo_center: Duration,
    left_factor: f64,
    right_factor: f64,
}

impl DriverConfig {
    pub fn new(left: Duration, center: Duration, right: Duration) -> Self {
        let motor_freq = 1000.0;
        let servo_center = center;
        let left = left.as_secs_f64();
        let center = center.as_secs_f64();
        let right = right.as_secs_f64();
        let left_factor = (center - left) / MAX;
        let right_factor = (right - center) / MAX;
        Self {servo_center, left_factor, right_factor, motor_freq}
    }
    pub fn with_motor_freq(mut self, motor_freq: f64)-> Self {
        self.motor_freq = motor_freq;
        self
    }
}

#[test]
fn test_config() {
    let conf = DriverConfig::new(Duration::from_micros(1300), Duration::from_micros(1500), Duration::from_micros(1800));
    dbg!(conf);
}

pub fn start(config: DriverConfig) -> Result<Sender<DriverMessage>> {
    tracing::info!("driver config: {:?}", config);
    let mut driver = Driver::init(config)?;
    let (sender, mut receiver) = unbounded_channel();
    tokio::spawn(async move {
        let mut prev_state: DriverMessage = receiver.recv().await.unwrap();
        loop {
            let timeout = tokio::time::sleep(TIMEOUT);
            tokio::select! {
                _ = timeout => {
                    if !prev_state.is_stopped() {
                        driver.stop().ok_or_log();
                        prev_state = DriverMessage { moving: 0, turning: 0 };
                    }
                },
                msg = receiver.recv() => {
                    if let Some(msg) = msg {
                        if msg.moving != prev_state.moving {
                            driver.set_moving(msg.moving).ok_or_log();
                        }
                        if msg.turning != prev_state.turning {
                            driver.turn(msg.turning).ok_or_log();
                        }
                        prev_state = msg;
                    } else {
                        break;
                    }
                },
            }
        }
        tracing::info!("driver stopped");
    });
    Ok(sender)
}


const FORWARD_POLARITY: Polarity = Polarity::Normal;
const BACKWARD_POLARITY: Polarity = Polarity::Inverse;
const SERVO_POLARITY: Polarity = Polarity::Normal;
const MOTOR_FREQ: f64 = 100.0;
const SERVO_PERIOD: Duration = Duration::from_millis(20);
fn forward(pin: &mut OutputPin) {
    pin.set_low();
} 
fn backward(pin: &mut OutputPin) {
    pin.set_high();
}


pub struct Driver {
    config: DriverConfig,
    direction: OutputPin,
    mover: Pwm,
    turner: Pwm,
}

impl Driver {
    pub fn init(config: DriverConfig) -> Result<Self> {
        let mut gpio = Gpio::new()?;
        let direction = gpio.get(22)?.into_output();
        let mover = Pwm::with_frequency(Channel::Pwm0, config.motor_freq, 0.0, FORWARD_POLARITY, true)?;
        let turner = Pwm::with_period(Channel::Pwm1, SERVO_PERIOD, config.servo_center, SERVO_POLARITY, true)?;
        Ok(Self {direction, mover, turner, config})
    }
    pub fn set_moving(&mut self, moving: i8) -> Result<()> {
        let duty = (moving.abs() as f64)/ MAX;
        let duty = 0.5 + duty/2.0;
        self.mover.set_duty_cycle(duty)?;
        if moving > 0 {
            forward(&mut self.direction);
            self.mover.set_polarity(FORWARD_POLARITY)?;
        } else if moving < 0 {
            backward(&mut self.direction);
            self.mover.set_polarity(BACKWARD_POLARITY)?;
        } else {
            self.mover.set_duty_cycle(0.0)?;
        }
        Ok(())
    }
    pub fn turn(&mut self, turn: i8) -> Result<()> {
        let secs = self.config.servo_center.as_secs_f64();
        let period = if turn > 0 {
            Duration::from_secs_f64(secs + self.config.right_factor * (turn as f64))
        } else if turn < 0 {
            Duration::from_secs_f64(secs + self.config.left_factor * (turn as f64))
        } else {
            self.config.servo_center
        };
        self.turner.set_pulse_width(period)?;
        Ok(())
    }
    pub fn stop(&mut self) -> Result<()> {
        self.mover.set_duty_cycle(0.0)?;
        self.turner.set_pulse_width(self.config.servo_center)?;
        Ok(())
    }
}

pub trait OkOrLog<T> {
    fn ok_or_log(self) -> Option<T>;
}

impl <T,E: std::error::Error> OkOrLog<T> for std::result::Result<T,E> {
    fn ok_or_log(self) -> Option<T> {
        match self {
            Ok(t) => Some(t),
            Err(e) => {
                tracing::error!("{}", e);
                None
            },
        }
    }
}


#[derive(Error, Debug)]
pub enum DriverError {
    #[error(transparent)]
    Pwm(#[from] PwmError),
    #[error(transparent)]
    Gpio(#[from] GpioError),
}



#[cfg(feature="rpi")]
mod driver {
    pub use rppal::{pwm::{Pwm, Polarity, Channel}, gpio::{OutputPin, Gpio}};
    pub use rppal::pwm::Error as PwmError;
    pub use rppal::gpio::Error as GpioError;
}

//mock driver
#[cfg(not(feature="rpi"))]
mod driver {
    use std::time::Duration;

    use thiserror::Error;

    #[derive(Debug)]
    pub enum Channel {
        Pwm0,
        Pwm1
    }

    #[derive(Debug)]
    pub enum Polarity {
        Normal,
        Inverse,
    }

    pub struct Gpio;
    pub struct OutputPin;

    impl OutputPin {
        pub fn into_output(self) -> Self {self}
        pub fn set_low(&mut self) {println!("set low");}
        pub fn set_high(&mut self) {println!("set high");}
    }

    impl Gpio {
        pub fn new() -> Result<Self, GpioError> {Ok(Self)}
        pub fn get(&mut self, pin: u32) -> Result<OutputPin, GpioError> {Ok(OutputPin)}
    }

    pub struct Pwm (Channel);
    impl Pwm {
        pub fn with_frequency(channel: Channel, freq: f64, duty: f64, polarity: Polarity, enabled: bool) -> Result<Self, PwmError> {
            println!("creating Pwm with freq, ch: {channel:?}, freq: {freq}, duty: {duty} dpol: {polarity:?}");
            Ok(Self(channel))
        }
        pub fn with_period(channel: Channel, period: Duration, pulse_width: Duration, polarity: Polarity, enabled: bool) -> Result<Self, PwmError> {
            println!("creating Pwm with dur, ch: {channel:?}, period: {period:?}, pulse_width: {pulse_width:?}, pol: {polarity:?}");
            Ok(Self(channel))
        }
        pub fn set_duty_cycle(&mut self, duty: f64) -> Result<(), PwmError> {
            println!("set duty cycle {duty} on {:?}", self.0);
            if duty > 1.0 {
                panic!("duty more than one: {}", duty);
            }
            Ok(())
        }
        pub fn set_polarity(&mut self, polarity: Polarity) -> Result<(), PwmError> {
            println!("set polarity {polarity:?} on {:?}", self.0);
            Ok(())
        }
        pub fn set_pulse_width(&mut self, dur: Duration) -> Result<(), PwmError> {
            println!("set pulse width {dur:?} on {:?}", self.0);
            Ok(())
        }
    }

    pub struct Driver;

    #[derive(Error, Debug)]
    #[error(transparent)]
    pub struct PwmError(#[from]Box<dyn std::error::Error>);    
    
    #[derive(Error, Debug)]
    #[error(transparent)]
    pub struct GpioError(#[from]Box<dyn std::error::Error>);
    
}

