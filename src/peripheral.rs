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
    pub motor_freq: f64,
    pub motor_pin: u8,
    pub min_duty: f64,
    servo_center: Duration,
    left_factor: f64,
    right_factor: f64,
    pin_forward_level: Level,
}

impl DriverConfig {
    pub fn new(left: Duration, center: Duration, right: Duration) -> Self {
        let motor_freq = 1000.0;
        let min_duty = 0.5;
        let motor_pin = 22;
        let servo_center = center;
        let left = left.as_secs_f64();
        let center = center.as_secs_f64();
        let right = right.as_secs_f64();
        let left_factor = (center - left) / MAX;
        let right_factor = (right - center) / MAX;
        let pin_forward_level = Level::Low;
        Self {servo_center, left_factor, right_factor, motor_freq, motor_pin, min_duty, pin_forward_level}
    }
    fn calculate_duty(&self, moving: i8) -> f64 {
        let moving = moving.abs() as f64 / MAX;
        let duty = self.min_duty + (1.0 - self.min_duty) * moving;
        if duty > 1.0 {
            tracing::error!("duty more than full: {}", duty);
            return 1.0;
        }
        return duty;
    }
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
const SERVO_POLARITY: Polarity = Polarity::Normal;
const SERVO_PERIOD: Duration = Duration::from_millis(20);


fn invert_polarity(pol: Polarity) -> Polarity {
    use Polarity::*;
    match pol {
        Normal => Inverse,
        Inverse => Normal,
    }
}

pub struct Driver {
    config: DriverConfig,
    direction: OutputPin,
    mover: Pwm,
    turner: Pwm,
}

impl Driver {
    //нужно заново переинициализировать все используемые пины
    fn raise_up(&mut self) -> Result<()> {
        self.set_moving(1)?;
        self.set_moving(-1)?;
        self.stop()?;
        Ok(())
    }
    pub fn init(config: DriverConfig) -> Result<Self> {
        let mut gpio = Gpio::new()?;
        let direction = gpio.get(config.motor_pin)?.into_output();
        let mover = Pwm::with_frequency(Channel::Pwm0, config.motor_freq, 0.0, FORWARD_POLARITY, true)?;
        let turner = Pwm::with_period(Channel::Pwm1, SERVO_PERIOD, config.servo_center, SERVO_POLARITY, true)?;
        let mut me = Self {direction, mover, turner, config};
        me.raise_up()?;
        Ok(me)
    }
    pub fn set_moving(&mut self, moving: i8) -> Result<()> {
        let duty = self.config.calculate_duty(moving);
        self.mover.set_duty_cycle(duty)?;
        if moving > 0 {
            self.move_forward()?;
        } else if moving < 0 {
            self.move_backward()?;
        } else {
            self.mover.set_duty_cycle(0.0)?;
        }
        Ok(())
    }
    fn move_forward(&mut self) -> Result<()> {
        self.direction.write(self.config.pin_forward_level);
        self.mover.set_polarity(FORWARD_POLARITY)?;
        Ok(())
    }
    fn move_backward(&mut self) -> Result<()> {
        self.direction.write(!self.config.pin_forward_level);
        self.mover.set_polarity(invert_polarity(FORWARD_POLARITY))?;
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
    pub use rppal::gpio::Level;
}

//mock driver
#[cfg(not(feature="rpi"))]
mod driver {
    use std::{time::Duration, ops::Not};

    use thiserror::Error;

    #[derive(Debug, Clone, Copy)]
    pub enum Level {
        Low,
        High,
    }

    impl Not for Level {
        type Output = Level;

        fn not(self) -> Self::Output {
            match self {
                Level::Low => Level::High,
                Level::High => Level::Low,
            }
        }
    }
    
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
        pub fn write(&mut self, level: Level) {println!("set {level:?}");}
    }

    impl Gpio {
        pub fn new() -> Result<Self, GpioError> {Ok(Self)}
        pub fn get(&mut self, pin: u8) -> Result<OutputPin, GpioError> {Ok(OutputPin)}
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

    #[derive(Error, Debug)]
    #[error(transparent)]
    pub struct PwmError(#[from]Box<dyn std::error::Error>);    
    
    #[derive(Error, Debug)]
    #[error(transparent)]
    pub struct GpioError(#[from]Box<dyn std::error::Error>);
    
}

