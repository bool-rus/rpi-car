use std::time::Duration;

use tokio::sync::mpsc::{UnboundedSender as Sender, unbounded_channel};
use driver::*;
pub type Result<T> = std::result::Result<T, DriverError>;

const TIMEOUT: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, Copy)]
pub struct DriverMessage {
    pub moving: i8,
    pub turning: i8,
}

#[derive(Debug, Clone, Copy)]
pub struct DriverConfig {
    servo_center: Duration,
    left_factor: f64,
    right_factor: f64,
}

impl DriverConfig {
    pub fn new(left: Duration, center: Duration, right: Duration) -> Self {
        let servo_center = center;
        let left = left.as_secs_f64();
        let center = center.as_secs_f64();
        let right = right.as_secs_f64();
        let left_factor = i8::MAX as f64 /  (center - left);
        let right_factor = i8::MAX as f64 / (right - center);
        Self {servo_center, left_factor, right_factor}
    }
}


pub fn start(config: DriverConfig) -> Result<Sender<DriverMessage>> {
    let mut driver = Driver::init(config)?;
    let (sender, mut receiver) = unbounded_channel();
    tokio::spawn(async move {
        let mut prev_state: DriverMessage = receiver.recv().await.unwrap();
        loop {
            let timeout = tokio::time::sleep(TIMEOUT);
            tokio::select! {
                _ = timeout => {
                    driver.stop();
                },
                msg = receiver.recv() => {
                    if let Some(msg) = msg {
                        if msg.moving != prev_state.moving {
                            driver.set_moving(msg.moving);
                        }
                        if msg.turning != prev_state.turning {
                            driver.turn(msg.turning);
                        }
                        prev_state = msg;
                    } else {
                        break;
                    }
                },
            }
        }
        println!("driver stopped");//todo: здесь просится log::info!
    });
    Ok(sender)
}

#[cfg(not(feature="rpi"))]
mod driver {

    use std::time::Duration;
    use thiserror::Error;


    use super::DriverConfig;
    use super::Result;

    #[derive(Error, Debug)]
    pub enum DriverError {
        #[error(transparent)]
        X(#[from] std::io::Error),
    }

    pub struct Driver;


    impl Driver {
        pub fn init(config: DriverConfig) -> Result<Self> {
            tracing::info!("init");
            Ok(Self)
        }
        pub fn set_moving(&mut self, moving: i8) -> Result<()> {
            tracing::info!("moving: {moving}");
            Ok(())
        }
        pub fn turn(&mut self, turn: i8) -> Result<()> {
            tracing::info!("turn: {turn}");
            Ok(())
        }
        pub fn stop(&mut self) -> Result<()> {
            tracing::info!("stop");
            Ok(())
        }
    }
}


#[cfg(feature="rpi")]
mod driver {
    use rppal::{pwm::{Pwm, Polarity, Channel}, gpio::{OutputPin, Gpio}};
    use std::time::Duration;
    use thiserror::Error;


    use super::DriverConfig;
    use super::Result;

    const FORWARD_POLARITY: Polarity = Polarity::Normal;
    const BACKWARD_POLARITY: Polarity = Polarity::Inverse;
    const SERVO_POLARITY: Polarity = Polarity::Normal;
    const MOTOR_FREQ: f64 = 1000.0;
    const SERVO_PERIOD: Duration = Duration::from_millis(20);
    fn forward(pin: &mut OutputPin) {
        pin.set_low();
    } 
    fn backward(pin: &mut OutputPin) {
        pin.set_high();
    }

    #[derive(Error, Debug)]
    pub enum DriverError {
        #[error(transparent)]
        Pwm(#[from] rppal::pwm::Error),

        #[error(transparent)]
        Gpio(#[from] rppal::gpio::Error),
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
            let mover = Pwm::with_frequency(Channel::Pwm0, MOTOR_FREQ, 0.0, FORWARD_POLARITY, true)?;
            let turner = Pwm::with_period(Channel::Pwm1, SERVO_PERIOD, config.servo_center, SERVO_POLARITY, true)?;
            Ok(Self {direction, mover, turner, config})
        }
        pub fn set_moving(&mut self, moving: i8) -> Result<()> {
            let duty = (moving.abs() as f64)/(i8::MAX as f64);
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
            self.turner.set_period(period)?;
            Ok(())
        }
        pub fn stop(&mut self) -> Result<()> {
            self.mover.set_duty_cycle(0.0)?;
            self.turner.set_period(self.config.servo_center)?;
            Ok(())
        }
    }
}