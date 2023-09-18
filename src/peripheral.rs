use std::time::Duration;

use rppal::{pwm::{Pwm, Polarity, Channel}, gpio::{OutputPin, Gpio}};
use thiserror::Error;
use tokio::sync::mpsc::{UnboundedSender as Sender, unbounded_channel};
pub type Result<T> = std::result::Result<T, DriverError>;

const FORWARD_POLARITY: Polarity = Polarity::Normal;
const BACKWARD_POLARITY: Polarity = Polarity::Inverse;
const SERVO_POLARITY: Polarity = Polarity::Normal;
const MOTOR_FREQ: f64 = 1000.0;
const SERVO_PERIOD: Duration = Duration::from_millis(20);
const SERVO_LEFT: Duration = Duration::from_micros(1300);
const SERVO_ZERO: Duration = Duration::from_micros(1500);
const SERVO_RIGHT: Duration = Duration::from_micros(1800);
const TIMEOUT: Duration = Duration::from_millis(100);
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

#[derive(Debug, Clone, Copy)]
pub struct DriverMessage {
    pub moving: i8,
    pub turning: i8,
}

struct Driver {
    left_factor: f64,
    right_factor: f64,
    direction: OutputPin,
    mover: Pwm,
    turner: Pwm,
}


impl Driver {
    fn init(gpio: &mut Gpio) -> Result<Self> {
        let direction = gpio.get(22)?.into_output();
        let mover = Pwm::with_frequency(Channel::Pwm0, MOTOR_FREQ, 0.0, FORWARD_POLARITY, true)?;
        let turner = Pwm::with_period(Channel::Pwm1, SERVO_PERIOD, SERVO_ZERO, SERVO_POLARITY, true)?;
        let left_factor = i8::MAX as f64/  (SERVO_ZERO - SERVO_LEFT).as_secs_f64();
        let right_factor = i8::MAX as f64 /(SERVO_RIGHT - SERVO_ZERO).as_secs_f64();
        Ok(Self {direction, mover, turner, left_factor, right_factor})
    }
    fn set_moving(&mut self, moving: i8) -> Result<()> {
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
    fn turn(&mut self, turn: i8) -> Result<()> {
        let secs = SERVO_ZERO.as_secs_f64();
        let period = if turn > 0 {
            Duration::from_secs_f64(secs + self.right_factor * (turn as f64))
        } else if turn < 0 {
            Duration::from_secs_f64(secs + self.left_factor * (turn as f64))
        } else {
            SERVO_ZERO
        };
        self.turner.set_period(period)?;
        Ok(())
    }
    fn stop(&mut self) -> Result<()> {
        self.mover.set_duty_cycle(0.0)?;
        self.turner.set_period(SERVO_ZERO)?;
        Ok(())
    }

}

pub fn start() -> Result<Sender<DriverMessage>> {
    let mut gpio = Gpio::new()?;
    let mut driver = Driver::init(&mut gpio)?;
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