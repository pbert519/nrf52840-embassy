use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    pubsub::{PubSubChannel, Subscriber},
};

#[derive(Clone)]
pub struct TemperatureHumidityMeasurement {
    pub temperature: f32,
    pub humidity: f32,
}

type MicData = [u16; 128];

const QUEUE_SIZE: usize = 1;
const SUBSCRIBER: usize = 2;
const PUBLISHER: usize = 1;
pub struct MessageHub {
    pub temperature_humidity: PubSubChannel<
        CriticalSectionRawMutex,
        TemperatureHumidityMeasurement,
        QUEUE_SIZE,
        SUBSCRIBER,
        PUBLISHER,
    >,
    pub pressure: PubSubChannel<CriticalSectionRawMutex, f32, QUEUE_SIZE, SUBSCRIBER, PUBLISHER>,
    pub battery_voltage:
        PubSubChannel<CriticalSectionRawMutex, f32, QUEUE_SIZE, SUBSCRIBER, PUBLISHER>,
    pub mic_data:
        PubSubChannel<CriticalSectionRawMutex, MicData, QUEUE_SIZE, SUBSCRIBER, PUBLISHER>,
}
impl MessageHub {
    pub fn new() -> Self {
        Self {
            temperature_humidity: PubSubChannel::new(),
            pressure: PubSubChannel::new(),
            battery_voltage: PubSubChannel::new(),
            mic_data: PubSubChannel::new(),
        }
    }
    pub fn subscriber(&'static self) -> MessageSubscriber {
        MessageSubscriber {
            temp_humidity_sub: self.temperature_humidity.subscriber().unwrap(),
            pressure_sub: self.pressure.subscriber().unwrap(),
            battery_voltage_sub: self.battery_voltage.subscriber().unwrap(),
            mic_sub: self.mic_data.subscriber().unwrap(),
        }
    }
}

pub struct MessageSubscriber {
    pub temp_humidity_sub: Subscriber<
        'static,
        CriticalSectionRawMutex,
        TemperatureHumidityMeasurement,
        QUEUE_SIZE,
        SUBSCRIBER,
        PUBLISHER,
    >,
    pub pressure_sub:
        Subscriber<'static, CriticalSectionRawMutex, f32, QUEUE_SIZE, SUBSCRIBER, PUBLISHER>,
    pub battery_voltage_sub:
        Subscriber<'static, CriticalSectionRawMutex, f32, QUEUE_SIZE, SUBSCRIBER, PUBLISHER>,
    pub mic_sub:
        Subscriber<'static, CriticalSectionRawMutex, MicData, QUEUE_SIZE, SUBSCRIBER, PUBLISHER>,
}
