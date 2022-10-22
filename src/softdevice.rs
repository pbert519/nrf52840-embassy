#![allow(clippy::enum_variant_names)]

use nrf_softdevice::ble::{gatt_server, peripheral};
use nrf_softdevice::{raw, Softdevice};

#[embassy_executor::task]
pub async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

#[embassy_executor::task]
pub async fn gatt_server_task(sd: &'static Softdevice, server: Server) {
    #[rustfmt::skip]
    let adv_data = &[
        0x02, 0x01, raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
        0x03, 0x03, 0x09, 0x18,
        0x0a, 0x09, b'H', b'e', b'l', b'l', b'o', b'R', b'u', b's', b't',
    ];
    #[rustfmt::skip]
    let scan_data = &[
        0x03, 0x03, 0x09, 0x18,
    ];
    let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
        adv_data,
        scan_data,
    };
    let config = peripheral::Config::default();
    loop {
        let conn = peripheral::advertise_connectable(sd, adv, &config)
            .await
            .unwrap();

        let gat_res = gatt_server::run(&conn, &server, |e| match e {
            ServerEvent::Bas(e) => match e {
                BatteryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                    defmt::info!("battery notifications: {}", notifications)
                }
            },
            ServerEvent::Env(_) => {}
        })
        .await;

        defmt::info!("Gatt Service stopped: {}", gat_res);
    }
}

#[nrf_softdevice::gatt_service(uuid = "180f")]
pub struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    pub battery_level: i16,
}
#[nrf_softdevice::gatt_service(uuid = "181A")]
pub struct EnviromentalSensingService {
    #[characteristic(uuid = "2A6D", read, notify)]
    pub pressure: i16,
    #[characteristic(uuid = "2A6E", read, notify)]
    pub temperature: i16,
    #[characteristic(uuid = "2A6F", read, notify)]
    pub humidity: i16,
}

#[nrf_softdevice::gatt_server]
pub struct Server {
    pub bas: BatteryService,
    pub env: EnviromentalSensingService,
}

pub fn configure_softdevice() -> &'static mut Softdevice {
    let config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_250_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 1,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: raw::BLE_GAP_ADV_SET_COUNT_DEFAULT as u8,
            periph_role_count: raw::BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT as u8,
            central_role_count: 0,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"HelloRust" as *const u8 as _,
            current_len: 9,
            max_len: 9,
            write_perm: unsafe { core::mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    };
    Softdevice::enable(&config)
}
