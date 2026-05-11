use std::thread;

use control_core::{
    lora::{LoraCommand, LoraEvent},
    protocol::{self, LoRaAddr, app::LoRaModuleState},
    wifi::WiFiEvent,
};
use crossbeam_channel::Receiver;
use slint::ToSharedString;
use tracing::{debug, error, info, info_span, warn};

mod colour_bar;
mod db;
mod utils;

slint::include_modules!();

fn main() {
    tracing_subscriber::fmt()
        .with_max_level(tracing::level_filters::LevelFilter::DEBUG)
        .init();

    let previous_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let location = info
            .location()
            .map(|l| format!("{}:{}:{}", l.file(), l.line(), l.column()))
            .unwrap_or_else(|| "<unknown>".into());
        let payload = if let Some(s) = info.payload().downcast_ref::<&str>() {
            (*s).to_string()
        } else if let Some(s) = info.payload().downcast_ref::<String>() {
            s.clone()
        } else {
            "<non-string payload>".into()
        };

        error!(%location, %payload, "Thread panicked.");

        // Call previous panic hook.
        previous_hook(info); // forward to previous (e.g. default stderr printer)
    }));

    info!("Starting...");

    let mut _single = None;
    match single_instance::SingleInstance::new("comms-module-controller") {
        Ok(si) => {
            if !si.is_single() {
                error!(
                    "Another app instance is already running. This is not supported. Quiting..."
                );

                std::process::exit(1);
            } else {
                _single = Some(si);
            }
        }
        Err(err) => warn!(?err, "Failed to check whether this was the only instance."),
    };

    let (controller, mock) = control_core::Controller::mocked();

    info!(%controller.addr, "Controller address selected.");

    let app_window = AppWindow::new().unwrap();

    app_window
        .global::<GlobalAppState>()
        .set_controller_addr(controller.addr.to_raw() as i32);

    {
        let lora_commands = controller.lora_commands.clone();
        app_window.on_lora_ping(move |addr| {
            let addr = LoRaAddr::from_str(&addr).unwrap();
            lora_commands.send(LoraCommand(addr, Vec::new())).unwrap();
        });

        let lora_commands = controller.lora_commands.clone();
        app_window.on_set_wifi_enabled(move |addr, enable| {
            let addr = LoRaAddr::from_str(&addr).unwrap();

            let cmd = if enable {
                protocol::app::LoRaCmd::EnableWiFi
            } else {
                protocol::app::LoRaCmd::DisableWiFi
            };

            let mut buf = [0u8; 32];
            let mut len = 0;
            cmd.serialize(&mut buf, &mut len).unwrap();

            lora_commands
                .send(LoraCommand(addr, buf[..len].to_vec()))
                .unwrap();
        });

        let lora_commands = controller.lora_commands.clone();
        app_window.on_set_lora_recv_window(move |addr, window| {
            let addr = LoRaAddr::from_str(&addr).unwrap();

            let mut buf = [0u8; 32];
            let mut len = 0;
            protocol::app::LoRaCmd::SetLoRaRecvWindow(protocol::app::LoRaRecvWindow {
                on_period: window.0 as _,
                total_period: window.1 as _,
            })
            .serialize(&mut buf, &mut len)
            .unwrap();

            lora_commands
                .send(LoraCommand(addr, buf[..len].to_vec()))
                .unwrap();
        });

        let lora_commands = controller.lora_commands.clone();
        app_window.on_set_storage_policy(move |addr, policy| {
            let addr = LoRaAddr::from_str(&addr).unwrap();
            let policy = protocol::app::StoragePolicy::from_str(policy.as_str());

            let mut buf = [0u8; 32];
            let mut len = 0;
            protocol::app::LoRaCmd::SetStoragePolicy(policy)
                .serialize(&mut buf, &mut len)
                .unwrap();

            lora_commands
                .send(LoraCommand(addr, buf[..len].to_vec()))
                .unwrap();
        });

        let lora_commands = controller.lora_commands.clone();
        app_window.on_set_overwritable_upto(move |addr, overwrite_upto| {
            let addr = LoRaAddr::from_str(&addr).unwrap();

            let mut buf = [0u8; 32];
            let mut len = 0;
            protocol::app::LoRaCmd::SetOverwritable(protocol::app::Overwritable {
                up_to: overwrite_upto as _,
            })
            .serialize(&mut buf, &mut len)
            .unwrap();

            lora_commands
                .send(LoraCommand(addr, buf[..len].to_vec()))
                .unwrap();
        });
    }

    {
        let mock_module = mock.clone();
        app_window.on_debug_set_lora_module_attached(move |attached| {
            mock_module.state().is_laptop_lora_module_attached = attached;
        });

        let mock_module = mock.clone();
        app_window.on_debug_set_wifi_module_attached(move |attached| {
            mock_module.state().is_laptop_wifi_module_attached = attached;
        });
    }

    {
        let events = controller.lora_events.clone();
        let ui_handle = app_window.as_weak();
        thread::spawn(move || lora_events_handler(events, ui_handle));

        let events = controller.wifi_receiver.clone();
        let ui_handle = app_window.as_weak();
        thread::spawn(move || wifi_events_handler(events, ui_handle));
    }

    let a = LoRaAddr::from_raw(0x33AD);
    let db_conn = db::Database::open().unwrap();
    db_conn.add_address(a);
    for i in 60..80 {
        db_conn.write_block(a, i, b"hello");
    }
    for i in 0..5 {
        db_conn.write_block(a, i + (1 << 32), b"hello");
    }

    app_window.run().unwrap();
}

fn lora_events_handler(lora_events: Receiver<LoraEvent>, ui_handle: slint::Weak<AppWindow>) {
    let _span = info_span!("lora_events_handler").entered();

    debug!("Listening for LoRa events.");
    while let Ok(event) = lora_events.recv() {
        debug!(?event, "Got LoRa Event");

        match &event {
            LoraEvent::Message(m) => {
                let m = String::from_utf8_lossy(m);
                info!(%m, "LoRa message received");
                warn!("TODO unimplemented: dropping message");
            }
            _ => {}
        }

        let ui_handle_copy = ui_handle.clone();
        slint::invoke_from_event_loop(move || {
            if let Some(ui) = ui_handle_copy.upgrade() {
                ui.invoke_on_lora_reply();

                match event {
                    LoraEvent::Attached => {
                        ui.global::<GlobalAppState>().set_lora_attached(true);
                        ui.global::<GlobalAppState>().set_lora_connected(false);
                        ui.global::<GlobalAppState>()
                            .set_lora_module_addr(format!("{}", LoRaAddr::bcast()).into());
                    }
                    LoraEvent::Detached => {
                        ui.global::<GlobalAppState>().set_lora_attached(false);
                        ui.global::<GlobalAppState>().set_lora_connected(false);
                        ui.global::<GlobalAppState>()
                            .set_lora_module_addr(format!("{}", LoRaAddr::bcast()).into());
                    }
                    LoraEvent::Discovered(addr) => {
                        ui.global::<GlobalAppState>()
                            .set_lora_module_addr(format!("{}", addr).into());
                        ui.global::<GlobalAppState>().set_lora_connected(true);
                    }
                    LoraEvent::TimedOut(_addr) => {
                        ui.global::<GlobalAppState>()
                            .set_lora_module_addr(format!("{}", LoRaAddr::bcast()).into());
                        ui.global::<GlobalAppState>().set_lora_connected(false)
                    }
                    LoraEvent::Message(m) => {
                        let mut pos = 0;
                        let state = match LoRaModuleState::parse(&m, &mut pos) {
                            Ok(state) => state,
                            Err(err) => {
                                warn!(?err, "Bad LoRa state message.");
                                return;
                            }
                        };

                        ui.global::<ModuleState>()
                            .set_storage_total(state.storage_info.total_blocks as _);
                        ui.global::<ModuleState>()
                            .set_storage_available_begin(state.storage_info.available_begin as _);
                        ui.global::<ModuleState>()
                            .set_storage_available_end(state.storage_info.available_end as _);
                        ui.global::<ModuleState>().set_storage_overwriteable_upto(
                            state.storage_info.overwritable_end as _,
                        );
                        ui.global::<ModuleState>()
                            .set_storage_generation(state.storage_info.generation as _);

                        let ring_points = utils::RingBufferPoints::from_storage_info(
                            &state.storage_info,
                            state.status_flags & LoRaModuleState::STATUS_STORAGE_FULL != 0,
                        );

                        ui.global::<ModuleState>()
                            .set_p_overwritable_1(ring_points.p_overwritable_1);
                        ui.global::<ModuleState>()
                            .set_p_preserved_1(ring_points.p_preserved_1);
                        ui.global::<ModuleState>()
                            .set_p_free_2(ring_points.p_free_2);
                        ui.global::<ModuleState>()
                            .set_p_overwritable_2(ring_points.p_overwritable_2);
                        ui.global::<ModuleState>()
                            .set_p_preserved_2(ring_points.p_preserved_2);

                        ui.global::<ModuleState>()
                            .set_lora_recv_win_on_period(state.lora_recv_window.on_period as _);
                        ui.global::<ModuleState>()
                            .set_lora_recv_win_on_period(state.lora_recv_window.total_period as _);

                        ui.global::<ModuleState>()
                            .set_storage_policy(state.storage_policy.to_str().into());

                        ui.global::<ModuleState>().set_wifi_enabled(
                            state.status_flags & LoRaModuleState::STATUS_WIFI_ON != 0,
                        );
                        ui.global::<ModuleState>().set_wifi_data_dumping(
                            state.status_flags & LoRaModuleState::STATUS_WIFI_DUMPING != 0,
                        );
                        ui.global::<ModuleState>().set_wifi_data_dumping(
                            state.status_flags & LoRaModuleState::STATUS_WIFI_DUMPING != 0,
                        );

                        let addr = ui.global::<GlobalAppState>().get_lora_module_addr();
                        let addr = LoRaAddr::from_str(&addr).unwrap();
                        let db_conn = db::Database::open().unwrap();
                        redraw_download_bar(&ui, addr, &db_conn);
                    }
                }
            }
        })
        .unwrap();
    }
}

fn wifi_events_handler(events: Receiver<WiFiEvent>, ui_handle: slint::Weak<AppWindow>) {
    let _span = info_span!("wifi_events_handler").entered();

    debug!("Listening for WiFi events.");
    while let Ok(event) = events.recv() {
        debug!(?event, "Got WiFi Event");

        match &event {
            WiFiEvent::ReceiveMessage(m) => {
                let m = String::from_utf8_lossy(m);
                info!(%m, "WiFi message received");
                warn!("TODO unimplemented: dropping message");
            }
            _ => {}
        };

        let ui_handle_copy = ui_handle.clone();
        slint::invoke_from_event_loop(move || {
            if let Some(ui) = ui_handle_copy.upgrade() {
                match event {
                    WiFiEvent::Attached => {
                        ui.global::<GlobalAppState>().set_wifi_attached(true);
                        ui.global::<GlobalAppState>().set_wifi_connected(false);
                    }
                    WiFiEvent::Detached => {
                        ui.global::<GlobalAppState>().set_wifi_attached(false);
                        ui.global::<GlobalAppState>().set_wifi_connected(false);
                    }
                    WiFiEvent::Ping((mac, addr)) => {
                        if ui.global::<GlobalAppState>().get_lora_module_addr()
                            == format!("{}", addr)
                        {
                            ui.global::<GlobalAppState>()
                                .set_wifi_connected_mac(mac.to_shared_string());
                            //
                            ui.invoke_on_wifi_recv();
                        }
                    }
                    WiFiEvent::Connected(mac) => {
                        ui.global::<GlobalAppState>().set_wifi_connected(true);
                        ui.global::<GlobalAppState>()
                            .set_wifi_connected_mac(mac.to_shared_string());
                    }
                    WiFiEvent::Disconnected(_mac) => {
                        ui.global::<GlobalAppState>().set_wifi_connected(false);
                    }
                    WiFiEvent::ReceiveMessage(message) => {
                        // Indicate that the connection is active and such.
                        ui.invoke_on_wifi_recv();

                        // Save the block to the database.
                        let addr = ui.global::<GlobalAppState>().get_lora_module_addr();
                        let addr = LoRaAddr::from_str(&addr).unwrap();

                        let end = ui.global::<ModuleState>().get_storage_available_end();
                        let generation = ui.global::<ModuleState>().get_storage_generation();

                        let (block_index, block) =
                            message.split_first_chunk::<{ size_of::<u32>() }>().unwrap();
                        let block_index = u32::from_le_bytes(*block_index);

                        let block_id = to_block_id(block_index, end as u32, generation as u16);

                        let db_conn = db::Database::open().unwrap();
                        db_conn.write_block(addr, block_id, block).unwrap();

                        // Update the downloaded blocks visualization.
                        redraw_download_bar(&ui, addr, &db_conn);
                    }
                }
            }
        })
        .unwrap();
    }
}

fn to_block_id(index: u32, end: u32, generation: u16) -> u64 {
    if index <= end {
        ((generation as u64) << u32::BITS) + index as u64
    } else {
        (((generation - 1) as u64) << u32::BITS) + index as u64
    }
}

fn redraw_download_bar(ui_handle: &AppWindow, addr: LoRaAddr, db: &db::Database) {
    let total = ui_handle.global::<ModuleState>().get_storage_total();
    let begin = ui_handle
        .global::<ModuleState>()
        .get_storage_available_begin();
    let end = ui_handle
        .global::<ModuleState>()
        .get_storage_available_end();
    let generation = ui_handle.global::<ModuleState>().get_storage_generation();

    let begin_id = to_block_id(begin as _, end as _, generation as _);
    let end_id = to_block_id(end as _, end as _, generation as _);

    let regions = db.present_ranges(addr, begin_id, end_id).unwrap();
    let mut regions = regions
        .iter()
        .map(|r| (r.0 as u32, r.1 as u32))
        .collect::<Vec<_>>();
    regions.sort_by(|r1, r2| r1.0.cmp(&r2.0));

    let w = ui_handle.get_download_bar_width();
    let empty_grey = ui_handle.global::<Theme>().get_empty_grey();
    let mut warn_orange = ui_handle
        .global::<Theme>()
        .get_warn_orange()
        .as_argb_encoded()
        .to_be_bytes();
    warn_orange.rotate_left(1);
    let mut active_blue = ui_handle
        .global::<Theme>()
        .get_active_blue()
        .as_argb_encoded()
        .to_le_bytes();
    active_blue.rotate_left(1);

    let mut buf = colour_bar::pixel_buffer(w as usize, empty_grey.red());
    colour_bar::rasterize_range(&mut buf, warn_orange, begin as _, end as _, total as _);
    colour_bar::rasterize_ranges(&mut buf, active_blue, &regions, total as _);
    let image = colour_bar::pixel_buffer_to_image(&buf);

    ui_handle.set_download_image(image);
}
