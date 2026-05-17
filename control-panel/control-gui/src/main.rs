use std::{iter, thread};

use control_core::{
    db, export,
    lora::{LoraCommand, LoraEvent},
    protocol::{
        self, LoRaAddr,
        app::{LoRaModuleState, StartDataDump},
        encoding::BLOCK_SIZE,
    },
    wifi::WiFiEvent,
};
use crossbeam_channel::{Receiver, Sender, unbounded};
use slint::{EventLoopError, ModelRc, ToSharedString, VecModel};
use tracing::{debug, error, info, info_span, warn};

mod colour_bar;

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

    let db_conn = db::Database::open().unwrap();

    let (dump_sender, dump_recerver) = unbounded::<DumpEvent>();

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
        app_window.on_set_lora_recv_window(move |addr, active, period| {
            let addr = LoRaAddr::from_str(&addr).unwrap();

            let mut buf = [0u8; 32];
            let mut len = 0;
            protocol::app::LoRaCmd::SetLoRaRecvWindow(protocol::app::LoRaRecvWindow {
                on_period: active as _,
                total_period: period as _,
            })
            .serialize(&mut buf, &mut len)
            .unwrap();

            lora_commands
                .send(LoraCommand(addr, buf[..len].to_vec()))
                .unwrap();
        });

        let lora_commands = controller.lora_commands.clone();
        app_window.on_storage_mark_obtained_overwritable(move |addr, avail_begin| {
            let addr = LoRaAddr::from_str(&addr).unwrap();

            let block_id = from_block_id(avail_begin);

            let db_conn = db::Database::open().unwrap();
            let last_contiguously_obtained = db_conn.first_missing_from(addr, block_id).unwrap();

            tracing::debug!(?last_contiguously_obtained, "Last contiguously obtained");

            let mut buf = [0u8; 32];
            let mut len = 0;
            protocol::app::LoRaCmd::SetOverwritable(protocol::app::Overwritable {
                up_to: last_contiguously_obtained,
            })
            .serialize(&mut buf, &mut len)
            .unwrap();

            lora_commands
                .send(LoraCommand(addr, buf[..len].to_vec()))
                .unwrap();
        });

        let lora_commands = controller.lora_commands.clone();
        app_window.on_storage_mark_all_overwritable(move |addr, avail_end| {
            let addr = LoRaAddr::from_str(&addr).unwrap();

            let mut buf = [0u8; 32];
            let mut len = 0;
            protocol::app::LoRaCmd::SetOverwritable(protocol::app::Overwritable {
                up_to: from_block_id(avail_end),
            })
            .serialize(&mut buf, &mut len)
            .unwrap();

            lora_commands
                .send(LoraCommand(addr, buf[..len].to_vec()))
                .unwrap();
        });

        let lora_commands = controller.lora_commands.clone();
        app_window.on_storage_unmark_overwritable(move |addr, avail_begin| {
            let addr = LoRaAddr::from_str(&addr).unwrap();

            let mut buf = [0u8; 32];
            let mut len = 0;
            protocol::app::LoRaCmd::SetOverwritable(protocol::app::Overwritable {
                up_to: from_block_id(avail_begin),
            })
            .serialize(&mut buf, &mut len)
            .unwrap();

            lora_commands
                .send(LoraCommand(addr, buf[..len].to_vec()))
                .unwrap();
        });

        let lora_commands = controller.lora_commands.clone();
        app_window.on_set_storage_overwrite(move |addr, policy| {
            let addr = LoRaAddr::from_str(&addr).unwrap();
            let policy = if policy {
                protocol::app::StoragePolicy::Overwrite
            } else {
                protocol::app::StoragePolicy::Preserve
            };

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

        let ds = dump_sender.clone();
        app_window.on_request_wifi_data_dump(move |addr, begin, end| {
            let addr = LoRaAddr::from_str(&addr).unwrap();

            let begin = from_block_id(begin);
            // Inclusivity:
            // Dumps are inclusive: [start, end]
            // So we must make sure not to include the block after the last.
            // AVAILABLE_END is the write pointer after the last readable block, so we need to subtract 1.
            let end = from_block_id(end) - 1;

            let missing = db_conn.missing_in_range(addr, begin, end).unwrap();
            let mut missing_runs = db::contiguous_runs(&missing);

            tracing::debug!(?missing_runs, "Requested data dump.");

            // Reverse the list so we can treat it as a stack and still grab oldest first.
            missing_runs.reverse();

            ds.send(DumpEvent::QueueDumps(missing_runs)).unwrap();
        });

        let lora_commands = controller.lora_commands.clone();
        let ds = dump_sender.clone();
        app_window.on_cancel_wifi_data_dump(move |addr| {
            let addr = LoRaAddr::from_str(&addr).unwrap();

            ds.send(DumpEvent::CancelQueue).unwrap();

            let mut buf = [0u8; 8];
            let mut len = 0;
            protocol::app::LoRaCmd::CancelDataDump
                .serialize(&mut buf, &mut len)
                .unwrap();

            lora_commands
                .send(LoraCommand(addr, buf[..len].to_vec()))
                .unwrap();
        });

        app_window.on_data_export(move |addr| {
            let addr = LoRaAddr::from_str(&addr).unwrap();
            export::export(addr);
        });
    }

    {
        let mock_module = mock.clone();
        app_window.on_debug_set_lora_module_attached(move |attached| {
            mock_module.state().is_laptop_lora_module_attached = attached;
        });

        let mock_module = mock.clone();
        app_window.on_debug_set_lora_send_fails(move |failing| {
            mock_module.state().is_lora_send_failing = failing;
        });

        let mock_module = mock.clone();
        app_window.on_debug_set_lora_recv_fails(move |failing| {
            mock_module.state().is_lora_recv_failing = failing;
        });

        let mock_module = mock.clone();
        app_window.on_debug_set_wifi_module_attached(move |attached| {
            mock_module.state().is_laptop_wifi_module_attached = attached;
        });

        let mock_module = mock.clone();
        app_window.on_debug_set_wifi_send_fails(move |failing| {
            mock_module.state().is_wifi_send_failing = failing;
        });

        let mock_module = mock.clone();
        app_window.on_debug_set_wifi_recv_fails(move |failing| {
            mock_module.state().is_wifi_recv_failing = failing;
        });
    }

    {
        let events = controller.lora_events.clone();
        let dump_sender = dump_sender.clone();
        let ui_handle = app_window.as_weak();
        thread::spawn(move || lora_events_handler(events, dump_sender, ui_handle));

        let events = controller.wifi_receiver.clone();
        let ui_handle = app_window.as_weak();
        thread::spawn(move || wifi_events_handler(events, ui_handle));

        let commands = controller.lora_commands.clone();
        let events = dump_recerver;
        thread::spawn(move || dump_event_handler(events, commands));
    }

    let a = LoRaAddr::from_raw(0x33AD);
    let db_conn = db::Database::open().unwrap();
    db_conn.add_address(a).unwrap();
    for i in 60..80 {
        db_conn.write_block(a, i, b"hello").unwrap();
    }
    for i in 100..105 {
        db_conn.write_block(a, i, b"hello").unwrap();
    }

    let a = LoRaAddr::from_raw(0x0001);
    let dummy_block = (0..200u8).into_iter().collect::<Vec<_>>();

    // // ATP04: don't seed two blocks, 80 and 99. Expect selective retransmission.
    // db_conn.add_address(a).unwrap();
    // for i in 0..=79 {
    //     db_conn.write_block(a, i, &dummy_block).unwrap();
    // }
    // for i in 81..=98 {
    //     db_conn.write_block(a, i, &dummy_block).unwrap();
    // }

    // Initialize the addresses combobox.
    update_addresses(&db_conn, &app_window);
    init_storage_bar(&app_window);
    init_download_bar(&app_window);

    app_window.run().unwrap();
}

fn lora_events_handler(
    lora_events: Receiver<LoraEvent>,
    dump_sender: Sender<DumpEvent>,
    ui_handle: slint::Weak<AppWindow>,
) -> Result<(), EventLoopError> {
    let _span = info_span!("lora_events_handler").entered();

    debug!("Listening for LoRa events.");
    while let Ok(event) = lora_events.recv() {
        debug!(?event, "Got LoRa Event");

        let ds = dump_sender.clone();
        let ui_handle_copy = ui_handle.clone();
        slint::invoke_from_event_loop(move || {
            if let Some(ui) = ui_handle_copy.upgrade() {
                match event {
                    LoraEvent::Attached => {
                        ui.global::<GlobalAppState>().set_lora_attached(true);
                        // ui.global::<GlobalAppState>()
                        //     .set_lora_module_addr(format!("{}", LoRaAddr::bcast()).into());
                    }
                    LoraEvent::Detached => {
                        ui.global::<GlobalAppState>().set_lora_attached(false);
                        // ui.global::<GlobalAppState>()
                        //     .set_lora_module_addr(format!("{}", LoRaAddr::bcast()).into());
                    }
                    LoraEvent::Discovered(addr) => {
                        ui.global::<GlobalAppState>()
                            .set_lora_module_addr(format!("{}", addr).into());

                        let db_conn = db::Database::open().unwrap();
                        db_conn.add_address(addr).unwrap();

                        update_addresses(&db_conn, &ui);
                    }
                    LoraEvent::TimedOut(_addr) => {
                        // ui.global::<GlobalAppState>()
                        //     .set_lora_module_addr(LoRaAddr::bcast().to_shared_string());
                    }
                    LoraEvent::Message(m) => {
                        ui.invoke_on_lora_reply();

                        let mut pos = 0;
                        let state = match LoRaModuleState::parse(&m, &mut pos) {
                            Ok(state) => state,
                            Err(err) => {
                                warn!(?err, "Bad LoRa state message.");
                                return;
                            }
                        };

                        ui.global::<ModuleState>().set_storage_overwrite(
                            state.storage_policy == protocol::app::StoragePolicy::Overwrite,
                        );

                        ui.global::<ModuleState>()
                            .set_storage_total(state.storage_info.total_blocks as _);
                        ui.global::<ModuleState>()
                            .set_storage_available_begin(to_block_id(
                                state.storage_info.available_begin,
                            ));
                        ui.global::<ModuleState>()
                            .set_storage_available_end(to_block_id(
                                state.storage_info.available_end,
                            ));
                        ui.global::<ModuleState>()
                            .set_storage_overwriteable_upto(to_block_id(
                                state.storage_info.overwritable_end,
                            ));

                        ui.global::<ModuleState>()
                            .set_lora_recv_win_on_period(state.lora_recv_window.on_period as _);
                        ui.global::<ModuleState>().set_lora_recv_win_total_period(
                            state.lora_recv_window.total_period as _,
                        );

                        ui.global::<ModuleState>().set_wifi_enabled(
                            state.status_flags & LoRaModuleState::STATUS_WIFI_ON != 0,
                        );
                        ui.global::<ModuleState>().set_wifi_data_dumping(
                            state.status_flags & LoRaModuleState::STATUS_WIFI_DUMPING != 0,
                        );

                        redraw_storage_bar(&ui, &state);

                        let addr = ui.global::<GlobalAppState>().get_lora_module_addr();
                        let addr = LoRaAddr::from_str(&addr).unwrap();
                        let db_conn = db::Database::open().unwrap();
                        redraw_download_bar(&ui, addr, &db_conn);

                        if state.status_flags & LoRaModuleState::STATUS_WIFI_DUMPING == 0 {
                            ds.send(DumpEvent::ClearToSendDumpRequest(addr));
                        }

                        let total = state.storage_info.total_blocks as u64;
                        let begin_id = state.storage_info.available_begin;
                        let overw_id = state.storage_info.overwritable_end;
                        let end_id = state.storage_info.available_end;

                        let overwritable = overw_id.saturating_sub(begin_id);
                        let reserved = end_id.saturating_sub(overw_id);
                        let total_available = overwritable + reserved;

                        let free = total.saturating_sub(end_id.saturating_sub(begin_id));

                        let downloaded = db_conn
                            .present_ranges(addr, begin_id, end_id)
                            .unwrap()
                            .iter()
                            .map(|r| r.1 - r.0)
                            .sum::<u64>();

                        let remote_only =
                            end_id.saturating_sub(begin_id).saturating_sub(downloaded);

                        let total_downloaded = db_conn
                            .present_ranges(addr, 0, u64::MAX / 2)
                            .unwrap()
                            .iter()
                            .map(|r| r.1 - r.0)
                            .sum::<u64>();

                        ui.global::<ModuleState>()
                            .set_downloaded_bytes(format_bytes(downloaded).into());
                        ui.global::<ModuleState>()
                            .set_overwritable_bytes(format_bytes(overwritable).into());
                        ui.global::<ModuleState>()
                            .set_protocted_bytes(format_bytes(reserved).into());
                        ui.global::<ModuleState>()
                            .set_total_bytes(format_bytes(total).into());
                        ui.global::<ModuleState>()
                            .set_remote_only_bytes(format_bytes(remote_only).into());
                        ui.global::<ModuleState>()
                            .set_total_db_bytes(format_bytes(total_downloaded).into());
                        ui.global::<ModuleState>()
                            .set_free_bytes(format_bytes(free).into());
                        ui.global::<ModuleState>()
                            .set_total_available_bytes(format_bytes(total_available).into());

                        let lat_lon = format_dms(state.gps_info.lat, true)
                            + " "
                            + &format_dms(state.gps_info.lon, false);
                        ui.global::<ModuleState>().set_lat_lon(lat_lon.into());
                    }
                }
            }
        })?;
    }

    Ok(())
}

fn wifi_events_handler(
    events: Receiver<WiFiEvent>,
    ui_handle: slint::Weak<AppWindow>,
) -> Result<(), slint::EventLoopError> {
    let _span = info_span!("wifi_events_handler").entered();

    debug!("Listening for WiFi events.");
    while let Ok(event) = events.recv() {
        debug!(?event, "Got WiFi Event");

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

                        let (block_index, block) =
                            message.split_first_chunk::<{ size_of::<u64>() }>().unwrap();
                        let block_id = u64::from_le_bytes(*block_index);

                        tracing::debug!(%block_id, "block_id received");

                        let db_conn = db::Database::open().unwrap();
                        db_conn.write_block(addr, block_id, block).unwrap();

                        // Update the downloaded blocks visualization.
                        redraw_download_bar(&ui, addr, &db_conn);

                        // check if we need to queue up another transmission
                    }
                }
            }
        })?;
    }

    Ok(())
}

pub fn to_block_id(blkid: u64) -> BlockId {
    BlockId {
        lo: blkid as u32 as i32,
        hi: (blkid >> 32) as u32 as i32,
    }
}

pub fn from_block_id(blkid: BlockId) -> u64 {
    blkid.lo as u64 + ((blkid.hi as u64) << 32)
}

fn init_storage_bar(ui_handle: &AppWindow) {
    let mut empty_grey = ui_handle
        .global::<Theme>()
        .get_empty_grey()
        .as_argb_encoded()
        .to_le_bytes();
    empty_grey.rotate_left(1);

    let w = ui_handle.get_storage_bar_width() as usize;
    let buf = colour_bar::pixel_buffer(w, empty_grey[0]);
    let image = colour_bar::pixel_buffer_to_image(&buf);

    ui_handle.set_storage_bar_image(image);
}

fn redraw_storage_bar(ui_handle: &AppWindow, module: &protocol::app::LoRaModuleState) {
    let overwritable = module
        .storage_info
        .available_end
        .min(module.storage_info.overwritable_end)
        - module.storage_info.available_begin;

    let readable = module.storage_info.available_end - module.storage_info.available_begin;

    if readable > module.storage_info.total_blocks as u64 {
        tracing::warn!(
            %module.storage_info.available_begin,
            %module.storage_info.available_end,
            %module.storage_info.total_blocks,
            "available end - begin > total blocks",
        );
    }

    let mut empty_grey = ui_handle
        .global::<Theme>()
        .get_empty_grey()
        .as_argb_encoded()
        .to_le_bytes();
    empty_grey.rotate_left(1);
    let mut overwritable_color = ui_handle
        .global::<Theme>()
        .get_warn_orange()
        .as_argb_encoded()
        .to_be_bytes();
    overwritable_color.rotate_left(1);
    let mut protected_color = ui_handle
        .global::<Theme>()
        .get_active_blue()
        .as_argb_encoded()
        .to_le_bytes();
    protected_color.rotate_left(1);

    let w = ui_handle.get_storage_bar_width() as usize;
    let mut buf = colour_bar::pixel_buffer(w, empty_grey[0]);

    colour_bar::rasterize_range(
        &mut buf,
        overwritable_color,
        0,
        overwritable,
        module.storage_info.total_blocks as _,
    );
    colour_bar::rasterize_range(
        &mut buf,
        protected_color,
        overwritable,
        readable,
        module.storage_info.total_blocks as _,
    );

    let image = colour_bar::pixel_buffer_to_image(&buf);

    ui_handle.set_storage_bar_image(image);
}

fn init_download_bar(ui_handle: &AppWindow) {
    let w = ui_handle.get_download_bar_width();
    let empty_grey = ui_handle.global::<Theme>().get_empty_grey();
    let mut warn_orange = ui_handle
        .global::<Theme>()
        .get_warn_orange()
        .as_argb_encoded()
        .to_be_bytes();
    warn_orange.rotate_left(1);

    let buf = colour_bar::pixel_buffer(w as usize, empty_grey.red());
    let image = colour_bar::pixel_buffer_to_image(&buf);

    ui_handle.set_download_bar_image(image);
}

fn redraw_download_bar(ui_handle: &AppWindow, addr: LoRaAddr, db: &db::Database) {
    let total = ui_handle.global::<ModuleState>().get_storage_total();
    let begin = ui_handle
        .global::<ModuleState>()
        .get_storage_available_begin();
    let end = ui_handle
        .global::<ModuleState>()
        .get_storage_available_end();

    let begin_id = from_block_id(begin);
    let end_id = from_block_id(end);

    let regions = db
        .present_ranges(addr, begin_id, end_id)
        .unwrap()
        .iter()
        .map(|r| ((r.0 - begin_id) as u32, (r.1 - begin_id) as u32))
        .collect::<Vec<_>>();

    let w = ui_handle.get_download_bar_width();
    let empty_grey = ui_handle.global::<Theme>().get_empty_grey();
    let mut remote_only_color = ui_handle
        .global::<Theme>()
        .get_remote_only()
        .as_argb_encoded()
        .to_be_bytes();
    remote_only_color.rotate_left(1);
    let mut downloaded_color = ui_handle
        .global::<Theme>()
        .get_downloaded()
        .as_argb_encoded()
        .to_be_bytes();
    downloaded_color.rotate_left(1);

    let mut buf = colour_bar::pixel_buffer(w as usize, empty_grey.red());

    let blocks = end_id - begin_id;
    colour_bar::rasterize_range(&mut buf, remote_only_color, 0, blocks, total as _);
    colour_bar::rasterize_ranges(&mut buf, downloaded_color, &regions, total as _);
    let image = colour_bar::pixel_buffer_to_image(&buf);

    ui_handle.set_download_bar_image(image);
}

#[derive(Debug)]
pub enum DumpEvent {
    CancelQueue,
    QueueDumps(Vec<(u64, u64)>),
    ClearToSendDumpRequest(LoRaAddr),
}

fn dump_event_handler(events: Receiver<DumpEvent>, commands: Sender<LoraCommand>) {
    let _span = info_span!("dump_events_handler").entered();

    let mut queue = Vec::new();

    debug!("Listening for dump events.");
    while let Ok(event) = events.recv() {
        debug!(?event, "Got Dump Event");

        match event {
            DumpEvent::CancelQueue => queue = Vec::new(),
            DumpEvent::QueueDumps(items) => {
                queue = items;
            }
            DumpEvent::ClearToSendDumpRequest(addr) => {
                if let Some(range) = queue.pop() {
                    let cmd = protocol::app::LoRaCmd::StartDataDump(StartDataDump {
                        from_block_incl: range.0,
                        to_block_incl: range.1,
                    });

                    let mut buf = [0u8; 32];
                    let mut len = 0;
                    cmd.serialize(&mut buf, &mut len).unwrap();

                    commands
                        .send(LoraCommand(addr, buf[..len].to_vec()))
                        .unwrap();
                }
            }
        }
    }
}

fn update_addresses(db_conn: &db::Database, ui: &AppWindow) {
    let addresses = db_conn.list_addresses().unwrap();

    let addresses = iter::once("Select...".to_shared_string())
        .chain(addresses.iter().map(|a| a.to_shared_string()));

    ui.global::<GlobalAppState>()
        .set_none_and_addresses(ModelRc::new(VecModel::from_iter(addresses)));
}

fn format_bytes(blocks: u64) -> String {
    const KIB: u64 = 1024;
    const MIB: u64 = 1024 * KIB;
    const GIB: u64 = 1024 * MIB;

    let bytes = blocks * BLOCK_SIZE;

    let (value, unit) = if bytes >= GIB {
        (bytes as f64 / GIB as f64, "GiB")
    } else if bytes >= MIB {
        (bytes as f64 / MIB as f64, "MiB")
    } else if bytes >= KIB {
        (bytes as f64 / KIB as f64, "KiB")
    } else {
        (bytes as f64, "B  ")
    };

    let formatted = if value >= 100.0 {
        format!("{:.1}", value)
    } else if value >= 10.0 {
        format!("{:.2}", value)
    } else {
        format!("{:.3}", value)
    };

    format!("{} {}", formatted, unit)
}

/// Formats a fixed-point coordinate (1e-7 degrees/unit) as "DDD° MM' SS.S""
pub fn format_dms(value: i32, is_lat: bool) -> String {
    let max = if is_lat {
        90_0000000i32
    } else {
        180_0000000i32
    };
    assert!(value.abs() <= max, "coordinate out of range: {}", value);

    let hemi = match (is_lat, value >= 0) {
        (true, true) => 'N',
        (true, false) => 'S',
        (false, true) => 'E',
        (false, false) => 'W',
    };

    let abs = value.unsigned_abs();
    let total_deciseconds = (abs as u64 * 36 / 10_000) as u32;

    let ds = total_deciseconds % 10;
    let sec = (total_deciseconds / 10) % 60;
    let min = (total_deciseconds / 600) % 60;
    let deg = total_deciseconds / 36_000;

    if is_lat {
        format!("{:02}° {:02}' {:02}.{}\" {}", deg, min, sec, ds, hemi)
    } else {
        format!("{:03}° {:02}' {:02}.{}\" {}", deg, min, sec, ds, hemi)
    }
}
