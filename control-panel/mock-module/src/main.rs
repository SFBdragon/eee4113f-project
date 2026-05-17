// src/main.rs
//
// STM32L4 integration test rig.

mod control_impl;
mod ffi;
mod injector;
mod netio_impl;
mod state;

use std::net::UdpSocket;
use std::time::Duration;

const WIFI_RELIABILITY_TEST: bool = false;
const WIFI_RECV_DROP_RATE: f64 = 0.33;
const WIFI_SEND_DROP_RATE: f64 = 0.33;

const WIFI_CORRUPT_TEST: bool = false;
const WIFI_RECV_BITFLIP_RATE: f64 = 0.0001;
const WIFI_SEND_BITFLIP_RATE: f64 = 0.0001;

const LORA_RELIABILITY_TEST: bool = true;
const LORA_RECV_DROP_RATE: f64 = 0.33;
const LORA_SEND_DROP_RATE: f64 = 0.33;

const LORA_CORRUPT_TEST: bool = true;
const LORA_RECV_BITFLIP_RATE: f64 = 0.0001;
const LORA_SEND_BITFLIP_RATE: f64 = 0.0001;

fn main() {
    let lora_sock = UdpSocket::bind("127.0.0.1:12001").unwrap();
    lora_sock.set_nonblocking(true).unwrap();
    let wifi_sock = UdpSocket::bind("127.0.0.1:12011").unwrap();
    wifi_sock.set_nonblocking(true).unwrap();

    println!(
        "[stm32_sim] binding LoRa socket -> {}",
        lora_sock.local_addr().unwrap()
    );
    println!(
        "[stm32_sim] binding WiFi socket -> {}",
        lora_sock.local_addr().unwrap()
    );

    netio_impl::init_sockets(lora_sock, wifi_sock);

    // Kick off the block injector thread.
    // std::thread::spawn(injector::run_injector);

    {
        // Seed initial state for ATPs.
        let lock = state::sim();
        let mut state = lock.lock().unwrap();

        let block = (0..200u8).into_iter().collect::<Vec<_>>();
        for i in 0..100 {
            state.append_block(&block).unwrap();
        }
    }

    eprintln!("[stm32_sim] calling protocol_init()");
    unsafe { ffi::protocol_init() };
    eprintln!("[stm32_sim] protocol_init() returned — entering event loop");
    println!("Type 'help' for commands.");

    let poll_interval = Duration::from_millis(10);

    loop {
        // eprintln!("before lora");
        // Poll sockets.
        netio_impl::poll_lora_recv();
        // eprintln!("after lora");
        netio_impl::poll_wifi_recv();
        // eprintln!("after wifi");

        // Fire expired timers.
        control_impl::tick_timers();
        // eprintln!("after tick_timers");

        // // Handle any stdin command.
        // match rx.try_recv() {
        //     Ok(line) => {
        //         if !handle_command(&line) {
        //             println!("bye");
        //             break;
        //         }
        //         print!("> ");
        //         io::stdout().flush().ok();
        //     }
        //     Err(std::sync::mpsc::TryRecvError::Empty) => {}
        //     Err(std::sync::mpsc::TryRecvError::Disconnected) => break,
        // }

        // eprintln!("after try recv");

        std::thread::sleep(poll_interval);
    }
}
