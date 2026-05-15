// src/main.rs
//
// STM32L4 integration test rig.

mod control_impl;
mod ffi;
mod injector;
mod netio_impl;
mod state;

use std::io::{self, BufRead, Write};
use std::net::UdpSocket;
use std::os::unix::net::UnixDatagram;
use std::time::Duration;

use ffi::Policy;
use state::sim;

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
    std::thread::spawn(injector::run_injector);

    // Call Shaun's C init.  This will call back into initialize_networking(),
    // initialize_lora(), initialize_wifi() etc., which are all resolved above.
    eprintln!("[stm32_sim] calling protocol_init()");
    unsafe { ffi::protocol_init() };
    eprintln!("[stm32_sim] protocol_init() returned — entering event loop");
    println!("Type 'help' for commands.");

    // // Event loop: poll sockets + timers on a tight sleep, handle stdin lines.
    // let stdin = io::stdin();
    // let stdin_lines = stdin.lock().lines().collect::<Vec<_>>();

    // // We need non-blocking stdin.  On Linux we can just use a separate thread.
    // let (tx, rx) = std::sync::mpsc::channel::<String>();
    // std::thread::spawn(move || {
    //     for line in stdin_lines {
    //         match line {
    //             Ok(l) => {
    //                 let _ = tx.send(l);
    //             }
    //             Err(_) => break,
    //         }
    //     }
    // });

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
