
slint::include_modules!();

fn main() {
    let ui = AppWindow::new().unwrap();

    ui.set_wifi_connected(true);

    ui.run().unwrap();
}
