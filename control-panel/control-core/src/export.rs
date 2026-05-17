use crate::db;

use crate::protocol::LoRaAddr;
use rfd::FileDialog;
use serde_derive::Serialize;

use std::{
    fs::{self},
    io::Write,
    path::PathBuf,
    thread,
};

pub fn export(addr: LoRaAddr) {
    thread::spawn(move || {
        let selection = FileDialog::new()
            .add_filter("JSON", &["json"])
            .set_title("Export Data")
            .set_file_name("data.json")
            .save_file();

        let Some(json_path) = selection else {
            tracing::info!("User cancelled export");
            return;
        };

        let mut bin_path = json_path.clone();
        bin_path.set_extension("bin");

        tracing::info!("Exporting JSON to: {:?}", json_path);
        tracing::info!("Exporting Binary to: {:?}", bin_path);

        write_export(json_path, bin_path, addr);
    });
}

#[derive(Debug, Clone, Copy, Serialize)]
struct Entry {
    pub block_id: u64,
    pub offset: u64,
    pub length: u64,
}

fn write_export(json_p: PathBuf, bin_p: PathBuf, addr: LoRaAddr) {
    let db = db::Database::open().unwrap();
    let blocks = db.present_in_range(addr, 0, (i64::MAX - 1) as u64).unwrap();

    let mut bin_file = match fs::File::create(&bin_p) {
        Ok(bin_file) => bin_file,
        Err(err) => {
            tracing::error!(?err, ?bin_p, "Could not open binary file for export.");
            return;
        }
    };

    let mut entries = Vec::new();

    let mut pos = 0;
    for block in blocks {
        let data = db.read_block(addr, block).unwrap().unwrap();

        let entry = Entry {
            block_id: block,
            offset: pos,
            length: data.len() as u64,
        };
        entries.push(entry);

        if let Err(err) = bin_file.write_all(&data) {
            tracing::error!(?err, ?bin_p, "Failed to write to binary file.");
            return;
        }

        pos += data.len() as u64;
    }

    let json = serde_json::to_string(&entries).unwrap();

    match fs::write(&json_p, &json) {
        Ok(()) => {}
        Err(err) => {
            tracing::error!(
                ?err,
                ?json_p,
                "Export failed, could not open and write to filepath for export."
            );
            return;
        }
    }
}
