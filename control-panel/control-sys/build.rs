use std::path::Path;


const SOURCE_DIR: &str = "../../laptop-drivers/src";

const C_SOURCE_FILES: [&str; 1] = [
    "lib.c",
];

const INCLUDE_DIR: &str = "../../laptop-drivers/include";

fn main() {
    let sources: Vec<_> = C_SOURCE_FILES
        .into_iter()
        .map(|f| Path::new(SOURCE_DIR).join(f))
        .collect();

    // Ensure the build script re-runs if any of the files are modified.
    for c_file in sources.iter() {
        println!("cargo::rerun-if-changed={}", c_file.display());
    }
    
    // Build the C files as object files and statically link them.
    cc::Build::new()
        .files(sources.iter())
        .include(INCLUDE_DIR)
        .compile("nicdrivers");
}