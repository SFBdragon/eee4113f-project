use std::env;
use std::path::Path;
use std::path::PathBuf;
use std::process::Command;

const SOURCE_DIR: &str = "../../STM32-SW/L4/Core/Src";

const C_SOURCE_FILES: [&str; 1] = ["protocol.c"];

const INCLUDE_DIR: &str = "../../STM32-SW/L4/Core/include";

fn main() {
    build_protocol_static();

    let sources: Vec<_> = C_SOURCE_FILES
        .into_iter()
        .map(|f| Path::new(SOURCE_DIR).join(f))
        .collect();

    // Ensure the build script re-runs if any of the files are modified.
    for c_file in sources.iter() {
        println!("cargo::rerun-if-changed={}", c_file.display());
    }

    // Build the C file(s) as object file(s) and statically link.
    cc::Build::new()
        .files(sources.iter())
        .include(INCLUDE_DIR)
        .flag("-std=c11")
        .flag("-Wall")
        .flag("-Wextra")
        .flag("-Wno-unused-function")
        .compile("stm");
}

fn build_protocol_static() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let sub_crate_path = manifest_dir.join("../control-protocol-static");

    // Run cargo build on the sub-crate
    let profile = env::var("PROFILE").unwrap(); // "debug" or "release"
    let profile_arg = if profile == "release" {
        Some("--release")
    } else {
        None
    };
    let status = Command::new("cargo")
        .env_remove("CARGO_ENCODED_RUSTFLAGS")
        .env("RUSTFLAGS", "-C panic=abort")
        .arg("build")
        .arg("--features")
        .arg("for-rust")
        .arg("--manifest-path")
        .arg(sub_crate_path.join("Cargo.toml"))
        .arg("--target-dir")
        .arg(&out_dir)
        .args(profile_arg)
        .status()
        .expect("Failed to build sub-crate");

    if !status.success() {
        panic!("Sub-crate build failed");
    }

    // Locate the resulting lib
    // Cargo puts these in ../target/release/ or ../target/debug/
    // relative to the sub-crate's manifest.
    let target_dir = out_dir.join(&profile);
    let lib_name = "control_protocol_static";

    // Tell Cargo where to find the library
    println!("cargo:rustc-link-search=native={}", target_dir.display());
    println!("cargo:rustc-link-lib=static={}", lib_name);

    // Re-run this script if the sub-crate source changes
    println!(
        "cargo:rerun-if-changed={}",
        sub_crate_path.join("src").display()
    );
}
