#!/bin/bash

rustup run nightly cargo run -p mock-module

# or
# RUSTFLAGS="-Clink-args=-Wl,--allow-multiple-definition" cargo run -p mock-module
