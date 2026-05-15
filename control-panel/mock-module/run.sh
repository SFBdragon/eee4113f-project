#!/bin/bash

RUSTFLAGS="-Clink-args=-Wl,--allow-multiple-definition" cargo run -p mock-module
