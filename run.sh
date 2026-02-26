#!/bin/bash

# runner にこれを指定すればいい気もするが、ビルドとリブートを並列に動かしたかった

PICO_MOUNT_POINT="/Volumes/RP2350"

{
    echo "reset_to_usbboot" | nc -w 1 -G 1 "$PICO_IP" 5151
    sleep 1
    while [[ ! -d "$PICO_MOUNT_POINT" ]]; do
        echo "waiting $PICO_MOUNT_POINT"
        sleep 1
    done
} &

cargo build --release
wait
cargo run --release