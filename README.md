# Project git repos
- git@github.com:truongdo/fsi-chamcong.git - admin/web user app
- git@github.com:truongdo/smart-attendance-firmware.git - firmware
- git@github.com:truongdo/smart-attendance-app.git - react native app

# Smart Attendance ESP32 (ESP-IDF) — BLE Base Project

This is a minimal **Espressif ESP-IDF** project that boots an ESP32 as a **BLE peripheral**:

- Advertises as **`SmartAttendance`**
- Exposes 1 custom 128-bit **GATT service**
- Exposes 1 **read/write characteristic** (default value: `hello`)

## Prereqs

- ESP-IDF installed and exported (so `idf.py` works and `IDF_PATH` is set)
- An ESP32 board connected over USB

## Build + flash

```bash
idf.py set-target esp32
idf.py build
idf.py merge-bin
idf.py -p /dev/tty.usbserial-XXXX flash monitor # or simply idf.py flash
# esptool.py write_flash 0x0 build/merged-binary.bin

```

## Connect from another device

- **iOS**: use “nRF Connect” app → scan → connect to `SmartAttendance` → find the custom service → read/write the characteristic
- **Android**: “nRF Connect” app works the same way
- **macOS**: use any BLE explorer (or write a small script) to connect and write bytes

When you write to the characteristic, the ESP32 logs a hex dump.

## GATT UUIDs

Service UUID (128-bit): `08f7e6d5-c4b3-a291-807f-6e5d4c3b2a7e`  
Characteristic UUID (128-bit): `faebdccd-beaf-9081-7263-544536271809`

