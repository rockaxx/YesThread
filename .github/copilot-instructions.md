## Quick context

This repository is an ESP-IDF example that implements a small custom 802.15.4-based protocol ("KroschuThread") on Espressif chips (ESP32-C5/C6/H2). The runtime entry is `main/extreme.c`. The custom protocol lives under `components/kroschuthread_protocol/` and depends on the platform `ieee802154` driver and `esp_hw_support` component.

## Big-picture architecture (what matters to changes)

- `main/extreme.c`: demo app that initializes NVS, configures the protocol and starts either a sender or receiver FreeRTOS task. Change `DEMO_MODE` here to toggle behavior.
- `components/kroschuthread_protocol/`: core implementation. Important files:
  - `include/kroschuthread_protocol.h` — protocol constants, frame structs, API (init/send/process/get_stats).
  - `kroschuthread_protocol.c` — high-level protocol logic: encapsulation, decapsulation, stats API.
  - `kroschuthread_radio.c` — radio/driver glue to `ieee802154`.
  - `kroschuthread_receiver.c` — receive/ACK handling and processing loop.
  - `kroschuthread_nodeid.c` and `include/kroschuthread_nodeid.h` — node id table, NVS persistence and MAC-derived self id (call `nodeid_init_from_mac()` to set self id).

Cross-component notes: components use `idf_component_register()` and declare `REQUIRES` on `ieee802154`, `esp_hw_support` and core ESP-IDF services. Changing those dependencies requires updating the component CMakeLists.

## Developer workflows (build, flash, monitor)

- Configure: `idf.py menuconfig` (use `Component config → ESP System Settings` for console selection).
- Build + Flash + Monitor: `idf.py -p PORT build flash monitor` — this is the standard flow used in the README and by CI artifacts already present in `build/`.
- NVS: the app initializes NVS in `app_main()`; node-table and PHY calibration values live in NVS via functions in the nodeid code. Re-flashing may require `nvs_flash_erase()` if schema changes.

## Project-specific coding patterns & conventions

- Protocol constants are defined as macros in `include/kroschuthread_protocol.h` (e.g. `KROSCHUTHREAD_CHANNEL`, `KROSCHUTHREAD_DATA_PORT`, `KROSCHUTHREAD_MAX_PAYLOAD_SIZE`). Prefer changing values there and rebuilding rather than scattering magic numbers.
- Frames use packed structs (`__attribute__((packed))`) — editing these structs requires careful attention to sizes and `KROSCHUTHREAD_MAX_FRAME_SIZE`.
- Callbacks: consumers register `data_callback` and `ack_callback` via `kroschuthread_config_t` passed to `kroschuthread_protocol_init()`. The demo in `main/extreme.c` shows typical usage.
- API shape: the component exposes init/deinit, send (fire-and-forget variant `kroschuthread_protocol_send_data_no_ack`), process loop `kroschuthread_protocol_process()`, and stats getters. Use the process API in long-running tasks rather than busy-waiting.
- Logging: use ESP_LOG* with tags. The demo uses tag "KroschuThreadMain" — search for that when debugging demo flows.

## Testing, debugging and common edits (concrete examples)

- To switch to receiver mode: edit `main/extreme.c` and change the `DEMO_MODE` macro to `DEMO_MODE_RECEIVER`, then run `idf.py -p PORT build flash monitor`.
- To change the radio channel or ports: update `KROSCHUTHREAD_CHANNEL` / `KROSCHUTHREAD_DATA_PORT` in `components/kroschuthread_protocol/include/kroschuthread_protocol.h` and rebuild.
- To add a new exported helper or change a frame field:
  - Update the header in `include/kroschuthread_protocol.h`.
  - Update serialization/CRC logic in `kroschuthread_protocol.c` (see `kroschuthread_encapsulate_frame` / `kroschuthread_decapsulate_frame`).
  - Run `idf.py build` and test on hardware; monitor logs for CRC or invalid frame errors.

## Integration & side-effects to watch for

- NVS usage: `node_table_save_to_nvs()` / `node_table_load_from_nvs()` persist node table. Tests that change persisted formats must migrate or clear NVS.
- TX power and timing: `KROSCHUTHREAD_TX_POWER` is defined in header; high values (20 dBm) are used — hardware/platform limits must be respected.
- Concurrency: protocol uses FreeRTOS tasks and callbacks. Avoid long-blocking operations inside callbacks (they run in application context).

## Where to look first when a change breaks the board

1. Serial monitor logs (`idf.py monitor`) — tags: `KroschuThreadMain`, and component-specific logs in `components/kroschuthread_protocol`.
2. Check CRC / invalid frame counters (exposed by `kroschuthread_protocol_get_stats()` and printed in demo receiver).
3. Ensure NVS is initialized (see `nvs_flash_init()` in `app_main()`) — corrupted NVS often manifests as calibration / radio failures.
4. If radio behavior changes, inspect `kroschuthread_radio.c` and the `ieee802154` glue.

## Minimal contributor contract (what changes should do)

- Inputs: C edits to `components/kroschuthread_protocol` or `main/*`.
- Outputs: pass `idf.py build`, no runtime CRC errors for the sample messages, and logs show correct init sequence.
- Error modes to notice: CRC errors, timeouts (`KROSCHUTHREAD_STATUS_TIMEOUT`), buffer-full (`KROSCHUTHREAD_STATUS_BUFFER_FULL`). These are defined in the header and surfaced in the demo logs.

---

If you'd like, I can iterate and add examples for patching a specific file (for example adding a new control frame type) or include a short checklist for CI runs. Any missing project details I should add (ESP-IDF version, target flash script, or test hardware)?
