# Tubes SKM + Mikro (PlatformIO / STM32F411)

Repo ini berisi proyek kontrol berbasis STM32F411 (Black Pill) dengan pendekatan *register-level* (framework `stm32cube`) untuk kebutuhan:

- **Mikroprosesor & Antarmuka**: fokus ke konfigurasi peripheral (UART, timer, interrupt, PWM, dsb).
- **Sistem Kendali & Mekanika (SKM)**: fokus ke perancangan dan evaluasi sistem kendali *closed-loop* (baseline saat ini: kontrol P; pengembangan PID dilakukan di branch `SKM`).

## Ringkas Proyek

- **Variabel terukur**: temperatur (sensor DS18B20 di `PA0`).
- **Aktuator**: PWM ke motor kipas (TIM1_CH1 di `PA8`).
- **Kontrol**: setpoint temperatur via tombol, output berupa duty PWM kipas.
- **Telemetry**: UART untuk logging (CSV) agar mudah dipakai analisis/grafik.

## Struktur Folder

- `src/`: source utama (`src/main.c`, modul DS18B20, PWM fan, UART, timing).
- `include/`: header project.
- `laporan/`: template dan draft laporan (Mikro & SKM).

## Build / Upload

- Build: `pio run`
- Upload DFU: `pio run -t upload`
- Monitor: `pio device monitor -b 9600`

## Catatan UART

Default UART logging memakai **USART1** di pin `PA9(TX)` / `PA10(RX)`.

## Serial Plotter (Bab 3.4)

PlatformIO tidak punya plotter built-in di CLI, tapi untuk VSCode ada ekstensi plotter seperti **Teleplot**.

Firmware bisa mengirim baris telemetry tambahan untuk plotter (diawali `>`), tanpa mengubah format CSV yang dipakai untuk analisis.

- Build & upload (contoh PI terbaik): `pio run -e genericSTM32F411CE_PI_best_plotter -t upload`
- Atau PID lengkap: `pio run -e genericSTM32F411CE_PID_full_plotter -t upload`
- Install ekstensi VSCode: `alexnesnes.teleplot`
- Buka Teleplot, pilih port, set baud `9600`, lalu connect

## Uji Open-Loop (Bab 3.1)

Mode ini menonaktifkan kontrol (tanpa PID/closed-loop) dan menyediakan 2 cara uji:

- **Sweep**: *PWM sweep* otomatis `0,20,40,60,80,100` (masing-masing 60 detik).
- **Per-duty (disarankan untuk linearitas)**: set duty dengan tombol lalu jalankan logging 60 detik, ulangi untuk tiap duty dengan kondisi awal yang sama (mis. suhu awal ~40°C).

Output UART menjadi CSV: `t,temp,duty,step`.

- Build & upload mode open-loop: `pio run -e genericSTM32F411CE_open_loop -t upload`
- Start test: tekan **dua tombol bersamaan** (PB4+PB5)
- Capture log: `pio device monitor -b 9600 --raw > open_loop.log`
- Ringkas tabel Bab 3.1: `python3 scripts/analyze_open_loop.py open_loop.log --markdown`

Mode per-duty:

- Build & upload: `pio run -e genericSTM32F411CE_open_loop_fixed -t upload`
- Pilih duty: tekan PB4 untuk berpindah `0 → 20 → 40 → 60 → 80 → 100 → 0 ...` (lihat marker `#OPEN_LOOP,DUTY,...`)
- Start run: tekan PB5 sampai muncul marker `#OPEN_LOOP,START,...`, lalu tunggu marker `#OPEN_LOOP,DONE,...` (±60 detik)

## Step Response (Bab 3.2)

Contoh build environment untuk 3–4 variasi tuning (lihat `platformio.ini`):

- `genericSTM32F411CE_P_cons` (P konservatif)
- `genericSTM32F411CE_P_aggr` (P agresif)
- `genericSTM32F411CE_PI_best` (PI)
- `genericSTM32F411CE_PID_full` (PID lengkap + filter derivatif)
