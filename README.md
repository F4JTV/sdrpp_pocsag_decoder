# SDR++ POCSAG Decoder

A POCSAG pager decoder module for [SDR++](https://github.com/AlexandreRouma/SDRPlusPlus),
supporting all three standard data rates (512 / 1200 / 2400 baud) with full
BCH(31,21) forward error correction.

This is a complete rewrite of the unfinished `pager_decoder` draft that ships
in the SDR++ source tree.

## Features

- **All three POCSAG baud rates** with on-the-fly switching: 512, 1200, 2400.
- **Full BCH(31,21) forward error correction** with even-parity check.
  Corrects up to 2 bit errors per codeword (the theoretical maximum for
  BCH(31,21)) and refuses to silently miscorrect codewords with more errors.
- **Auto / Numeric / Alphanumeric** decode modes. In *Auto*, function bits
  drive the choice as per ITU-R M.584-2.
- **Inverted FSK polarity** toggle for non-standard networks.
- **Audio-bandwidth low-pass filter** (modeled on the radio module's NFM
  Low Pass option). Enabled by default. Significantly improves decoding at
  low SNR by removing high-frequency noise before the matched filter and
  symbol decision.
- **Detached messages window** with:
  - Timestamp, CAPCODE, function bits, decode type, FEC counters, content
  - "Hide errors" filter (default on) — hides messages with any
    uncorrectable codeword
  - "Hide tone-only" filter (default on) — hides legitimate tone-only
    pages *and* the noise-induced false-positive empty messages that any
    BCH-FEC decoder produces at very low SNR
  - Auto-scroll
  - One-click TSV snapshot export
- **Optional real-time logging** to a TSV file, with a folder picker using
  the native file dialog.
- **Configurable VFO snap interval**: 1 Hz, 10 Hz, 100 Hz, 1 kHz, 2.5 kHz,
  6.25 kHz, 12.5 kHz, 25 kHz (default 1 kHz).
- **Per-instance persistent settings**: baudrate, decode mode, polarity,
  snap interval, filters, log folder.
- **Soft-decision symbol diagram** for tuning aid.
- No external dependencies beyond what SDR++ itself requires.

## Building

### Ubuntu 24.04 / 24.10

Install the SDR++ build dependencies:

```bash
sudo apt install --no-install-recommends \
    build-essential cmake pkg-config \
    libfftw3-dev libglfw3-dev libvolk-dev libzstd-dev
```

Replace the stock `decoder_modules/pager_decoder` folder in the SDR++ source
tree with this one, then build:

```bash
git clone https://github.com/AlexandreRouma/SDRPlusPlus.git
cd SDRPlusPlus
rm -rf decoder_modules/pager_decoder
cp -r /path/to/this/pager_decoder decoder_modules/

mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOPT_BUILD_PAGER_DECODER=ON
make -j$(nproc)
sudo make install
```

The compiled module is at `build/decoder_modules/pager_decoder/pager_decoder.so`.

### Windows 11

Prerequisites:

- [CMake](https://cmake.org) (>= 3.13)
- [vcpkg](https://vcpkg.io) with `fftw3:x64-windows`, `glfw3:x64-windows`,
  `zstd:x64-windows`
- [PothosSDR](https://github.com/pothosware/PothosSDR) installed in
  `C:\Program Files\PothosSDR` (used by SDR++ for VOLK)
- Visual Studio 2019 or later with the C++ Desktop workload

From a Developer PowerShell:

```powershell
git clone https://github.com/AlexandreRouma/SDRPlusPlus.git
cd SDRPlusPlus
Remove-Item -Recurse -Force decoder_modules\pager_decoder
Copy-Item -Recurse path\to\this\pager_decoder decoder_modules\

mkdir build
cd build
cmake .. "-DCMAKE_TOOLCHAIN_FILE=C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake" `
         -DOPT_BUILD_PAGER_DECODER=ON `
         -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release --target pager_decoder
```

The compiled module is at
`build\decoder_modules\pager_decoder\Release\pager_decoder.dll`.

To use the freshly built DLL inside the SDR++ development root, copy it into
`root_dev\modules\` and add an entry to `"modules"` in `root_dev\config.json`.
For an installed SDR++, drop the DLL into the install's `modules\` folder and
add an instance via the Module Manager.

## Usage

1. In SDR++, open the Module Manager and create an instance of
   `pager_decoder`.
2. A new VFO appears on the waterfall. Tune it to the POCSAG channel
   (typically 12.5 kHz wide).
3. In the module's menu panel, pick the right **Baudrate**:
   - Most public networks use 1200 baud
   - Some emergency-services networks and DAPNET-style links use 512 baud
   - A few commercial systems use 2400 baud
4. Pick a **Snap** that matches your channel grid (1 kHz is a good general
   default, 12.5 kHz steps directly between adjacent pager channels).
5. Leave **Decode** on *Auto* unless you know the network uses non-standard
   function-bit semantics.
6. If you see synchronization but the output is garbled, try the
   **Invert FSK** checkbox. Some networks transmit reversed FSK polarity.
7. Click **Show Messages** to open the decoded-messages window.

## Reading the FEC column

| Value | Meaning |
| --- | --- |
| `OK` | The whole message decoded with no bits corrected. |
| `+N` (green) | The message decoded after BCH corrected `N` bits total. |
| `N/M` (red) | `N` bits were corrected, `M` codewords were uncorrectable. Only visible when "Hide errors" is unchecked. |

## Live logging vs. TSV snapshot

The module offers two distinct ways to save messages to disk:

| Action | Frequency | Filename | Use when |
| --- | --- | --- | --- |
| **Log to file** | Real-time, appended as each message arrives | `<folder>/pocsag_log.tsv` (always the same file) | You want a continuous transcript across the whole session. |
| **Save as TSV** | One-shot snapshot of the messages currently in memory | `<folder>/pocsag_YYYYMMDD_HHMMSS.tsv` (timestamped, never overwritten) | You want to freeze a snapshot of what you're seeing, or split your session into separate files. |

Both honor the "Hide errors" and "Hide tone-only" filters, so what you save
matches what you see in the table.

## POCSAG technical reference

| Parameter | Value |
| --- | --- |
| Modulation | 2-FSK, ±4.5 kHz deviation |
| Bit convention | Low tone = 1, high tone = 0 |
| Sync word | `0x7CD215D8` (32 bits) |
| Idle codeword | `0x7A89C197` (32 bits) |
| Codeword layout | 1 type bit + 20 data bits + 10 BCH bits + 1 parity bit |
| BCH polynomial | `x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1` (`0x769`) |
| BCH correction capability | up to 2 bit errors per codeword |
| Frames per batch | 8 (16 codewords) |
| Frame addressing | bottom 3 bits of CAPCODE select the frame |

## A note on false-positive empty messages

POCSAG uses BCH(31,21), which can correct up to 2 bit errors per 32-bit
codeword. At very low SNR, pure noise occasionally happens to land within
Hamming distance 2 of a valid address codeword and gets "corrected" into one.
When no message codewords follow, this would otherwise show up in the
decoded list as a tone-only page with an essentially random CAPCODE (often
very low: 0, 1, 2, ...).

This decoder counts how many message codewords actually follow the address
codeword. When the count is zero the message is classified as
`MESSAGE_TYPE_TONE_ONLY` regardless of what the function bits originally
suggested. The "Hide tone-only" filter (default on) then suppresses these
from the table, the live log, and the snapshot export.

Disable the filter if you specifically want to receive legitimate tone-only
paging signals — these are rare today but still occasionally seen on older
beeper networks.

## License

Released under [GPL-3.0](LICENSE), matching SDR++.
