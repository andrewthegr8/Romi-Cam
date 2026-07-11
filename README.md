# Romi Pose Tracking System

This repository contains the deployment code and support files for a Romi pose tracking system. It is written for future students who need to run, maintain, or extend the project.

The system uses a Raspberry Pi camera setup to detect ArUco markers, sends pose packets from the Raspberry Pi to an ESP32 transmitter over SPI, broadcasts those packets over ESP-NOW, receives them on an ESP32 mounted on the Romi, and serves requested pose data to the STM32 over UART.

```text
Raspberry Pi + Camera
        |
        | SPI
        v
ESP32 Transmitter
        |
        | ESP-NOW broadcast
        v
ESP32 Receiver on Romi
        |
        | UART request/response
        v
STM32 on Romi
```

For background and validation details, see the thesis document associated with this project. This README focuses only on code deployment, repository contents, and physical device connections. Earlier Raspberry Pi development code is available in the related development repository: [RomiCam](https://github.com/andrewthegr8/RomiCam).

---

## Repository Layout

```text
ROMI-CAM/
├── ESP Receiver/
│   └── main.c
├── ESP Transmitter/
│   └── main.c
├── Raspberry Pi/
│   ├── D.json
│   ├── K.json
│   ├── RomiTrackerV9_helper.py
│   └── RomiTrackerV9.py
├── STM Code/
│   └── ESP_Comm.py
├── Support Files/
│   ├── CAD/
│   │   ├── NewCameraMount.SLDASM
│   │   ├── NewCameraMount.STL
│   │   ├── romi-ARUCO-mount.SLDASM
│   │   ├── romi-chassis-kit.step
│   │   ├── RomiMountv1.3MF
│   │   ├── threads.SLDPRT
│   │   └── threads.STL
│   ├── RealMarkerBoard.ai
│   └── RealMarkerBoard.pdf
└── README.md
```

### File and Folder Purposes

| Path | Purpose |
| --- | --- |
| `ESP Receiver/main.c` | ESP32 firmware for the board mounted on Romi. It receives ESP-NOW pose packets and responds to STM32 UART pose requests.  |
| `ESP Transmitter/main.c` | ESP32 firmware for the board connected to the Raspberry Pi. It acts as an SPI peripheral and broadcasts received pose packets over ESP-NOW. |
| `Raspberry Pi/RomiTrackerV9.py` | Main Raspberry Pi runtime script. It initializes the camera, runs the tracking callback, prints pose data to the terminal, and sends pose packets over SPI. |
| `Raspberry Pi/RomiTrackerV9_helper.py` | Helper code for calibration-file loading, ArUco detection setup, marker-to-world coordinate conversion, heading calculation, terminal display formatting, and packet construction. |
| `Raspberry Pi/K.json` | Camera intrinsic matrix used by the Raspberry Pi tracker. Keep this file in the Raspberry Pi working directory. |
| `Raspberry Pi/D.json` | Camera distortion coefficients used by the Raspberry Pi tracker. Keep this file in the Raspberry Pi working directory. |
| `STM Code/ESP_Comm.py` | MicroPython helper class used on the STM32 to request pose data from the ESP32 receiver over UART. |
| `Support Files/CAD/` | Mechanical support files for the Romi marker mount and Raspberry Pi/camera mounting hardware. |
| `Support Files/RealMarkerBoard.ai` | Adobe Illustrator version of the final calibration/marker pattern. |
| `Support Files/RealMarkerBoard.pdf` | PDF version of the final calibration/marker pattern. |

---

## Hardware Used by the Deployment

This deployment uses the following device roles:

| Device | Role |
| --- | --- |
| Raspberry Pi with camera | Captures frames, detects markers, builds pose packets, and sends packets over SPI. |
| ESP32-WROOM-32 development board, transmitter | Receives SPI data from the Raspberry Pi and broadcasts the packet using ESP-NOW. |
| ESP32-WROOM-32 development board, receiver | Receives ESP-NOW packets on the Romi and serves the latest pose data to the STM32 over UART. |
| STM32 on Romi | Requests pose data for a given Romi marker ID over UART. |
| Final marker board / calibration pattern | Fixed ArUco marker pattern used by the Raspberry Pi code to locate the world frame. |

### Power and Logic Levels

| Device | Power Source Used in This Deployment |
| --- | --- |
| Raspberry Pi | USB-C power supply. |
| ESP32 transmitter | USB cable from the Raspberry Pi. |
| ESP32 receiver on Romi | 5 V output from the STM32 board. |
| STM32 / Romi | Romi/STM32 power system. |

All SPI and UART logic connections are 3.3 V logic. Tie grounds together for every wired interface. Do not connect 5 V logic directly to SPI or UART signal pins.

---

## Raspberry Pi Software Setup

The Raspberry Pi code was developed on:

| Component | Known Working Version / Source |
| --- | --- |
| OS | Debian GNU/Linux 12 Bookworm |
| Python | 3.11.2 |
| `cv2` | 4.9.0 |
| `numpy` | 1.24.2 |
| `picamera2` | 0.0.31 |
| `spidev` | 3.8 |
| `gpiozero` | 2.0.1 |

The script was developed with older versions of some libraries. If you don't use versions close to these, be prepared to retest the camera and SPI pipeline.

### Install Python Dependencies

APT packages are preferred when available, especially for libraries that interface with Raspberry Pi hardware.

From the repository root on the Raspberry Pi:

```bash
sudo apt update
sudo apt install python3 python3-venv python3-pip python3-numpy python3-opencv python3-picamera2 python3-gpiozero python3-spidev
```

Create a virtual environment that can still see system-installed Raspberry Pi packages:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
```

If `spidev` is not available from APT on the target image, install it inside the virtual environment:

```bash
python -m pip install "spidev==3.8"
```

### Enable Raspberry Pi Interfaces

The tracker opens SPI device `/dev/spidev0.0`, which corresponds to SPI0 with CE0. Enable SPI on the Raspberry Pi before running the tracker.

The code uses `picamera2`, so the attached camera must be visible to the Raspberry Pi camera/libcamera stack.

### Required Runtime Files and Folders

Run the tracker from inside the `Raspberry Pi/` folder because `K.json`, `D.json`, and `debug_pics/` are referenced with relative paths.

```bash
cd "Raspberry Pi"
mkdir -p debug_pics
```

Keep these files in the same folder as `RomiTrackerV9.py`:

```text
Raspberry Pi/
├── D.json
├── K.json
├── RomiTrackerV9_helper.py
└── RomiTrackerV9.py
```

The `debug_pics/` folder is required because the helper writes initialization images into it.

---

## Raspberry Pi Tracker Runtime

Start the tracker from the `Raspberry Pi/` folder:

```bash
source ../.venv/bin/activate
python RomiTrackerV9.py
```

At startup, the script:

1. Starts the camera using `picamera2`.
2. Captures an initialization frame.
3. Loads `K.json` and `D.json`.
4. Writes initialization images into `debug_pics/`.
5. Detects the fixed locating markers.
6. Starts the camera callback loop.
7. Starts the SPI and terminal-printer threads.

During normal operation, the terminal displays a table with marker ID, position, heading, and frame timing data. Pose packets are sent to the ESP32 transmitter only after marker data is available.

Current Raspberry Pi runtime settings in `RomiTrackerV9.py`:

| Setting | Current Value |
| --- | --- |
| Camera main stream | `2304 x 1296`, `YUV420` |
| Camera sensor output | `2304 x 1296`, 10-bit |
| SPI device | `/dev/spidev0.0` |
| SPI mode | Mode 1, `0b01` |
| SPI clock | `6_000_000` Hz |
| Pose packet size | 163 bytes |
| SPI transfer size | 256 bytes, with the valid packet padded using zero bytes |
| Profiling GPIO in Python | GPIO 21 through `gpiozero.LED(21)` |

Stop the tracker with `Ctrl+C`. The current implementation doesn't have all threads exit gracefully, so you'll have to hit `Ctrl+C` a few times and wait for all the threads to hit an exception.

---

## Marker Board and Romi Marker Deployment

The Raspberry Pi code uses OpenCV's 4x4 ArUco dictionary with 50 available marker IDs:

```python
aruco.DICT_4X4_50
```

The fixed locating markers are hard-coded as marker IDs `0` through `10` in `RomiTrackerV9_helper.py`. The tracker ignores these IDs after initialization because they define the fixed reference pattern.

Use marker IDs `11` through `49` for Romis. The `ROMIID` used by the STM32 should match the ArUco marker ID mounted on the Romi.

Current deployment constants encoded in `RomiTrackerV9_helper.py`:

| Item | Current Value |
| --- | --- |
| Fixed locating marker IDs | `0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10` |
| Fixed locating marker width | `100.0` mm |
| Fixed locating marker coordinates | Stored in `MarkerCenters_mm` |
| Romi marker plane height used by the tracker | `130` mm |
| Current pose packet capacity | 10 Romi detections per packet |

Use the files in `Support Files/` for the physical setup:

| File / Folder | Use |
| --- | --- |
| `Support Files/RealMarkerBoard.ai` | Editable Adobe Illustrator version of the final marker board. |
| `Support Files/RealMarkerBoard.pdf` | Printable PDF version of the final marker board. (36 inches by 72 inches) - PRINT AT FULL SCALE! |
| `Support Files/CAD/romi-ARUCO-mount.SLDASM` | Romi ArUco marker mount assembly. |
| `Support Files/CAD/RomiMountv1.3MF` | 3MF version of the Romi marker mount. |
| `Support Files/CAD/NewCameraMount.SLDASM` / `.STL` | Camera/Raspberry Pi mounting hardware for the wooden stand. |
| `Support Files/CAD/romi-chassis-kit.step` | Romi chassis reference geometry. |
| `Support Files/CAD/threads.SLDPRT` / `.STL` | 3-D printable fasteners to constrain the mount of the wooden stand. |

If the marker board is printed at a different scale, the fixed marker dimensions or positions change, or the Romi marker height changes, update the corresponding constants in `RomiTrackerV9_helper.py` before running the system.

---

## ESP32 Firmware Setup

The ESP32 code was developed using ESP-IDF version 5.5 in VS Code with the Espressif extension.

Use two separate ESP32 boards:

| Board Label | Firmware Folder | Physical Location |
| --- | --- | --- |
| ESP32 Transmitter | `ESP Transmitter/main.c` | Near the Raspberry Pi. |
| ESP32 Receiver | `ESP Receiver/main.c` | Mounted on the Romi. |

The repository stores the application `main.c` files. If your local ESP-IDF workspace expects a full ESP-IDF project structure, create or open an ESP-IDF project in VS Code and place the corresponding `main.c` in that project's main application component.

Use ESP-IDF target `esp32` for ESP32-WROOM-32 development boards. Build and flash each board with the VS Code ESP-IDF workflow.

### ESP-NOW Broadcast Behavior

Both ESP32 programs use ESP-NOW on Wi-Fi channel 1. The transmitter sends to the broadcast MAC address:

```c
FF:FF:FF:FF:FF:FF
```

That is intentional. Any ESP32 receiver running the matching receiver firmware and listening on the same Wi-Fi channel can receive the pose packets. This allows multiple Romis to receive the same broadcast stream without pairing each transmitter/receiver pair by MAC address.

---

## Raspberry Pi to ESP32 Transmitter Wiring

The Raspberry Pi is the SPI controller. The ESP32 transmitter is the SPI peripheral.

| Raspberry Pi Pin | Raspberry Pi Function | ESP32 Transmitter Pin | ESP32 Function |
| --- | --- | --- | --- |
| BCM GPIO 10 | SPI0 MOSI | GPIO 23 | `GPIO_MOSI` |
| BCM GPIO 9 | SPI0 MISO | GPIO 19 | `GPIO_MISO` |
| BCM GPIO 11 | SPI0 SCLK | GPIO 18 | `GPIO_SCLK` |
| BCM GPIO 8 | SPI0 CE0 / CS | GPIO 5 | `GPIO_CS` |
| GND | Ground | GND | Ground reference |

The transmitter firmware also defines a handshake pin on ESP32 GPIO 2. The current Raspberry Pi Python script does not read a handshake GPIO, so the handshake pin is not used in this deployment.

---

## ESP32 Receiver to STM32 Wiring

The ESP32 receiver uses `UART_NUM_2` on the ESP32 side and communicates with the STM32 over UART at 115200 baud. The STM32 MicroPython setup uses `UART(3)`.

| ESP32 Receiver Pin | STM32 / MicroPython Pin | Direction | Notes |
| --- | --- | --- | --- |
| GPIO 17 | `Pin.cpu.C11` / UART3 RX | ESP32 TX to STM32 RX | ESP32 sends the 12-byte pose response. |
| GPIO 16 | `Pin.cpu.C10` / UART3 TX | STM32 TX to ESP32 RX | STM32 sends the 4-byte pose request. |
| GND | GND | Shared reference | Required. |

Power the ESP32 receiver from the STM32 board's 5 V output as used in the deployed system, but keep the UART signal lines at 3.3 V logic.

If you wish to use these exact pins for UART on the STM32, follow this MicroPython setup pattern. The `Pin.cpu.C4` and `Pin.cpu.C5` lines are included because these are the default pins for UART3 and have to be remapped after the `UART` object is instantiated if alternate pins are being used.

```python
from pyb import Pin, UART
from ESP_Comm import ESP_Comm

# ROMIID must match the ArUco marker ID mounted on the Romi.
ROMIID = 11

# Init UART object for communication with ESP32.
Pin(Pin.cpu.C10, Pin.ALT, alt=7)  # UART3 TX
Pin(Pin.cpu.C11, Pin.ALT, alt=7)  # UART3 RX
uart_esp = UART(3, 115200)

# Set pin modes back to default for UART3 remapping.
Pin(Pin.cpu.C4, mode=Pin.ANALOG)
Pin(Pin.cpu.C5, mode=Pin.ANALOG)

esp_comm = ESP_Comm(uart_esp, ROMIID)
```

---

## STM32 Pose Request Code

Copy `STM Code/ESP_Comm.py` onto the STM32 MicroPython filesystem so it can be imported by the Romi control code.

Minimal usage:

```python
pose = esp_comm.get_pose()

if pose is not None:
    x_in, y_in, heading_deg = pose
```

The included `ESP_Comm.py` helper performs this transaction:

1. Send a 4-byte UART request to the ESP32 receiver.
2. Wait 2 ms for the ESP32 receiver to process the request.
3. Read a 12-byte UART response.
4. Unpack the response as three little-endian floats.

The request bytes are:

| Byte | Value | Meaning |
| --- | --- | --- |
| 0 | `ROMIID` | Requested Romi marker ID. |
| 1 | `0x0A` | Newline terminator byte. |
| 2 | `0x0A` | Newline terminator byte. |
| 3 | `0x0A` | Newline terminator byte. |

The ESP32 receiver response is 12 bytes:

| Bytes | Type | Meaning |
| --- | --- | --- |
| 0-3 | `float32`, little-endian | `center_x` in millimeters on the wire. |
| 4-7 | `float32`, little-endian | `center_y` in millimeters on the wire. |
| 8-11 | `float32`, little-endian | Heading in degrees. |

Important unit note: the wire protocol uses millimeters for `center_x` and `center_y`, but the included `ESP_Comm.py` helper divides `x` and `y` by `25.4` before returning them. Therefore `ESP_Comm.get_pose()` currently returns `x` and `y` in inches, and heading in degrees. To work in millimeters on the STM32, remove the `/25.4` conversion in `ESP_Comm.py` or multiply the returned `x` and `y` values by `25.4`.

---

## Pose Packet Layout

The Raspberry Pi builds a fixed 163-byte pose packet before padding the SPI transfer to 256 bytes.

All multi-byte numeric values are little-endian.

| Byte Offset | Size | Type | Field | Notes |
| --- | ---: | --- | --- | --- |
| `0` | 1 byte | `uint8` | Header byte 0 | Always `0x5A`. |
| `1` | 1 byte | `uint8` | Header byte 1 | Always `0x5A`. |
| `2` | 1 byte | `uint8` | Packet depth | Number of detected Romi markers included in this packet. |
| `3 + 16*i` | 4 bytes | `float32` | Marker ID | Stored as a float in the current packet format. |
| `7 + 16*i` | 4 bytes | `float32` | Center x | Millimeters. |
| `11 + 16*i` | 4 bytes | `float32` | Center y | Millimeters. |
| `15 + 16*i` | 4 bytes | `float32` | Heading | Degrees. |

`i` is the packet entry index, currently `0` through `9` for up to 10 Romi detections per packet.

The ESP32 receiver stores the most recent pose by marker ID. When the STM32 requests `ROMIID`, the receiver returns the latest stored `(center_x, center_y, heading)` for that ID.

---

## Code Configuration Values Most Likely to Change

Future contributors should keep these values synchronized across the repository when extending the project.

| Change | Files to Inspect |
| --- | --- |
| Change the SPI pins | `ESP Transmitter/main.c`, Raspberry Pi wiring, and any Pi-side SPI setup changes. |
| Change the UART pins | `ESP Receiver/main.c`, STM32 MicroPython UART setup, and wiring. |
| Change the number of Romis per packet | `Raspberry Pi/RomiTrackerV9.py`, `Raspberry Pi/RomiTrackerV9_helper.py`, `ESP Transmitter/main.c`, `ESP Receiver/main.c`, and any STM32 parsing assumptions. |
| Change marker board geometry | `Support Files/RealMarkerBoard.*` and `MarkerCenters_mm` / `MarkerWidth_mm` in `Raspberry Pi/RomiTrackerV9_helper.py`. |
| Change camera, lens, or calibration | `Raspberry Pi/K.json`, `Raspberry Pi/D.json`, and the camera configuration in `Raspberry Pi/RomiTrackerV9.py`. |
| Change Romi marker height | The `Zw` value passed to `pixel_to_world(...)` in `Raspberry Pi/RomiTrackerV9_helper.py`. |
| Change ESP-NOW channel | `WIFI_CHANNEL` in both ESP32 source files. |
| Change the STM request format | `STM Code/ESP_Comm.py` and the UART request parsing in `ESP Receiver/main.c`. |

When changing the packet layout, update all layers together: Raspberry Pi packet builder, ESP32 transmitter packet length, ESP32 receiver unpacking, and STM32 response parsing.

---

## Normal Deployment Sequence

1. Mount the Raspberry Pi and camera on the wooden stand using the support hardware.
2. Place or attach the final marker board where the camera can see locating markers `0` through `10`.
3. Mount an ArUco marker on the Romi using the Romi marker mount.
4. Set `ROMIID` in the STM32 code to the ArUco marker ID mounted on that Romi.
5. Wire the Raspberry Pi to the ESP32 transmitter over SPI.
6. Wire the ESP32 receiver to the STM32 over UART.
7. Power the Raspberry Pi, ESP32 transmitter, Romi/STM32, and ESP32 receiver.
8. Flash the transmitter firmware to the ESP32 connected to the Raspberry Pi.
9. Flash the receiver firmware to the ESP32 mounted on the Romi.
10. Copy `STM Code/ESP_Comm.py` to the STM32 MicroPython filesystem and import it from the Romi control code.
11. Start the Raspberry Pi tracker from the `Raspberry Pi/` folder.
12. In the Romi code, call `esp_comm.get_pose()` whenever the STM32 needs the latest pose estimate.

---

## Notes for Future Development

The current code is organized around a working deployment, not a general-purpose library. The values in the Python helper, ESP32 firmware, and STM32 helper are tightly coupled to the deployed wiring, marker IDs, packet format, and calibration files. Deviations from the exact setup described here will likely require (potentially extensive) code changes to get the system working again.
