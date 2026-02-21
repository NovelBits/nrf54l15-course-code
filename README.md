# nRF54L15 Deep Dive - Course Code

Code samples for the **nRF54L15 Deep Dive** course from [Novel Bits Academy](https://academy.novelbits.io).

## Repository Structure

Each phase includes:
- **`starter/`** - Starting template for the lesson
- **`solution/`** - Complete working solution

```
nrf54l15-course-code/
├── phase0_raw_adc/          # Module 2 - Reading the Pulse Sensor
│   ├── starter/
│   └── solution/
├── phase1_bpm_detection/    # Module 3 - Detecting Heartbeats
│   ├── starter/
│   └── solution/
├── phase2_nrfx_saadc/       # Module 4 - Direct Hardware Control (nrfx)
│   ├── starter/
│   └── solution/
├── phase3_timer_dppi/       # Module 4 - Autonomous Sampling (TIMER+DPPI)
│   ├── starter/
│   └── solution/
├── phase4_grtc_idle/        # Module 5 - Idle Mode with GRTC (Burst Sampling)
│   ├── starter/
│   └── solution/
├── phase5_ble_hrs/          # Module 6 - Bluetooth LE Heart Rate Monitor
│   ├── starter/
│   └── solution/
└── lib/
    └── pulse_sensor/        # Shared PulseSensor library
```

## Hardware Requirements

- **nRF54L15 DK** (PCA10156)
- **PulseSensor** (https://pulsesensor.com)
- **3 jumper wires**

## Wiring

| PulseSensor Pin | nRF54L15 DK Pin |
|-----------------|-----------------|
| Signal (Purple) | P1.11 (AIN4)    |
| VCC (Red)       | 3.3V            |
| GND (Black)     | GND             |

## Software Requirements

- **nRF Connect SDK** v2.9.0 or later
- **Segger J-Link** drivers
- **VS Code** with nRF Connect extension (recommended)

## How to Use

### Option 1: Download ZIP
1. Click the green **Code** button above
2. Select **Download ZIP**
3. Extract to your workspace

### Option 2: Clone with Git
```bash
git clone https://github.com/NovelBits/nrf54l15-course-code.git
cd nrf54l15-course-code
```

### Build and Flash

Each phase is a complete Zephyr application. Use VS Code with the nRF Connect extension:

1. Open **VS Code**
2. Go to the **nRF Connect** extension panel
3. Click **Add an existing application** and select the phase folder (e.g., `phase0_raw_adc/starter`)
4. Click **Add build configuration**
   - Board: `nrf54l15dk/nrf54l15/cpuapp`
   - Leave other settings at defaults
5. Click **Build** (hammer icon)
6. Connect your DK via USB
7. Click **Flash** (lightning icon)
8. View output in the **nRF Terminal** or your preferred serial terminal (115200 baud)

## Support

Questions about the course? Email: support@novelbits.io

---

© 2026 Novel Bits LLC. All rights reserved.
