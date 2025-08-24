# LFR (Line Following Robot) using ESP32

A project for building a Line Following Robot (LFR) using ESP32, featuring two modes of operation:
- **PID Tuning Mode:** Tune your robot’s PID values via a web server interface.
- **Run Mode:** Load finalized PID values for optimal line following.

## Table of Contents

- [Features](#features)
- [Folder Structure](#folder-structure)
- [Hardware Requirements](#hardware-requirements)
- [Setup & Installation](#setup--installation)
- [Usage](#usage)
- [Circuit Diagram](#circuit-diagram)
- [Contributing](#contributing)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Features

- **PID Tuning Mode:**  
  Run a web server on your ESP32. Adjust and test PID parameters in real time from any web browser.
- **Run Mode:**  
  Use the tuned PID values for high-performance line following.

## Folder Structure

```
LFR/
├── pid-tuner/    # ESP32 code for PID tuning via web server
├── pid-runner/   # ESP32 code for final run using tuned PID values
├── docs/         # Documentation, schematics, etc.
└── README.md
```

## Hardware Requirements

- ESP32 Dev Board
- IR Sensors (for line detection)
- Motor Driver (e.g., L298N)
- DC Motors (x2)
- Chassis and wheels
- Jumper wires, breadboard (or PCB)
- Power supply (battery)

## Setup & Installation

### 1. Clone the Repository

```bash
git clone https://github.com/Nitir4/LFR.git
cd LFR
```

### 2. ESP32 Setup

- Install [ESP32 board support](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html) in Arduino IDE.

### 3. PID Tuning Mode

1. Go to `/pid-tuner/`
2. Open `pid_tuner.ino` in Arduino IDE
3. Change WiFi credentials if needed
4. Upload to ESP32
5. Access the web server from your browser (IP will be shown on serial monitor)
6. Adjust PID values live and observe robot behavior

### 4. Run Mode

1. Go to `/pid-runner/`
2. Open `pid_runner.ino`
3. Enter the final PID values obtained from tuning
4. Upload to ESP32
5. Let your robot follow the line using optimal PID settings

## Circuit Diagram
I am too lazy to draw a circuit diagram or a schematic (i can probably take a photo but the connections are messy enough to make my head spin).
So let's just make do with pin definitions provided in the code.

## Contributing

Contributions are welcome! See [CONTRIBUTING.md](docs/CONTRIBUTING.md) for details.

## Troubleshooting

- **Web server not accessible:** Check WiFi credentials and serial output.
- **PID not responding:** Make sure sensors/motors are connected properly.
- **Robot not following line:** Re-tune PID values or check IR sensor placement.

## License

This project is licensed under the [MIT License](LICENSE).

---

**Contact:** [Your Name/Email]  
**Project Website:** [If any]
