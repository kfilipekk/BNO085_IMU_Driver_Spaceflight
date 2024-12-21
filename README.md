# BNO085 IMU Driver

[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5.1-blue.svg)](https://docs.espressif.com/projects/esp-idf/)

BNO085 sensor driver for ESP32 with 100Hz quaternion updates and complete SHTP protocol implementation. Made by Krystian Filipek for the Lander Project.

## Features

| Sensor Type | Rate | Precision | Use Case |
|-------------|------|-----------|----------|
| Rotation Vector | 100Hz | Q14 (±2°) | Attitude Control |
| Linear Acceleration | 100Hz | Q8 (m/s²) | Flight Dynamics |
| Gravity Vector | 50Hz | Q8 (m/s²) | Navigation |

### Hardware
```
BNO085    ESP32
------    -----
VIN   ->  3.3V
GND   ->  GND  
SDA   ->  GPIO21
SCL   ->  GPIO22
```

### Build
```bash
idf.py build
idf.py flash monitor
```

<div align="center">

Developed by [kfilipekk](https://github.com/kfilipekk)

</div>