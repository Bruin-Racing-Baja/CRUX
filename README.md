# CRUX
CVT Ratio Utility eXecutor
Bruin Racing Baja M25 Repository

## Install Dependencies
Install the ESP-IDF Development Environment

Tested with: 
- ESP-IDF v5.1
- Python 3.8+ 
 
Install Instructions: 
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html

## Clone the Repository 
```bash
git clone https://github.com/Bruin-Racing-Baja/m25_esp.git
cd m25_esp
```

## Setup Environment
### macOS
```bash
. $HOME/esp/esp-idf/export.sh
```

## Building Project
```
idf.py fullclean
idf.py build
```

## Flash to Device 
```
idf.py flash
```

## Monitor Serial Output 
```
idf.py monitor
```

## Directory Contents

```
├── CMakeLists.txt
├── README.md
├── main
│   ├── CMakeLists.txt
│   ├── app_main.cpp
│   ├── include
│   │   ├── README
│   │   ├── centerlock_controller.h
│   │   ├── ecvt_controller.h
│   │   ├── filters
│   │   │   ├── iir_filter.h
│   │   │   └── median_filter.h
│   │   ├── input_output
│   │   │   ├── button.h
│   │   │   ├── centerlock_limit_switch.h
│   │   │   ├── ecvt_limit_switch.h
│   │   │   ├── led.h
│   │   │   └── shift_register.h
│   │   ├── odrive.h
│   │   ├── sensors
│   │   │   ├── brake_pot_sensor.h
│   │   │   ├── gear_tooth_sensor.h
│   │   │   ├── sensor.h
│   │   │   └── throt_pot_sensor.h
│   │   └── telemetry.h
│   └── src
│       ├── centerlock_controller.cpp
│       ├── ecvt_controller.cpp
│       └── odrive.cpp
├── sdkconfig
├── sdkconfig.ci
└── test
    └── pytest_hello_world.py
```

- `sdkconfig.ci` is a reproducible configuration snapshot and is safe to commit.  
- `sdkconfig` is machine-specific and should not be committed.


