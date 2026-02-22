cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(phase6_secure_hrs)

# Include path for pulse_sensor library
target_include_directories(app PRIVATE ../../lib/pulse_sensor)

# Source files
target_sources(app PRIVATE
    src/main.c
    ../../lib/pulse_sensor/pulse_sensor.c
)
