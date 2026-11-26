# Smart Solar Energy Management & Power Monitoring System

## Project Overview

The **Smart Solar Energy Management & Power Monitoring System** is an ESP32-based IoT project designed to monitor solar energy, battery conditions, and electrical load consumption.

The system uses sensors to measure voltage, current, and power. Based on the estimated battery state of charge (SOC), it automatically manages connected electrical loads to help preserve battery energy.

A Wi-Fi-based dashboard provides real-time monitoring, graphical data visualization, and manual load control.

## Objectives

- Monitor solar panel and load electrical parameters.
- Estimate battery state of charge.
- Automatically manage electrical loads according to battery level.
- Provide real-time monitoring through a Wi-Fi dashboard.
- Store operational data on a micro SD card.
- Develop a low-cost solar energy monitoring and management solution.

## Key Features

- Real-time voltage, current, and power monitoring.
- INA226-based electrical measurement.
- Battery state-of-charge estimation.
- Three-level automatic load management.
- ESP32-based control system.
- Wi-Fi-based monitoring dashboard.
- Graphical data visualization.
- Micro SD card data logging.
- CSV data export.
- Automatic and manual operating modes.

## Hardware Components

- ESP32 Development Board
- Solar Panel
- 12V Battery
- Solar Charge Controller
- INA226 Current and Power Sensors (2 units)
- Voltage Detection Module
- Buck Converter
- Relay Modules
- LED Loads
- DC Fan
- Micro SD Card Module

## Operating Modes

The system uses three operating modes based on the estimated battery charge:

| Mode | Battery Level | Load Condition |
|---|---|---|
| NORMAL | 50% or above | Both LEDs and fan ON |
| CRITICAL | Below 50% | Fan disconnected; both LEDs remain ON |
| INTENSE | Below 30% | Only LED 1 remains ON |

The system also uses separate recovery thresholds to avoid frequent switching between operating modes.

## Monitoring Dashboard

The ESP32 creates its own Wi-Fi access point, allowing users to access a web-based dashboard.

The dashboard provides:

- Live electrical measurements.
- Battery monitoring.
- Load status.
- Graphical analysis.
- System event information.
- Manual load control.
- CSV data download.

## Data Logging

The system stores measured operational data on a micro SD card in CSV format for further analysis.

## Technologies Used

- ESP32
- Embedded C/C++
- Arduino IDE
- INA226 Sensor
- IoT
- Wi-Fi
- HTML, CSS and JavaScript
- Micro SD Card Data Logging
- Relay-Based Load Control

## Applications

- Small solar power systems.
- Off-grid lighting applications.
- Remote-area energy monitoring.
- Small DC power management systems.
- Renewable energy monitoring.

## Project Team

- Meer MD. Tafhim Uddin
- MD. Mirazul Biswas
- Annay Bhowmick

## Acknowledgement

We would like to express our sincere gratitude to our project supervisor, **Dr. T.H.M. Sumon Rashid**, Associate Professor, Department of Electrical and Electronic Engineering, PUST, for his guidance and support throughout this project.

---

**Department of Electrical and Electronic Engineering**  
**Pabna University of Science and Technology (PUST)**
