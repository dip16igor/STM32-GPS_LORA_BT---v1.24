# STM32 LoRa GPS Tracker

This is the firmware for a "radio direction finder" controller based on a Time-Division LoRa MESH network. The project was originally developed using CooCox IDE for the STM32F103C8T6 microcontroller.

[Русская версия](README.ru.md)

## Features

*   Parses NMEA sentences from a GPS/GLONASS receiver.
*   Implements a time-division MESH network for communication between nodes using LoRa (RFM98).
*   Transmits and receives coordinate data, and retransmits packets from other nodes.
*   Calculates distances between beacons using trigonometric functions.
*   Communicates with a smartphone app via a BLE module (HM-10 clone).
*   Displays extensive information on an OLED SSD1306 screen, including:
    *   GPS status, coordinates, and satellite information.
    *   Battery voltage levels.
    *   LoRa packet statistics (RSSI, SNR, PER).
    *   Distances to other beacons.
*   Monitors battery voltage using the ADC.
*   Uses a 1PPS signal from the GPS for network time synchronization.

## Hardware

*   **MCU:** STM32F103C8T6 ("Blue Pill")
*   **GPS:** SIM-33ELA GPS/GLONASS module
*   **LoRa:** RFM98 LoRa Transceiver module
*   **Bluetooth:** HM-10 (or clone) BLE module
*   **Display:** SSD1306 I2C OLED Display
*   **Debugger:** ST-Link v2 or CoLinkEx clone via SWD

## Peripherals Used

*   **USART1:** Debug terminal communication with a PC.
*   **USART2:** Communication with the BLE module.
*   **USART3:** Communication with the GPS module.
*   **SPI1:** Communication with the LoRa module.
*   **I2C1:** Communication with the OLED display.
*   **ADC1 (Channel 0):** Main battery voltage measurement.
*   **ADC1 (Channel 1):** Backup battery voltage measurement.
*   **TIM1, TIM4, EXTI:** Used for 1PPS signal processing and network synchronization.
*   **DMA:** Used for efficient I2C display updates.

## Original Development Environment

*   **IDE:** CooCox CoIDE v.1.7.8
*   **Toolchain:** GCC 5.3
*   **Libraries:** STM32 Standard Peripherals Library (StdPeriphLib) v.3.5.0

