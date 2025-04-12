## Description

This project uses an STM32 microcontroller to control LEDs and buttons. The main code is located in `main.c`, where peripherals are configured, interrupts are handled, and inputs and outputs are managed.


## Installation

To install and set up the project, follow these steps:

1. Clone the repository:
   ```bash
   git clone https://github.com/GuillermoBaUr/PersonalProjects/tree/main/C/CRJ-200%20Autopilot%20Panel%20STM32
   ```
2. Open the project in STM32CubeIDE.

3. Ensure you have STM32CubeMX installed to configure the peripherals.

4. Compile and upload the code to your STM32F4RE11 microcontroller.

##Usage

Main Features
LED Control: The project allows you to turn on and off various LEDs connected to the microcontroller.
Button Reading: It reads the states of several buttons and handles interrupts to detect changes.
Debounce: Implementation of debounce to avoid erroneous button readings.
UART: Serial communication to send and receive data.
Example Usage

##License
This project is licensed under the MIT License. See the LICENSE file for more details.

### Summary of Code in `main.c`

The `main.c` file includes the following sections and functionalities:

- **System Configuration**: Initialization of the system clock and peripheral configuration.
- **Peripheral Initialization**: Configuration of GPIO and UART.
- **Interrupt Functions**: Handling interrupts for buttons and encoders.
- **Control Functions**: Updating LEDs and reading buttons.
- **UART Functions**: Sending and receiving data via UART.
