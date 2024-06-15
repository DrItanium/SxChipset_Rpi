This is a next generation i960 chipset which uses raspberry pi based tools to
solve problems.

- An may be RP2040 is used to generate the clock signals and other low level peripherals
- A Raspberry Pi 4 B is used to act as the rest of the system (GPU, RAM, etc)
- There can also be a 2560 and 4808 in the design as well to be on the safe side


# Initial Setup

After cloning the repo do the following from the root of the repo

1. ` python -m venv . `
2. ` . bin/activate `
3. ` pip install --upgrade platformio `

After this point the initial setup is done. After this all you need to do on a
new terminal is run

` . bin/activate `


# Project Structure

There are two important folders, the research directory is a scratch space to
try things out. The projects directory contains the platformio projects that I
can use to program different chips I have access to. The here is a legend for
the different projects:

- ClockChip - An atmega4808 chip that generates the 20MHz, 10Mhz, and 5MHz clocks used  It also uses the remaining pins to provide a sensor framework. TWI is hooked up
- Chipset2560 - An atmega2560 which has a dedicated UART, SDCard socket, and enabled TWI. All other signals are exposed for use
- ChipsetRP2040 - A raspberry pi pico that is installed into the design through a separate custom PCB. Has logic level shifters to allow for easy interfacing of different pieces of code.

