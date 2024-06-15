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

