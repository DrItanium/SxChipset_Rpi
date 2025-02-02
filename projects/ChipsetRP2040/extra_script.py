import os
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

# Path to the PIO assembler
pioasm_path = os.path.join(env.PioPlatform().get_package_dir("tool-pico-sdk"), "tools", "pioasm")

# List of PIO files to compile
pio_files = [
    "src/address_assignment.pio",
    "src/read_parallel.pio",
    "src/write_parallel.pio"
]

# Compile each PIO file
for pio_file in pio_files:
    pio_header = pio_file.replace(".pio", ".pio.h")
    env.Command(
        pio_header,
        pio_file,
        f"{pioasm_path} -o header $SOURCE $TARGET"
    )

# Add the generated headers to the build environment
env.Append(CPPPATH=["src"])