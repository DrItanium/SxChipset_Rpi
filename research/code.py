# 
# i960SxChipset_Rpi
# Copyright (c) 2024, Joshua Scoggins
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import time
import board
import rp2pio
import microcontroller
import adafruit_pioasm
# this program will generate a stable clock rate the fourth of the input clock
# rate
clkgen_program = """
.program clkgen
    set pindirs, 0b111
    set x, 0b11111
loop_start:
    mov pins, x
    jmp x-- loop_start
    jmp loop_start
"""

hello = """
.program hello
loop:
    pull
    out pins, 1
    jmp loop
"""

assembled_hello = adafruit_pioasm.assemble(hello)
assembled_clkgen = adafruit_pioasm.assemble(clkgen_program)


sm = rp2pio.StateMachine(
        assembled_hello,
        frequency=2000,
        first_out_pin=board.LED,
)
cpuFrequency = microcontroller.cpu.frequency
desiredRawClockRate = 20000000
quadRawClockRate = desiredRawClockRate * 4
divisor = cpuFrequency /quadRawClockRate 
print("\n\n")
print("state machine divisor", divisor)
sm2 = rp2pio.StateMachine(
        assembled_clkgen,
        frequency=int(cpuFrequency / divisor),
        first_set_pin=board.GP2,
        first_out_pin=board.GP2,
        out_pin_count=3,
        set_pin_count=3,
)

print("\n\n")
print("cpu frequency:", microcontroller.cpu.frequency)
print("hello real frequency", sm.frequency)
print("clkgen real frequency", sm2.frequency)

while True:
    sm.write(bytes((1,)))
    time.sleep(0.5)
    sm.write(bytes((0,)))
    time.sleep(0.5)
