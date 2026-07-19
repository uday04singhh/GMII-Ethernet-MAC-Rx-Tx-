# GMII Ethernet MAC (Rx/Tx)

A SystemVerilog RTL implementation of a Gigabit Ethernet MAC datapath, supporting both 
transmit and receive paths over a GMII interface.

## Overview
- FSM-based control logic for frame handling
- Frame decoding on the receive path
- CRC validation for data integrity checking
- Verified at 125 MHz operation with ~500 ns end-to-end packet parsing

## Tech Stack
- **Language:** SystemVerilog
- **Verification:** SystemVerilog testbench (simulation-based)

## Files
- `Sources/` — GMII MAC top, Receiver, Transmitter
- `Testbench/` — SystemVerilog testbench

## Status
Functionally verified in simulation. FPGA synthesis/hardware validation not yet performed.

## Author
Uday Singh — BE Electronics and Computer Engineering, Thapar Institute of Engineering and Technology
