<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

The architecture uses Toffoli gates for the arithmetic and logic unit (ADD, AND, NAND via ancilla bits), Fredkin gates for the routing and shifting modes (SWAP, SHIFT, MUX), and a CNOT chain for XOR and parity. The 3-bit opcode decoder selects among 8 modes. The clock drives a 4-bit accumulator register so the chip acts statefully — load an operand, clock in an operation, read the result, then chain further operations.

## How to test

I/O mapping is tight: 3 bits of opcode, 4 bits of operand, and 1 load/execute control all fit in ui_in[7:0]. The 8 output bits carry the 4-bit result, 2 status flags (zero and carry), and 2 garbage output bits that are intentionally exposed to demonstrate reversibility 

## External hardware

No External HW, maybe a MCU or FPGA for monitaroing 
