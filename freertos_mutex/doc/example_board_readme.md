Hardware requirements
=====================
- Micro USB cable
- Personal Computer
- FRDM-MCXW71 board

Board settings
==============
No special settings are required.

Prepare the Demo
================
1. Connect the micro and mini USB cable between the PC host and the USB ports on the board.
2. Open a serial terminal on PC for the serial device with these settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3. Download the program to the target board.
4. Either press the reset button on your board or launch the debugger in your IDE to begin running
   the demo.

Running the demo
================
The following lines are printed to the serial terminal when the demo program is executed.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
"ABCD | EFGH"
"1234 | 5678"
"ABCD | EFGH"
"1234 | 5678"
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

