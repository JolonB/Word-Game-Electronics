# Word-Game-Electronics

Electronics for the Word Game project.

Dimensions are based on a standard Scrabble board, and therefore

## Tiles

The game tiles represent different letters that can be used to spell words, including a blank space tile which can be used to represent any letter.
The value of each tile is determined by the resistor inside it which is read by an ADC in the quarter it is placed in.
The tiles also have a 2-terminal male connector for connecting to the board.

The size of a single tile is 0.75"x0.75" (19.05x19.05mm), but the internal circuit is smaller.

## Segments

The internal board is made up of nine segments.
These segments each manage a 5x5 section of the 15x15 board.

Each segment consists of a microcontroller, and ADC, an analog multiplexer, a button, and LCD display, and 25 units.
Each unit is made up of a two-terminal female piece connector and an LED.

<!-- TBD: perhaps use a digital MUX to control MOSFET connections to each piece instead, depending on price -->

The entire board has dimensions of 15"x15" (38.1cm), but there is some small spacing between letters, although the width isn't documented anywhere.
I have seen 0.05" (1.27mm) being used in some projects, which would make the play space 11.95" (0.05\*14 + 0.75\*15). This leaves 1.455" as outer padding, although this dimension isn't too important for the electronics.

The segments are communicated with over I2C.
Using the general call address (0x00) allows all I2C devices to receive a broadcast.
The segments connect to the controller board over a 8 wire interface:
* 2x I2C (SDA + SCK)
* 1x device select / interrupt
* 2x power
* 3x ground

The device select pin works much like SPI's chip select pin, but only during provisioning.
This allows a single segment to interpret the message, despite all segment receiving it, which allows the controller to set the I2C addresses automatically.
Once the I2C address is set, the segment takes control of the pin and can use it to send interrupts to the controller.
This pin is pulled up and each side can only control the pin in open collector mode.

## Controller

The controller is the central device that handles all of the communication.
It has nine ports arranged in a 3x3 grid for connecting each of the segments too.
Connecting the segments correctly means it can automatically commission the boards to have the correct coordinates.

Besides these connections, this board will also display content on LCD screens and will have connections for buttons for each of the players to interact with.

The controller will communicate with the segments using some protocol.
I considered SPI, but the Raspberry Pi Pico SDK only supports blocking SPI reads and writes, which could lock up the program.
I also considered I2C, but that would require provisioning to set up the ID for each segment, which I would rather avoid.
The alternative would be to add a chip-select pin, like SPI, to select the I2C device and rather than using an address, just have a method for specifying whether the message can be responded to (personal) or not (broadcasted).
The same logic could be used for UART too, although UART isn't meant to have multiple slaves connected.
