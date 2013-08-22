filterstage
===========

It is an optical bandpass controller that runs on an Arduino board controlling two TMC223 stepper drivers and can communicate using JSON.

JSON client is: https://github.com/interactive-matter/aJson

Tested on Arduino UNO and Arduino MEGA.

Usage
=====

To drive a filter to a certain position type

    {"fpos":{"type":<pass>,"pos":<x>}}

with <pass> = "short" or "long"
and <x> = [-32768:32767]

Example:
    {"fpos":{"type":"short","pos":230}}

To Do
=====

Currently only positioning is possible. Changing motor parameters and readout of the motor status will be implemented in future.

