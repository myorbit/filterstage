filterstage
===========

Optical bandpass controller that runs on an Arduino board controlling two TMC223 stepper drivers and can communicate using JSON.

JSON client is: https://github.com/interactive-matter/aJson

Tested on Arduino UNO and Arduino MEGA.

Usage
-----

To drive a filter to a certain position type

    {"fpos":{"type":<pass>,"pos":<x>}}

with `<pass>` = "short" or "long"
and `<x>` = [-32768:32767]

Example: `{"fpos":{"type":"short","pos":230}}`

Other possible commands are:

`{"fdrive":{"type":"short","action":"hardstop"}}`   immediate full stop
`{"fdrive":{"type":"short","action":"softstop"}}`   stops motor deceleration phase
`{"fdrive":{"type":"short","action":"resetpos"}}`   set actual position to zero
`{"fdrive":{"type":"short","action":"resetdefault"}}`   overwrite RAM with OTP contents
`{"fdrive":{"type":"short","action":"gotosecure"}}`   drives to secure position

`{"fmotor":{"type":"short","ihold":2, ... }}`   set motor parameter

`{"fstall":{"type":"short","dc100":0, ... }}`   set stall detection parameter


The controller can send back the motor status or stall detection status:

`{"fstat":{"type":"short","steploss":0, ... }}`   send motor status
`{"fstall":{"type":"short","absstall":0, ... }}`   send stall detection status

