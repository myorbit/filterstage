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

**Note** `<x>` is now limited to a maximum value of 5100! (This is due to the mechanical end stop.)

Example: `{"fpos":{"type":"short","pos":230}}`

###### Other possible commands are:

`{"fdrive":{"type":"short","action":"hardstop"}}`   immediate full stop
`{"fdrive":{"type":"short","action":"softstop"}}`   stops motor deceleration phase
`{"fdrive":{"type":"short","action":"resetpos"}}`   set actual position to zero
`{"fdrive":{"type":"short","action":"resetdefault"}}`   overwrite RAM with OTP contents
`{"fdrive":{"type":"short","action":"gotosecure"}}`   drives to secure position

###### Quick query commands for driving:

`{"fdrive":{"type":"short","action":"qmotion"}}`    query current motion of filter
response is `{"fstat":{"type":<pass>,"motion":<0:7>}}` with <0:7> being the actual position where 0 means Actual Position = Target Position and Velocity = 0
`{"fdrive":{"type":"short","action":"qactualpos"}}`     query actual position of filter
response is `{"fstat":{"type":<pass>,"actualpos":<actualpos>}}` with <actualpos> returning the Actual Position of the filter


###### Setting parameters:

`{"fmotor":{"type":"short","ihold":2, ... }}`   set motor parameter

`{"fstall":{"type":"short","dc100":0, ... }}`   set stall detection parameter

**Note** Changing motor parameter, reference position, etc. is not recommended!

###### The controller can send back the motor status or stall detection status:

`{"fstat":{"type":"short","steploss":0, ... }}`   send motor status
`{"fstall":{"type":"short","absstall":0, ... }}`   send stall detection status

