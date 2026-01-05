# hehe sci oly ev for lambert

Basically this ev will use cascading PID with an outer position loop and inner
velocity loop to precisely control the motor to move the right distance in the right time.

The path that the PID will try to match up with is a trapezoidal velocity-time profile,
with a constant acceleration period, a cruise period, and a constant deceleration period.

The mathematical setup for the calculations can be found in
[this overleaf doc](https://www.overleaf.com/read/zccntryzrtrq#407659) and the actual
result of the derivation can be viewed and verified
[here on desmos](https://www.desmos.com/calculator/avxj6yh4fy).

*i am not showing the derivation itself btw that is too much work*

The target dist and time can be set on the vehicle itself with an ec11 w/ button similar
to how unphayzed can. both values are set digit by digit, and a press of the encoder button confirms
the selection for each digit. The values are displayed by an onboard SSD1306 display.

the cad model is [here on Onshape](https://cad.onshape.com/documents/09e719b422f29904583c4133/w/bb05655bc81b422d4fd31c2f/e/7e8e20df467981abf628ac93?renderMode=0&uiState=695bf9a4c22a165212b69b68)

and here is the wiring diagram
![if this text shows up ur cooked](schematic.jpg)

### acknowledgements

inspiration for the design of the vehicle was taken from https://unphayzed.com/kit-instructions/ev26/