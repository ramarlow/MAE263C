### MAE 263C stuff

Dependencies:
- [`dynamixel_controller`](https://pypi.org/project/dynamixel-controller/) python module
- `numpy`
- [`plotjuggler`](https://github.com/facontidavide/PlotJuggler) for plotting (data is sent over UDP)

So far just testing with the dynamixels to see if we can actually get torque transparency. I think whatever method is used to guess dynamixel torque (not a load cell) causes it to not work well during movement. You can sorta see in the plots how applying a constant-ish torque results in a signal that keeps jumping down to zero. The filtering helps a little bit with preventing oscillation while standing still but free space movement feels not great.

![plots](/initial%20torque%20transparency%20test.jpg)
