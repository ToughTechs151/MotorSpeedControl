# Motor Speed Control

Use an Xbox Controller to set the speed of a single Neo motor with a PID controller and feedforward. Speed, PID and feedforward constants are adjustable from a dashboard.

- Run forward at fixed speed when the right trigger is held and backward when the left trigger is held.
- Run forward/reverse proportional to left joystick forward/backward

Fixed speed is adjustable from a dashboard for each trigger press, but other values need to be changed while disabled.

In simulation mode a DCMotorSim is run to model the dynamics of the motor and controller, and this mode can be used for training PIDF control of a flywheel subsystem.
