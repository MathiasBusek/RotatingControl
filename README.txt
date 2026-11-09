Nanotec USB / Modbus RTU connection GUI

Start by double-clicking "Start Nanotec GUI.bat". Keep these files in the
outputs folder alongside the workspace's .venv environment.

1. Connect and power the controller, and connect its USB interface.
2. On the Connection tab, select Find interfaces. For this C5-E-2-11 select Nanotec VCP (COM4)
   using MODBUS VCP. The COM number can change when reconnected. For a
   Modbus RTU controller, select the appropriate COM interface.
3. For Modbus RTU choose the controller's baud rate and parity, then select
   Open interface. Those serial settings do not apply to MSC.
4. Select Scan devices, choose the controller, and select Connect.
5. Live feedback shows encoder increments, motor shaft angle since the most
   recent Start cycle, and averaged velocity in rpm. The angle is calculated
   modulo 360 degrees using 16,384 counts per motor revolution. It is zeroed
   using an encoder reading immediately before the start command is sent and
   is cleared on disconnect or read failure. It is a motor shaft angle; a
   gearbox or belt ratio must be included to obtain the platform angle.
   The velocity is the mean of up to 10 recent readings divided by 10, as
   requested. Values update approximately twice per second.
6. Select the Connection tab and choose Disconnect when finished.

Position cycle controls (requires the modified NanoJ program):
- Drag the Speed 1 and Speed 2 sliders from 0.1 to 10.0 rpm, in 0.1 rpm steps.
  The GUI multiplies each displayed value by 10 before writing the integer
  setting to the controller. Each is used at the next corresponding segment.
- Symmetric hides Speed 2 and uses Speed 1 for both settings. Changes are
  sent with Apply speeds or Start cycle.
- The cycle button alternates between sending start and stop requests. NanoJ
  does not acknowledge the cycle state. On connection the state is unknown,
  so the button first offers Stop cycle; check the platform before starting.
- NanoJ must be running. Cycle stop is not an emergency stop.

The Modbus RTU defaults, 19200 baud and even parity, come from Nanotec's
example and must match the controller configuration. The C5-E-2-11 USB
connection was successfully tested using the Nanotec MODBUS VCP interface.
The NanoJ buttons start or stop the program; stopping the program does not
necessarily stop the motor. Starting reloads the program and resets its
variables. Program control has not been tested on the physical controller.
The library may require access outside a
restricted sandbox to load its native Windows DLLs.

Based on the NanoLib Python example by Nanotec GmbH & Co. KG, licensed under
Creative Commons Attribution 4.0: https://creativecommons.org/licenses/by/4.0/
