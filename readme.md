tinyFab Heatbed Controller

Description
tinyFab Heatbed provide thermal control for the cetus3D / up mini or other 3d printer

The controller controls a companion driver board which provides power to the heatbed  


Installation instruction
Connect the 3 wire power source cable from the controller board to the driver board
Connect the 4 wire power source cable from the controller board to the driver board
Connect the 2 pin sensor port to a thermister


For Cetus3D or up mini, connect the short FFC to the driver board CH1 input, this will supply power (19V) from the mainboard through the driver board to the controller.

For other printer, DO NOT use the FFC cable. Use external input wires and link J5 and J10 on the driver board to divert power to the controller

There are two option for connecting cetus3D and up mini 
For other printers only external power (Option 2) is available. 

Option 1 (tinyfab heatbed temperature max 60°C): you should have connected the short flat ribbon cable (FFC) from the mainboard to the driver CH1 input, this provides power (19V) to the controller also the heatbed. Then connect the long FFC from the driver CH1 output to the heatbed FFC connector 

Option 2 (controller temperature max 110°C): Power the driver board with a external power supply. 


Using the controller
When the controller is powered on the driver is set to off state.

OFF -> short press -> TEMPERATURE SET -> short press -> RUN -> long press -> OFF
OFF -> turn dial -> SETTINGS -> 
RUN -> turn dial -> TIMER SET -> short press -> RUN

a long press will turn off heatbed in RUN mode
a long press will return to upper level in SETTINGS


Settings    default     Description
PID         1           Set heating mode 0 is bit band 1 is PID
-P-         5.00        p coefficient of PID
-I-         0.01        i coefficient of PID
-d-         1.0         d coefficient of PID
HYS         2           Hysteresis of bit band (degree)
-t-         10          Maximum time allowed for the heatbed to reach set temperature (minutes)
-b-         395         NTC b constants (395x10)

Error code
E00  unknown error
E01  Sensor error
E02  Temperature raise error (max allow time reached to set temperature)


Troubleshoot
E01  check temperature sensor connection
E02  make sure you give enough time for the heatbed to heat up to your set temperature and it has not reached the capability of the heatbed.
driver switches off at around 60°C (+-10°C) when powered by the flat ribbon cable -> max temperature is reached. Wait a few minutes until heatbed is cooled down
   

Change Log

V1.00
First Release

V1.01
Add setting for NTC B coefficient
Add setting for thermal runaway time
removed idle screen off



