tinyFab Heatbed Controller

Description
tinyFab Heatbed provide thermal control for the cetus3D / up mini or other 3d printer

The controller controls a companion driver board which provides power to the heatbed  


Installation instruction
Connect the 3 wire power source cable from the controller board to the driver board
Connect the 4 wire power source cable from the controller board to the driver board
Connect the 2 pin sensor port to a thermister


for Cetus3D or up mini there are two ways to provide power to the driver Board:

Option 1 (heatbed temperature max 60°C): Connect the flat ribbon cable (FFC) to the mainboard of the 3d printer, this provides power to the controller. 

Option 2 (Heatbed temperature max 110°C: Power the driver board with a external power supply. In this case you have to bridge Jumper J5 and J10 on the driver board to provide power to the controller.

For other printers only external power (Option 2) is available.

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



