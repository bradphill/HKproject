# Testrig
This package allows driving of the testrig.

## Reqs
Apart from ROS, you need ```rosserial```, ```rosserial_arduino``` and the Python package ```pynput``` for the keyboard nodes.

## Connecting it all
Make sure the arduino is connected correctly. Check so that pins in ```arduino_code/controller/controller.ino``` are correct. Motors need power, obviously.

## Keyboard nodes
There are two keyboard nodes available.

The first is ```keyboard_wsad_node.py``` which allows WSAD steering. Easy. It publishes an action message that is then translated into pin output by the motor controller node and subsequentially fed to the Arduino. Quit with ```q```.

The second is ```keyboard_pins_node.py```. There you input how you want your motors to drive. For example ```10 10 00 00``` would mean that motors one and two are enabled and drive in CCW direction. Motors 3 and 4 are not enabled.

## Running the testrig
1. Make sure that the Arduino port in ```launch/testrig.launch``` is correct.
2. Start the testrig with ROS:
```console
roslaunch testrig testrig.launch
```
3. Open a new terminal and start a chosed keyboard node:
```console
rosrun testrig keyboard_xxxx_node.py
```
4. Drive!

## Info om filer
Testrig.launch  
startar testriggen med följande noder:  
motor_controller_node.cpp som motor_controller_node  
Arduino med controller.ino som arduino_motor_node 
  
  
Controller.ino  
    • Tar in meddelanden från motorkontroller, topic ”pins”, läser av dem och kör önskade motorer i önskad riktning  
    • Visualiserar även information om vilka motorer som körs och åt vilket håll på den lilla displayen (OLED).  
  
    
keyboard_wsad_node.py  
    • Gör att riggen går att kontrollera med wasd-tangenterna genom att publicera meddelanden till topic ”motor_action”  
      
      
motor_controller_node.cpp  
    • hämtar meddelanden från topic ”motor action”  
    • publicerar meddelanden till topic ”pins” samt ”motor_override” varav den senare inte används   
    • Skriver ut lite information i terminalen (om motorn är på och vilket håll den roterar åt)  

