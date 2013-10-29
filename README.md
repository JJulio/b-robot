B-Robot
=======
Self Balancing arduino robot. Control via Smartphone. Fully 3D printed project.

Video: http://youtu.be/V-_uxpX9aFQ

Blog: http://cienciaycacharreo.blogspot.com.es/2013/10/b-robot-un-robot-equilibrista-impreso_28.html

Board: custom board (WITA): It´s an Arduino Leonardo + Wifi module (RN131)

Motors: NEMA17 Stepper Motors 

Drivers: A4988 with 1/8 microstepping configuration

IMU: MPU6050 connected via I2C bus. We are using the DMP internal quaternion solution at 200Hz

Sonar sensor: LV-MaxSonar-EZ3.

Control algorithms: PI speed control driving a PD control for stability (control robot angle).
   The output of the stability control is a motor speed that is integrated (acceleration).

Wifi module : RN131 module in Soft AP mode, so you don´t need an existing Wifi network, the module
   generates it´s own network that you need to join with your smatphone

Communication protocol: OSC protocol (this is an open music protocol). It´s an UDP messages based
   protocol. You could find many applications that implement this protocol. I recommend TouchOSC 
   because you could create your own interfaces.

Modes: Manual mode with external Wifi control via OSC protocol (Smartphone/Tablet...)
       Autonomous mode. It uses the sonar sensor for obstacle avoiding. When the robot detects
       an obstacle it starts turning until it finds an empty space and continue.
       
      
