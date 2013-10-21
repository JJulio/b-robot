/*
  Example of WITA library.
  Code by Jose Julio and Jordi Muñoz . 3DRobotics.com

*/

#include <WITA.h> // ArduPilot Mega RC Library

void setup()
{
  WITA.LED_blink(5);
  WITA.InitServos();   // Servos initialization
  delay(1000);
}
void loop()
{
int i;

  for (i=0;i<500;i++)
	{
	WITA.Servo(0,1000+i*2);
	delay(50);
	}
}
  