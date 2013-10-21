/*
	WITA.cpp -  Library for Arduino WITA board.
	Code by Jose Julio and Jordi Muñoz. 3DRobotics.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	RC Output : 4 Servo outputs (standard 20ms frame) D9,D10,D11,D5 (S0,S1,S2,S3)

	Methods:
		LED_on() : WITA LED on
		LED_off() : WITA LED off
		LED_blink(times) : WITA blink "times" times
		InitServos() : Initialization of interrupts an Timers for Servos (TIMER1 and 3)
		OutpuCh(ch,pwm) : Output value to servos (range : 900-2100us) ch=0..3
		
		
*/
#include "Arduino.h"
#include "WITA.h"

#include <avr/interrupt.h>
//#include "WProgram.h"


// Constructors ////////////////////////////////////////////////////////////////

WITA_Class::WITA_Class()
{
}

// Private function
void _SerialEcho()
{
unsigned char incomingByte;

  delay(300);
  //Serial.println("SerialEcho");
  while (Serial1.available() > 0) {
    incomingByte = Serial1.read();
    Serial.print(incomingByte);
  }
}


// Public Methods //////////////////////////////////////////////////////////////
void WITA_Class::InitServos(void)
{
  // Init PWM Timer 1  (D9,D10,D11)
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  //Remember the registers not declared here remains zero by default... 
  TCCR1A =((1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)); //Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all... 
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11); //Prescaler set to 8, that give us a resolution of 0.5us, read page 134 of data sheet
  OCR1A = 3000; //PB5, D9,   SERVO 0
  OCR1B = 3000; //PB6, D10,  SERVO 1
  OCR1C = 3000; //PB7, D11,  SERVO 2
  ICR1 = 40000; //50hz freq... Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000,

  // Init PWM Timer 3  (PC6, D5)
  //pinMode(2,OUTPUT);
  //pinMode(3,OUTPUT);
  //pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  //TCCR3A =((1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1));
  TCCR3A =((1<<WGM31)|(1<<COM3A1));
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31); 
  OCR3A = 3000; //PC6, D5, SERVO 3
  ICR3 = 40000; //50hz freq
  
}

void WITA_Class::ServoWrite(unsigned char ch, unsigned int angle)
{
	int pwm;
	
	pwm = 1000 + (angle*100)/18;  // Angle from 0 to 180 => pwm from 1000 to 2000
	Servo(ch,pwm);
}

void WITA_Class::Servo(unsigned char ch, int pwm)
{
  pwm=constrain(pwm,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
  pwm<<=1;   // pwm*2;
 
 switch(ch)
  {
    case 0:  OCR1A=pwm; break;  //ch0
    case 1:  OCR1B=pwm; break;  //ch1
    case 2:  OCR1C=pwm; break;  //ch2
    case 3:  OCR3A=pwm; break;  //ch3    
  } 
}

void WITA_Class::LedOn()
{
	digitalWrite(LED_pin,HIGH);
}

void WITA_Class::LedOff()
{
	digitalWrite(LED_pin,LOW);
}

void WITA_Class::LedBlink(int times, boolean FinalStatus)
{
	for (int i=0;i<times;i++)
    {
      LedOn();
      delay(180);
      LedOff();
      delay(180);
    }
    if (FinalStatus) LedOn();
}

// WIFI methods
void WITA_Class::WifiInit()
{
    Serial1.begin(115200);  // Default Wifi module Serial speed
    if (!Serial)
        Serial.begin(9600);     // Default baud rate if not initialized
    delay(1000);
}

void WITA_Class::WifiSendCommand(char command[],boolean EndOfLine)
{
  if (EndOfLine){
    Serial1.println(command);
    _SerialEcho();
  }
  else{
    Serial1.print(command);
  }
  //return true;
}

void WITA_Class::WifiEnterCommandMode()
{
  // First we try an exit (to exit an old command mode)
  Serial1.print("exit\r");
  _SerialEcho();
  // Entramos en modo Comando
  Serial1.print("$$$");
  _SerialEcho();
}

void WITA_Class::WifiExitCommandMode()
{
  // First we try an exit (to exit an old command mode)
  WifiSendCommand("exit");
}

void WITA_Class::WifiFactoryReset()
{
  WifiEnterCommandMode();
  WifiSendCommand("factory RESET");
  WifiSendCommand("reboot");
  delay(1000);
}

// Change baudrate to 115200
void WITA_Class::WifiChangeBaudRateFast()
{
  Serial1.end();
  Serial1.begin(9600);    // Deafult baud rate of Wifi module
  WifiEnterCommandMode();
  WifiSendCommand("set uart baudrate 115200");
  WifiSendCommand("save");
  WifiSendCommand("reboot");
  delay(500);
  Serial1.end();
  Serial1.begin(115200);
  delay(500);
}

// Change baudrate to 9600 (default baud rate)
void WITA_Class::WifiChangeBaudRateSlow()
{
  Serial1.end();
  Serial1.begin(115200);    // Fast baud rate 
  WifiEnterCommandMode();
  WifiSendCommand("set uart baudrate 9600");
  WifiSendCommand("save");
  WifiSendCommand("reboot");
  delay(500);
  Serial1.end();
  Serial1.begin(9600);
}

void WITA_Class::WifiJoin(char ssid[], char password[], boolean WPA, boolean DHCP)
{
  WifiEnterCommandMode();
  if (DHCP)
    WifiSendCommand("set ip dhcp 1");  // Enable DHCP
  else
    WifiSendCommand("set ip dhcp 0");  // Disable DHCP (use static IP)
  if (WPA){
    WifiSendCommand("set wlan phrase ",false);
    WifiSendCommand(password);
  }
  else{  // WEP 128bits
    WifiSendCommand("set wlan key ",false);
    WifiSendCommand(password);
  }
  WifiSendCommand("join ",false);
  WifiSendCommand(ssid);
  WifiExitCommandMode();
}

void WITA_Class::WifiSetIP(char ip[], char netmask[], char gateway[])
{
  WifiEnterCommandMode();
  WifiSendCommand("set ip address ",false);
  WifiSendCommand(ip);
  WifiSendCommand("set ip netmask ",false);
  WifiSendCommand(netmask);
  WifiSendCommand("set ip gateway ",false);
  WifiSendCommand(gateway);
  WifiExitCommandMode();
}

void WITA_Class::WifiEnableUDP(char local_port[], char remote_port[], char remote[])
{
  WifiEnterCommandMode();
  WifiSendCommand("set ip proto 1");
  WifiSendCommand("set ip localport ",false);
  WifiSendCommand(local_port);
  WifiSendCommand("set ip host ",false);
  WifiSendCommand(remote);
  WifiSendCommand("set ip remote ",false);
  WifiSendCommand(remote_port);
  WifiSendCommand("save");
  WifiSendCommand("reboot");   // We need to reboot to take effect
  delay(500);
  //Wifi_ExitCommandMode();
}

void WITA_Class::WifiViewConfig()
{
  WifiEnterCommandMode();
  WifiSendCommand("get ip");
  //Wifi_SendCommand("get dns");
  //Wifi_SendCommand("get wlan");
  WifiExitCommandMode();
}

void WITA_Class::WifiAP()
{
  WifiEnterCommandMode();
  WifiSendCommand("set wlan join 7");
  WifiSendCommand("set wlan ssid WITA_AP");
  WifiSendCommand("set wlan chan 1");
  WifiSendCommand("set ip address 192.168.1.1");
  WifiSendCommand("set ip netmask 255.255.255.0");
  WifiSendCommand("set ip dhcp 4");   // DHCP server
  WifiSendCommand("save");
  WifiSendCommand("reboot");   // We need to reboot to take effect
}

// make one instance for the user to use
WITA_Class WITA;