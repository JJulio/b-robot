#ifndef WITA_h
#define WITA_h

#include "Arduino.h"

#define MIN_PULSEWIDTH 600
#define MAX_PULSEWIDTH 2400
#define LED_pin 13

class WITA_Class
{
  private:
  public:
	WITA_Class();
	void InitServos();
	void Servo(unsigned char ch, int pwm);
	void ServoWrite(unsigned char ch, unsigned int angle);   // Arduino compatible angle value
	void LedOn();
	void LedOff();
	void LedBlink(int times, boolean FinalStatus = false);
    void WifiInit();
    void WifiSendCommand(char command[],boolean EndOfLine = true);
    void WifiEnterCommandMode();
    void WifiExitCommandMode();
    void WifiFactoryReset();
    void WifiChangeBaudRateFast();
    void WifiChangeBaudRateSlow();
    void WifiJoin(char ssid[], char password[], boolean WPA, boolean DHCP = true);
    void WifiSetIP(char ip[], char netmask[], char gateway[]);
    void WifiEnableUDP(char local_port[], char remote_port[], char remote[]);
    void WifiViewConfig();
    void WifiAP();
};

extern WITA_Class WITA;

#endif