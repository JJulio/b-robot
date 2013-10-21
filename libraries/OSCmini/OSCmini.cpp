/*
	OSCmini.cpp -  Library for OSC control (designed originally for Arduino WITA board).
	Code by Jose Julio and Jordi Muñoz. 3DRobotics.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    OSC Messages read:
             FADDER (1,2,3,4)
             XY (1,2)
             PUSH (1,2,3,4)
             TOGLE (1,2,3,4)
    OSC Message send:
            
    
	Methods:
            MsgSend()
            MsgRead()
		
*/
#include "Arduino.h"
#include "OSCmini.h"

#include <avr/interrupt.h>
//#include "WProgram.h"

//#define DEBUG 0


// Private Methods
float OSCmini_Class::extractParamFloat1(){
union{
  unsigned char Buff[4];
  float d;
}u;
 
  u.Buff[0] = (unsigned char)UDPBuffer[0];
  u.Buff[1] = (unsigned char)UDPBuffer[1];
  u.Buff[2] = (unsigned char)UDPBuffer[2];
  u.Buff[3] = (unsigned char)UDPBuffer[3];
  return(u.d); 
}
float OSCmini_Class::extractParamFloat2(){
union{
  unsigned char Buff[4];
  float d;
}u;
 
  u.Buff[0] = (unsigned char)UDPBuffer[4];
  u.Buff[1] = (unsigned char)UDPBuffer[5];
  u.Buff[2] = (unsigned char)UDPBuffer[6];
  u.Buff[3] = (unsigned char)UDPBuffer[7];
  return(u.d); 
}


// Constructors ////////////////////////////////////////////////////////////////

OSCmini_Class::OSCmini_Class()
{
  readStatus=0;
  readCounter=0;
  readNumParams=0;
  commandType=0;
  fadder1 = 0.5;
  fadder2 = 0.5;
  fadder3 = 0.5;
  fadder4 = 0.5;
}

// Private function
// This function lets us send simple one param messages (float param)
void OSCmini_Class::MsgSend(char *c,unsigned char msgSize, float p)
{

union{
  unsigned char Buff[4];
  float d;
}u;

  // We copy the param in the last 4 bytes
  u.d = p;
  c[msgSize-4] = u.Buff[0];
  c[msgSize-3] = u.Buff[1];
  c[msgSize-2] = u.Buff[2];
  c[msgSize-1] = u.Buff[3];
  // We send the message
  //for (i=0;i< msgSize;i++)
  Serial1.write((const uint8_t *)c,msgSize);
}


void OSCmini_Class::MsgRead()
{
  unsigned char i;
  float value;
  float value2;
  if (Serial1.available() > 0) {
    // We rotate the Buffer (we could implement a ring buffer in future)
    for (i=7;i>0;i--){
      UDPBuffer[i] = UDPBuffer[i-1];
    }
    UDPBuffer[0] = Serial1.read();
    #ifdef DEBUG
        Serial.print(UDPBuffer[0]);
    #endif
    // We look for an OSC message start like /x/
    if ((UDPBuffer[0] == '/')&&(UDPBuffer[2] == '/')){
      if (readStatus == 0){
	    page = UDPBuffer[1] - '0';  // Convert page to int
        readStatus = 1;
        //Serial.print("$");
      }
      else{
        readStatus = 1;
        Serial.println("!ERR:osc");
      }
      return;
    } else if (readStatus==1){   // looking for the message type
      // Fadder    /1/fadder1 ,f  xxxx   
      if ((UDPBuffer[3] == 'd')&&(UDPBuffer[2] == 'e')&&(UDPBuffer[1] == 'r')){
        readStatus=2;    // Message type detected
        readCounter=11;  // Bytes to read the parameter
        readNumParams=1; // 1 parameters
        switch (UDPBuffer[0]){  // fadder number
          case '1':
            commandType = 1;
			#ifdef DEBUG 
				Serial.print("$FAD1$");
			#endif
            break;
          case '2':
            commandType = 2;
			#ifdef DEBUG 
				Serial.print("$FAD2$");
			#endif
            break;
          case '3':
            commandType = 3;
			#ifdef DEBUG 
				Serial.print("$FAD3$");
			#endif
            break;
          case '4':
            commandType = 4;
			#ifdef DEBUG 
				Serial.print("$FAD4$");
			#endif
            break;
        }
        return;
      } // end fadder
      // XY message
      if ((UDPBuffer[2] == 'x')&&(UDPBuffer[1] == 'y')){
        readStatus=2;    // Message type detected
        readCounter=14;  // Bytes to read the parameters
        readNumParams=2; // 2 parameters
        switch (UDPBuffer[0]){  // xy number
          case '1':
            commandType = 11;
			#ifdef DEBUG 
				Serial.print("$XY1:");
			#endif
            break;
          case '2':
            commandType = 12;
			#ifdef DEBUG 
				Serial.print("$XY2:");
			#endif
            break;
          default:
            commandType = 11;
			#ifdef DEBUG 
				Serial.print("$XY:");
			#endif
            break;
        }
        return;
      }  // End XY message
      // Push message
       if ((UDPBuffer[4] == 'p')&&(UDPBuffer[3] == 'u')&&(UDPBuffer[2] == 's')&&(UDPBuffer[1] == 'h')){
        readStatus=2;    // Message type detected
        readCounter=10;  // Bytes to read the parameter
        readNumParams=1; // 1 parameters
        switch (UDPBuffer[0]){  // push number
          case '1':
            commandType = 21;
			#ifdef DEBUG 
				Serial.print("$P1:");
			#endif
            break;
          case '2':
            commandType = 22;
			#ifdef DEBUG 
				Serial.print("$P2:");
			#endif
            break;
          case '3':
            commandType = 23;
			#ifdef DEBUG 
				Serial.print("$P3:");
			#endif
            break;
          case '4':
            commandType = 24;
			#ifdef DEBUG 
				Serial.print("$P4:");
			#endif
            break;
        }
        return;
      } // end push
	  // Toggle message
       if ((UDPBuffer[3] == 'g')&&(UDPBuffer[2] == 'l')&&(UDPBuffer[1] == 'e')){
        readStatus=2;    // Message type detected
        readCounter=10;  // Bytes to read the parameter
        readNumParams=1; // 1 parameters
        switch (UDPBuffer[0]){  // push number
          case '1':
            commandType = 31;
            #ifdef DEBUG 
				Serial.print("$T1:");
			#endif
            break;
          case '2':
            commandType = 32;
			#ifdef DEBUG 
				Serial.print("$T2:");
			#endif
            break;
          case '3':
            commandType = 33;
			#ifdef DEBUG 
				Serial.print("$T3:");
			#endif
            break;
          case '4':
            commandType = 34;
			#ifdef DEBUG 
				Serial.print("$T4:");
			#endif
            break;
        }
        return;
      } // end toggle
    } else if (readStatus==2){
      readCounter--;   // Reading counter until we reach the Parameter position
      if (readCounter<=0){
        readStatus=3;
        //Serial.print("$LEERNUM$");
        value = extractParamFloat1();
        //Serial.print("F:");
        //Serial.println(value);
        if ((value<0)||(value>1))
          Serial.println("!ERR:float1!");
        if (readNumParams==2){
          value2 = extractParamFloat2();
          if ((value2<0)||(value2>1))
            Serial.println("!ERR:float2!");
        }
        //Serial.println(value);
        readStatus=0;
        switch (commandType){
          case 1:
            fadder1 = value;
			#ifdef DEBUG 
				Serial.println(fadder1);
			#endif
            break;
          case 2:
            fadder2 = value;
			#ifdef DEBUG 
				Serial.println(fadder2);
			#endif
            break;
          case 3:
            fadder3 = value;
			#ifdef DEBUG 
				Serial.println(fadder3);
			#endif
            break;
          case 4:
            fadder4 = value;
			#ifdef DEBUG 
				Serial.println(fadder4);
			#endif
            break;
          case 11:
            xy1_x = value;
            xy1_y = value2;
			#ifdef DEBUG 
				Serial.print(xy1_x);
				Serial.print(",");
				Serial.println(xy1_y);
			#endif
            break;
          case 12:
            xy2_x = value;
            xy2_y = value2;
			#ifdef DEBUG 
				Serial.print(xy2_x);
				Serial.print(",");
				Serial.println(xy2_y);
			#endif
            break;
          case 21:
            if (value==0)
              push1 = 0;
            else
              push1 = 1;
			#ifdef DEBUG 
				Serial.println((int)push1);
			#endif
            break;
          case 22:
            if (value==0)
              push2 = 0;
            else
              push2 = 1;
			#ifdef DEBUG 
				Serial.println((int)push2);
			#endif
            break;
          case 23:
            if (value==0)
              push3 = 0;
            else
              push3 = 1;
			#ifdef DEBUG 
				Serial.println((int)push3);
			#endif
            break;
          case 24:
            if (value==0)
              push4 = 0;
            else
              push4 = 1;
			#ifdef DEBUG 
				Serial.println((int)push4);
			#endif
            break;
		  case 31:
            if (value==0)
              toggle1 = 0;
            else
              toggle1 = 1;
			#ifdef DEBUG 
				Serial.println((int)toggle1);
			#endif
            break;
          case 32:
            if (value==0)
              toggle2 = 0;
            else
              toggle2 = 1;
			#ifdef DEBUG 
				Serial.println((int)toggle2);
			#endif
            break;
          case 33:
            if (value==0)
              toggle3 = 0;
            else
              toggle3 = 1;
			#ifdef DEBUG 
				Serial.println((int)toggle3);
			#endif
            break;
          case 34:
            if (value==0)
              toggle4 = 0;
            else
              toggle4 = 1;
			#ifdef DEBUG 
				Serial.println((int)toggle4);
			#endif
            break;
        }
      }
    }
  }  // end Serial.available()
}


// Public Methods //////////////////////////////////////////////////////////////

// make one instance for the user to use
OSCmini_Class OSC;