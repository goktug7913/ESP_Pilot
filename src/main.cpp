#include "Init.h"                     //Initialization

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void setup(){
  coldstart();
  initEscDrive();
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
int counter;
bool ledset = 0;

void loop() {
  // Program should not stay in this loop for more than a second or two during flight
  // You should get back into the flight loop as fast as possible if fallback happens

  //FliCon.writeEsc(1000,1000,1000,1000);
  SerialMan.ReceiveMsg();

  if(FliCon.rx_raw[4] == 2000){FliCon.armed = 1;}  // Arm on CH5 high

  //if(FliCon.usbmode){flightLoop_d();}

  if (FliCon.armed){FliCon.Start();} // Directly enter flight loop with debugging
}