#include "Init.h" //Initialization mostly done here

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void setup(){
  coldstart();
  initEscDrive(); // move into coldstart()
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
void loop() {
  // Program should recover from here in a second or two during flight or the quadcopter will probably crash
  // You should get back into the flight loop as fast as possible if fallback happens

  //FliCon.writeEsc(1000,1000,1000,1000);
  SerialMan.ReceiveMsg();

  if(FliCon.rx_raw[4] == 2000){FliCon.armed = true;}  // Arm on CH5 high

  if (FliCon.armed){FliCon.Start();} // Directly enter flight loop with debugging
}