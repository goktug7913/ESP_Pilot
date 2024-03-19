#include "Init.h" //Initialization mostly done here
#include "components/ComponentBase.hpp"

/*
  The components are stored in a vector of pointers to ComponentBase.
  This allows for easy initialization and updating of all components.
  Also, the components can be added and removed at runtime.
  However, the memory layout is a bit more complex and we should
  consider the overhead of using pointers.
*/
std::vector<ComponentBase*> components;

void init()
{
  // Initialize all components
  for (auto component : components)
  {
    component->init();
  }
}

void update()
{
  // Update all components
  for (auto component : components)
  {
    component->update();
  }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void setup()
{
  init();
  coldstart();
  initEscDrive(); // move into coldstart()
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void loop()
{
  /*
    TODO: This function needs a total rework according to the new architecture.
  */

  // Program should recover from here in a second or two during flight or the quadcopter will probably crash
  // You should get back into the flight loop as fast as possible if fallback happens

  // FliCon.writeEsc(1000,1000,1000,1000); // Set all ESCs to 1000us (min throttle)

  if (FliCon.rx_raw[4] == 2000)
  {
    FliCon.armed = true;
  } // Arm on CH5 high

  if (FliCon.armed)
  {
    FliCon.Start();
  } // Directly enter flight loop with debugging
}