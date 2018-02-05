#include <chrono>
#include <cstdio>
#include <thread>

#include "ntcore.h"
#include <math.h>

int main() 
{
  double sinCount = 0.0;

  nt::StartClient("roborio-4089-frc.local", 1735);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  while (true)
  {
    double sinValue = sin(sinCount);
    sinCount += 0.01;
    nt::SetEntryValue("/sin/sin", nt::Value::MakeDouble(sinValue));
    printf("%f\n", sinValue);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}
