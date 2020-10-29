#ifndef OBJECT_DISCOVERY_TIMER_HPP
#define OBJECT_DISCOVERY_TIMER_HPP


#include <iostream>
#include <time.h>

class TickMeter
{
public:
  TickMeter() {reset();}
  void start() {startTime = clock();}
  void stop() 
  {
    int time = clock();
    if (startTime == 0)
      return;
    ++counter;
    sumTime += (time-startTime);
    startTime = 0;
  };

  void reset()
  {
    startTime = 0;
    sumTime = 0;
    counter = 0;
  }

  double getTimeSec() const
  {
    return (double)getTimeTicks()/CLOCKS_PER_SEC;
  }

  int counter;

private:
  int sumTime;
  int startTime;
  int getTimeTicks() const {return sumTime;}
};

#endif