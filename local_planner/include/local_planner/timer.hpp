#ifndef TIMER_HPP
#define TIMER_HPP

#include <stdio.h>
#include <sys/time.h>

/**
 @brief  Benchmark timer
 **/
class Timer
{
public:
  /**
   @brief Initializes Timer
   **/
  Timer() { reset(); }
  ~Timer(){}

  /**
   @brief Reset Timer
   **/
  void reset();
  /**
   @brief Masurement
   @return elapsed time in seconds
   **/
  double getSeconds();

private:
  struct timeval start, stop;
};
#endif
