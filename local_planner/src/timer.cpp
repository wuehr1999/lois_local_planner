#include <planner/timer.hpp>

void Timer::reset()
{
  gettimeofday(&start, 0);
}

double Timer::getSeconds()
{
	gettimeofday(&stop, 0);
	long seconds = stop.tv_sec - start.tv_sec;
	long microseconds = stop.tv_usec - start.tv_usec;

	return (double)seconds + (double)microseconds * 1e-6;
}
