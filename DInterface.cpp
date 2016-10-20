#include <stdio.h>
#include <conio.h>
#if _MSC_VER > 1200
#include <stdint.h>
#endif
#include <math.h>
#include <Windows.h>


#include "DInterface.h"
#include "v2d.h"
#include "driver.h"

Driver driver;

void GfProcess(shared_use_st *shared)
{
	driver.drive(shared);

#ifdef SMS

	double coe_steer = 1.0;
	double vmax = 20.0;

	shared->steerCmd = (shared->angle - shared->toMiddle / shared->track_width) * (coe_steer / 0.541052);

	double c = 2.772;
	double d = -0.693;
	double min, v;

	min = 20.0;
	for (int i = 0; i < 10; i += 2)
	{
		v = vmax * (1 - exp(-c / vmax*shared->dist_cars[i] - d));

		if (v < min)
			min = v;
	}

	if (min < shared->speed)
	{
		shared->brakeCmd = min / shared->speed;
		shared->accelCmd = 0;
	}
	else
	{
		shared->accelCmd = min / shared->speed;
		shared->brakeCmd = 0;
	}

	shared->backwardCmd = GEAR_FORWARD;
#endif
}