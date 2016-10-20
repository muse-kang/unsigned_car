#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <math.h>

#include "v2d.h"
#include "DInterface.h"


#ifndef MAX
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif

#ifndef MIN
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif

const double PI = 3.14159265358979323846;  /**< PI */
const float G = 9.80665f; /**< m/s/s */

						  /* conversion */
#define RADS2RPM(x) ((x)*9.549296585)		/**< Radian/s to RPM conversion */
#define RPM2RADS(x) ((x)*.104719755)		/**< RPM to Radian/s conversion */
#define RAD2DEG(x)  ((x)*(180.0/PI))		/**< Radian to degree conversion */
#define DEG2RAD(x)  ((x)*(PI/180.0))		/**< Degree to radian conversion */
#define FEET2M(x)   ((x)*0.304801)		/**< Feet to meter conversion */
#define SIGN(x)     ((x) < 0 ? -1.0 : 1.0)	/**< Sign of the expression */

						  /** Angle normalization between 0 and 2 * PI */
#define NORM0_2PI(x) 				\
do {						\
	while ((x) > 2*PI) { (x) -= 2*PI; }	\
	while ((x) < 0) { (x) += 2*PI; } 	\
} while (0)

						  /** Angle normalization between -PI and PI */
#define NORM_PI_PI(x) 				\
do {						\
	while ((x) > PI) { (x) -= 2*PI; }	\
	while ((x) < -PI) { (x) += 2*PI; } 	\
} while (0)


#ifndef DIST
						  /** Distance between two points */
#define DIST(x1, y1, x2, y2) sqrt(((x1) - (x2)) * ((x1) - (x2)) + ((y1) - (y2)) * ((y1) - (y2)))
#endif

#define RCM_MAX_DT_ROBOTS	0.02
#define STEER_LOCK 0.43f


class Driver {
	public:
		Driver();
		~Driver();

		shared_use_st *m_s;
		int m_stuck;
		int m_MAX_UNSTUCK_COUNT;
		float m_OVERTAKE_OFFSET_INC;		///< [m/timestep]

		void drive(shared_use_st *s);

	private:
		// Utility functions.
		bool isStuck();
		float getAllowedSpeed();
		float getAccel();
		float getDistToSegEnd();
		float getBrake();
		int getGear();
		float getSteer();
		v2d getTargetPoint();
		float getOffset();
		float brakedist(float allowedspeed);

		float filterOverlap(float accel);
		float filterBColl(float brake);
		float filterABS(float brake);
		float filterBPit(float brake);
		float filterBrakeSpeed(float brake);
		float filterTurnSpeed(float brake);

		float filterTCL(float accel);
		float filterTrk(float accel);

		float filterSColl(float steer);

		void computeRadius(float *radius);
		int isAlone();

		// Class constants.
		static const float MAX_UNSTUCK_ANGLE;
		static const float UNSTUCK_TIME_LIMIT;
		static const float MAX_UNSTUCK_SPEED;
		static const float MIN_UNSTUCK_DIST;
		static const float G;
		static const float FULL_ACCEL_MARGIN;
		static const float SHIFT;
		static const float SHIFT_MARGIN;
		static const float ABS_SLIP;
		static const float ABS_RANGE ;
		static const float ABS_MINSPEED;
		static const float TCL_SLIP;
		static const float LOOKAHEAD_CONST;
		static const float LOOKAHEAD_FACTOR;
		static const float WIDTHDIV;
		static const float SIDECOLL_MARGIN;
		static const float BORDER_OVERTAKE_MARGIN;
		static const float OVERTAKE_OFFSET_SPEED;
		static const float PIT_LOOKAHEAD;
		static const float PIT_BRAKE_AHEAD;
		static const float PIT_MU;
		static const float MAX_SPEED;
		static const float TCL_RANGE;
		static const float MAX_FUEL_PER_METER;
		static const float CLUTCH_SPEED;
		static const float CENTERDIV;
		static const float DISTCUTOFF;
		static const float MAX_INC_FACTOR;
		static const float CATCH_FACTOR;
		static const float CLUTCH_FULL_MAX_TIME;
		static const float USE_LEARNED_OFFSET_RANGE;

		static const float TEAM_REAR_DIST;
		static const int TEAM_DAMAGE_CHANGE_LEAD;
};

#endif // _DRIVER_H_

