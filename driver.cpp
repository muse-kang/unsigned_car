/***************************************************************************

    file                 : driver.cpp
    created              : Thu Dec 20 01:21:49 CET 2002
    copyright            : (C) 2002-2004 Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: driver.cpp,v 1.16.2.2 2008/12/31 03:53:53 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "driver.h"

const float Driver::MAX_UNSTUCK_ANGLE = 15.0f/180.0f*PI;	// [radians] If the angle of the car on the track is smaller, we assume we are not m_stuck.
const float Driver::UNSTUCK_TIME_LIMIT = 2.0f;				// [s] We try to get unstuck after this time.
const float Driver::MAX_UNSTUCK_SPEED = 5.0f;				// [m/s] Below this speed we consider being m_stuck.
const float Driver::MIN_UNSTUCK_DIST = 3.0f;				// [m] If we are closer to the middle we assume to be not m_stuck.
const float Driver::G = 9.81f;								// [m/(s*s)] Welcome on Earth.
const float Driver::FULL_ACCEL_MARGIN = 1.0f;				// [m/s] Margin reduce oscillation of brake/acceleration.
const float Driver::SHIFT = 0.9f;							// [-] (% of rpmredline) When do we like to shift gears.
const float Driver::SHIFT_MARGIN = 4.0f;					// [m/s] Avoid oscillating gear changes.
const float Driver::ABS_SLIP = 2.0f;						// [m/s] range [0..10]
const float Driver::ABS_RANGE = 5.0f;						// [m/s] range [0..10]
const float Driver::ABS_MINSPEED = 3.0f;					// [m/s] Below this speed the ABS is disabled (numeric, division by small numbers).
const float Driver::TCL_SLIP = 2.0f;						// [m/s] range [0..10]
const float Driver::TCL_RANGE = 10.0f;						// [m/s] range [0..10]
const float Driver::LOOKAHEAD_CONST = 17.0f;				// [m]
const float Driver::LOOKAHEAD_FACTOR = 0.33f;				// [-]
const float Driver::WIDTHDIV = 3.0f;						// [-] Defines the percentage of the track to use (2/WIDTHDIV).
const float Driver::SIDECOLL_MARGIN = 3.0f;					// [m] Distance between car centers to avoid side collisions.
const float Driver::BORDER_OVERTAKE_MARGIN = 0.5f;			// [m]
const float Driver::OVERTAKE_OFFSET_SPEED = 5.0f;			// [m/s] Offset change speed.
const float Driver::PIT_LOOKAHEAD = 6.0f;					// [m] Lookahead to stop in the pit.
const float Driver::PIT_BRAKE_AHEAD = 200.0f;				// [m] Workaround for "broken" pitentries.
const float Driver::PIT_MU = 0.4f;							// [-] Friction of pit concrete.
const float Driver::MAX_SPEED = 84.0f;						// [m/s] Speed to compute the percentage of brake to apply.
const float Driver::MAX_FUEL_PER_METER = 0.0008f;			// [liter/m] fuel consumtion.
const float Driver::CLUTCH_SPEED = 5.0f;					// [m/s]
const float Driver::CENTERDIV = 0.1f;						// [-] (factor) [0.01..0.6].
const float Driver::DISTCUTOFF = 200.0f;					// [m] How far to look, terminate while loops.
const float Driver::MAX_INC_FACTOR = 5.0f;					// [m] Increment faster if speed is slow [1.0..10.0].
const float Driver::CATCH_FACTOR = 10.0f;					// [-] select MIN(catchdist, dist*CATCH_FACTOR) to overtake.
const float Driver::CLUTCH_FULL_MAX_TIME = 2.0f;			// [s] Time to apply full clutch.
const float Driver::USE_LEARNED_OFFSET_RANGE = 0.2f;		// [m] if offset < this use the learned stuff

const float Driver::TEAM_REAR_DIST = 50.0f;					//
const int Driver::TEAM_DAMAGE_CHANGE_LEAD = 700;			// When to change position in the team?


Driver::Driver()
{
	m_s = NULL;
	m_stuck = 0;

	float deltaTime = (float)RCM_MAX_DT_ROBOTS;
	m_MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT / deltaTime);
	m_OVERTAKE_OFFSET_INC = OVERTAKE_OFFSET_SPEED*deltaTime;
}


Driver::~Driver()
{
}


// Check if I'm m_stuck.
bool Driver::isStuck()
{
	if (fabs(m_s->angle) > MAX_UNSTUCK_ANGLE &&
		m_s->speed < MAX_UNSTUCK_SPEED &&
		fabs(m_s->toMiddle) > MIN_UNSTUCK_DIST) 
	{
		if (m_stuck > m_MAX_UNSTUCK_COUNT && m_s->toMiddle*m_s->angle < 0.0) 
		{
			return true;
		}
		else 
		{
			m_stuck++;
			return false;
		}
	}
	else 
	{
		m_stuck = 0;
		return false;
	}
}


// Drive during race.
void Driver::drive(shared_use_st *s)
{
	m_s = s;

	if (isStuck()) 
	{
		m_s->steerCmd = -m_s->angle / STEER_LOCK;
		m_s->backwardCmd = GEAR_BACKWARD; // Reverse gear.
		m_s->accelCmd = 1.0f;			  // 100% accelerator pedal.
		m_s->brakeCmd = 0.0f;	          // No brakes.
	}
	else 
	{
		m_s->steerCmd = (m_s->angle - m_s->toMiddle / m_s->track_width) / STEER_LOCK;
		m_s->backwardCmd = GEAR_FORWARD;
		m_s->brakeCmd = 0.0f;
		m_s->accelCmd = 0.2f;



		double c = 2.772;
		double d = -0.693;
		double min, v;
		double vmax = 20.0;

		min = 20.0;
		for (int i = 0; i < 10; i += 2)
		{
			v = vmax * (1 - exp(-c / vmax*m_s->dist_cars[i] - d));

			if (v < min)
				min = v;
		}

		if (min < m_s->speed)
		{
			m_s->brakeCmd = min / m_s->speed;
			m_s->accelCmd = 0;
		}
		else
		{
			m_s->accelCmd = min / m_s->speed;
			m_s->brakeCmd = 0;
		}


#ifdef SMS
		m_s->steerCmd = filterSColl(getSteer());
		m_s->backwardCmd = getGear();
		m_s->brakeCmd = filterBrakeSpeed(filterBColl(getBrake()));
		if (m_s->brakeCmd == 0.0f) 
		{
			m_s->accelCmd = filterTrk(filterOverlap(getAccel()));
		}
		else 
		{
			m_s->accelCmd = 0.0f;
		}
#endif
	}
}



void Driver::computeRadius(float *radius)
{
#ifdef SMS
	float lastturnarc = 0.0f;
	int lastsegtype = TR_STR;

	tTrackSeg *currentseg, *startseg = track->seg;
	currentseg = startseg;

	do {
		if (currentseg->type == TR_STR) {
			lastsegtype = TR_STR;
			radius[currentseg->id] = FLT_MAX;
		} else {
			if (currentseg->type != lastsegtype) {
				float arc = 0.0f;
				tTrackSeg *s = currentseg;
				lastsegtype = currentseg->type;

				while (s->type == lastsegtype && arc < PI/2.0f) {
					arc += s->arc;
					s = s->next;
				}
				lastturnarc = arc/(PI/2.0f);
			}
			radius[currentseg->id] = (currentseg->radius + currentseg->width/2.0)/lastturnarc;
		}
		currentseg = currentseg->next;
	} while (currentseg != startseg);

#endif

}


// Compute the allowed speed on a segment.
float Driver::getAllowedSpeed()
{
#ifdef SMS
	float mu = segment->surface->kFriction*TIREMU*MU_FACTOR;
	float r = radius[segment->id];
	float dr = learn->getRadius(segment);
	if (dr < 0.0f) {
		r += dr;
	} else {
		float tdr = dr*(1.0f - MIN(1.0f, fabs(myoffset)*2.0f/segment->width));
		r += tdr;	
	}
	// README: the outcommented code is the more save version.
	/*if ((alone > 0 && fabs(myoffset) < USE_LEARNED_OFFSET_RANGE) ||
		dr < 0.0f
	) {
		r += dr;
	}*/
	r = MAX(1.0, r);

	return sqrt((mu*G*r)/(1.0f - MIN(1.0f, r*CA*mu/mass)));
#endif

	return	1.0;
}


// Compute the length to the end of the segment.
float Driver::getDistToSegEnd()
{
#ifdef SMS
	if (car->_trkPos.seg->type == TR_STR) {
		return car->_trkPos.seg->length - car->_trkPos.toStart;
	} else {
		return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
	}
#endif

	return	1.0;
}


// Compute fitting acceleration.
float Driver::getAccel()
{
#ifdef SMS
	if (car->_gear > 0) {
		float allowedspeed = getAllowedSpeed(car->_trkPos.seg);
		if (allowedspeed > m_s->speed + FULL_ACCEL_MARGIN) {
			return 1.0;
		} else {
			float gr = car->_gearRatio[car->_gear + car->_gearOffset];
			float rm = car->_enginerpmRedLine;
			return allowedspeed/car->_wheelRadius(REAR_RGT)*gr /rm;
		}
	} else {
		return 1.0;
	}
#endif

	return	1.0;

}


// If we get lapped reduce accelerator.
float Driver::filterOverlap(float accel)
{
#ifdef SMS
	int i;
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if (opponent[i].getState() & OPP_LETPASS) {
			return MIN(accel, 0.5f);
		}
	}
	return accel;
#endif

	return 1.0;
}


// Compute initial brake value.
float Driver::getBrake()
{
#ifdef SMS
	// Car drives backward?
	if (m_s->speed < -MAX_UNSTUCK_SPEED) {
		// Yes, brake.
		return 1.0;
	} else {
		// We drive forward, normal braking.
		tTrackSeg *segptr = car->_trkPos.seg;
		float mu = segptr->surface->kFriction;
		float maxlookaheaddist = currentspeedsqr/(2.0f*mu*G);
		float lookaheaddist = getDistToSegEnd();

		float allowedspeed = getAllowedSpeed(segptr);
		if (allowedspeed < m_s->speed) {
			return MIN(1.0f, (m_s->speed-allowedspeed)/(FULL_ACCEL_MARGIN));
		}

		segptr = segptr->next;
		while (lookaheaddist < maxlookaheaddist) {
			allowedspeed = getAllowedSpeed(segptr);
			if (allowedspeed < m_s->speed) {
				if (brakedist(allowedspeed, mu) > lookaheaddist) {
					return 1.0f;
				}
			}
			lookaheaddist += segptr->length;
			segptr = segptr->next;
		}
		return 0.0f;
	}
#endif

	return 1.0;
}


// Compute gear.
int Driver::getGear()
{
#ifdef SMS
	if (car->_gear <= 0) {
		return 1;
	}
	float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
	float omega = car->_enginerpmRedLine/gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*SHIFT < m_s->speed) {
		return car->_gear + 1;
	} else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = car->_enginerpmRedLine/gr_down;
		if (car->_gear > 1 && omega*wr*SHIFT > m_s->speed + SHIFT_MARGIN) {
			return car->_gear - 1;
		}
	}
	return car->_gear;
#endif

	return 1;
}


// Compute steer value.
float Driver::getSteer()
{
#ifdef SMS
	float targetAngle;
	vec2f target = getTargetPoint();

	targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
	targetAngle -= car->_yaw;
	NORM_PI_PI(targetAngle);
	return targetAngle / STEER_LOCK;
#endif

	return	1.0;
}



// Compute target point for steering.
v2d Driver::getTargetPoint()
{
#ifdef SMS
	tTrackSeg *seg = car->_trkPos.seg;
	float lookahead;
	float length = getDistToSegEnd();
	float offset = getOffset();

	if (pit->getInPit()) {
		// To stop in the pit we need special lookahead values.
		if (currentspeedsqr > pit->getSpeedlimitSqr()) {
			lookahead = PIT_LOOKAHEAD + m_s->speed*LOOKAHEAD_FACTOR;
		} else {
			lookahead = PIT_LOOKAHEAD;
		}
	} else {
		// Usual lookahead.
		lookahead = LOOKAHEAD_CONST + m_s->speed*LOOKAHEAD_FACTOR;
		// Prevent "snap back" of lookahead on harsh braking.
		float cmplookahead = oldlookahead - m_s->speed*RCM_MAX_DT_ROBOTS;
		if (lookahead < cmplookahead) {
			lookahead = cmplookahead;
		}
	}

	oldlookahead = lookahead;

	// Search for the segment containing the target point.
	while (length < lookahead) {
		seg = seg->next;
		length += seg->length;
	}

	length = lookahead - length + seg->length;
	float fromstart = seg->lgfromstart;
	fromstart += length;

	// Compute the target point.
	offset = myoffset = pit->getPitOffset(offset, fromstart);

	vec2f s;
	s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0f;
	s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0f;

	if ( seg->type == TR_STR) {
		vec2f d, n;
		n.x = (seg->vertex[TR_EL].x - seg->vertex[TR_ER].x)/seg->length;
		n.y = (seg->vertex[TR_EL].y - seg->vertex[TR_ER].y)/seg->length;
		n.normalize();
		d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x)/seg->length;
		d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y)/seg->length;
		return s + d*length + offset*n;
	} else {
		vec2f c, n;
		c.x = seg->center.x;
		c.y = seg->center.y;
		float arc = length/seg->radius;
		float arcsign = (seg->type == TR_RGT) ? -1.0f : 1.0f;
		arc = arc*arcsign;
		s = s.rotate(c, arc);

		n = c - s;
		n.normalize();
		return s + arcsign*offset*n;
	}
#endif

	return v2d();
}


// Compute offset to normal target point for overtaking or let pass an opponent.
float Driver::getOffset()
{
#ifdef SMS
	int i;
	float catchdist, mincatchdist = FLT_MAX, mindist = -1000.0f;
	Opponent *o = NULL;

	// Increment speed dependent.
	float incfactor = MAX_INC_FACTOR - MIN(fabs(m_s->speed)/MAX_INC_FACTOR, (MAX_INC_FACTOR - 1.0f));

	// Let overlap or let less damaged team mate pass.
	for (i = 0; i < opponents->getNOpponents(); i++) {
		// Let the teammate with less damage overtake to use slipstreaming.
		// The position change happens when the damage difference is greater than
		// TEAM_DAMAGE_CHANGE_LEAD.
		if (((opponent[i].getState() & OPP_LETPASS) && !opponent[i].isTeamMate()) ||
			(opponent[i].isTeamMate() && (car->_dammage - opponent[i].getDamage() > TEAM_DAMAGE_CHANGE_LEAD) &&
			(opponent[i].getDistance() > -TEAM_REAR_DIST) && (opponent[i].getDistance() < -car->_dimension_x) &&
			car->race.laps == opponent[i].getCarPtr()->race.laps))
		{
			// Behind, larger distances are smaller ("more negative").
			if (opponent[i].getDistance() > mindist) {
				mindist = opponent[i].getDistance();
				o = &opponent[i];
			}
		}
	}

	if (o != NULL) {
		tCarElt *ocar = o->getCarPtr();
		float side = m_s->toMiddle - om_s->toMiddle;
		float w = car->_trkPos.seg->width/WIDTHDIV-BORDER_OVERTAKE_MARGIN;
		if (side > 0.0f) {
			if (myoffset < w) {
				myoffset += m_OVERTAKE_OFFSET_INC*incfactor;
			}
		} else {
			if (myoffset > -w) {
				myoffset -= m_OVERTAKE_OFFSET_INC*incfactor;
			}
		}
		return myoffset;
	}


	// Overtake.
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if ((opponent[i].getState() & OPP_FRONT) &&
			!(opponent[i].isTeamMate() && car->race.laps <= opponent[i].getCarPtr()->race.laps))
		{
			catchdist = MIN(opponent[i].getCatchDist(), opponent[i].getDistance()*CATCH_FACTOR);
			if (catchdist < mincatchdist) {
				mincatchdist = catchdist;
				o = &opponent[i];
			}
		}
	}

	if (o != NULL) {
		// Compute the width around the middle which we can use for overtaking.
		float w = o->getCarPtr()->_trkPos.seg->width/WIDTHDIV-BORDER_OVERTAKE_MARGIN;
		// Compute the opponents distance to the middle.
		float otm = o->getCarPtr()->_trkPos.toMiddle;
		// Define the with of the middle range.
		float wm = o->getCarPtr()->_trkPos.seg->width*CENTERDIV;

		if (otm > wm && myoffset > -w) {
			myoffset -= m_OVERTAKE_OFFSET_INC*incfactor;
		} else if (otm < -wm && myoffset < w) {
			myoffset += m_OVERTAKE_OFFSET_INC*incfactor;
		} else {
			// If the opponent is near the middle we try to move the offset toward
			// the inside of the expected turn.
			// Try to find out the characteristic of the track up to catchdist.
			tTrackSeg *seg = car->_trkPos.seg;
			float length = getDistToSegEnd();
			float oldlen, seglen = length;
			float lenright = 0.0f, lenleft = 0.0f;
			mincatchdist = MIN(mincatchdist, DISTCUTOFF);

			do {
				switch (seg->type) {
				case TR_LFT:
					lenleft += seglen;
					break;
				case TR_RGT:
					lenright += seglen;
					break;
				default:
					// Do nothing.
					break;
				}
				seg = seg->next;
				seglen = seg->length;
				oldlen = length;
				length += seglen;
			} while (oldlen < mincatchdist);

			// If we are on a straight look for the next turn.
			if (lenleft == 0.0f && lenright == 0.0f) {
				while (seg->type == TR_STR) {
					seg = seg->next;
				}
				// Assume: left or right if not straight.
				if (seg->type == TR_LFT) {
					lenleft = 1.0f;
				} else {
					lenright = 1.0f;
				}
			}

			// Because we are inside we can go to the border.
			float maxoff = (o->getCarPtr()->_trkPos.seg->width - car->_dimension_y)/2.0-BORDER_OVERTAKE_MARGIN;
			if (lenleft > lenright) {
				if (myoffset < maxoff) {
					myoffset += m_OVERTAKE_OFFSET_INC*incfactor;
				}
			} else {
				if (myoffset > -maxoff) {
					myoffset -= m_OVERTAKE_OFFSET_INC*incfactor;
				}
			}
		}
	} else {
		// There is no opponent to overtake, so the offset goes slowly back to zero.
		if (myoffset > m_OVERTAKE_OFFSET_INC) {
			myoffset -= m_OVERTAKE_OFFSET_INC;
		} else if (myoffset < -m_OVERTAKE_OFFSET_INC) {
			myoffset += m_OVERTAKE_OFFSET_INC;
		} else {
			myoffset = 0.0f;
		}
	}

	return myoffset;
#endif

	return	1.0;
}


int Driver::isAlone()
{
#ifdef SMS
	int i;
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if (opponent[i].getState() & (OPP_COLL | OPP_LETPASS)) {
			return 0;	// Not alone.
		}
	}
	return 1;	// Alone.
#endif

	return 1;
}





// Reduces the brake value such that it fits the speed (more downforce -> more braking).
float Driver::filterBrakeSpeed(float brake)
{
#ifdef SMS
	float weight = (CARMASS + car->_fuel)*G;
	float maxForce = weight + CA*MAX_SPEED*MAX_SPEED;
	float force = weight + CA*currentspeedsqr;
	return brake*force/maxForce;
#endif

	return 1.0;
}




// Brake filter for collision avoidance.
float Driver::filterBColl(float brake)
{
#ifdef SMS
	float mu = car->_trkPos.seg->surface->kFriction;
	int i;
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if (opponent[i].getState() & OPP_COLL) {
			if (brakedist(opponent[i].getSpeed(), mu) > opponent[i].getDistance()) {
				return 1.0f;
			}
		}
	}
	return brake;
#endif

	return 1.0;
}


// Steer filter for collision avoidance.
float Driver::filterSColl(float steer)
{
#ifdef SMS
	int i;
	float sidedist = 0.0f, fsidedist = 0.0f, minsidedist = FLT_MAX;
	Opponent *o = NULL;

	// Get the index of the nearest car (o).
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if (opponent[i].getState() & OPP_SIDE) {
			sidedist = opponent[i].getSideDist();
			fsidedist = fabs(sidedist);
			if (fsidedist < minsidedist) {
				minsidedist = fsidedist;
				o = &opponent[i];
			}
		}
	}

	// If there is another car handle the situation.
	if (o != NULL) {
		float d = fsidedist - o->getWidth();
		// Near, so we need to look at it.
		if (d < SIDECOLL_MARGIN) {
			/* compute angle between cars */
			tCarElt *ocar = o->getCarPtr();
			float diffangle = ocar->_yaw - car->_yaw;
			NORM_PI_PI(diffangle);
			// We are near and heading toward the car.
			if (diffangle*o->getSideDist() < 0.0f) {
				const float c = SIDECOLL_MARGIN/2.0f;
				d = d - c;
				if (d < 0.0f) {
					d = 0.0f;
				}

				// Steer delta required to drive parallel to the opponent.
				float psteer = diffangle/STEER_LOCK;
				myoffset = m_s->toMiddle;

				// Limit myoffset to suitable limits.
				float w = o->getCarPtr()->_trkPos.seg->width/WIDTHDIV-BORDER_OVERTAKE_MARGIN;
				if (fabs(myoffset) > w) {
					myoffset = (myoffset > 0.0f) ? w : -w;
				}
				
				// On straights the car near to the middle can correct more, in turns the car inside
				// the turn does (because if you leave the track on the turn "inside" you will skid
				// back to the track.
				if (car->_trkPos.seg->type == TR_STR) {
					if (fabs(m_s->toMiddle) > fabs(om_s->toMiddle)) {
						// Its me, I do correct not that much.
						psteer = steer*(d/c) + 1.5f*psteer*(1.0f - d/c);
					} else {
						// Its the opponent, so I correct more.
						psteer = steer*(d/c) + 2.0f*psteer*(1.0f - d/c);
					}
				} else {
					// Who is outside, heavy corrections are less dangerous
					// if you drive near the middle of the track.
					float outside = m_s->toMiddle - om_s->toMiddle;
					float sign = (car->_trkPos.seg->type == TR_RGT) ? 1.0f : -1.0f;
					if (outside*sign > 0.0f) {
						psteer = steer*(d/c) + 1.5f*psteer*(1.0f - d/c);
					} else {
						psteer = steer*(d/c) + 2.0f*psteer*(1.0f - d/c);
					}
				}

				if (psteer*steer > 0.0f && fabs(steer) > fabs(psteer)) {
					return steer;
				} else {
					return psteer;
				}
			}
		}
	}
	return steer;
#endif

	return 1.0;
}


// Antilocking filter for brakes.
float Driver::filterABS(float brake)
{
#ifdef SMS
	if (m_s->speed < ABS_MINSPEED) return brake;
	int i;
	float slip = 0.0f;
	for (i = 0; i < 4; i++) {
		slip += car->_wheelSpinVel(i) * car->_wheelRadius(i);
	}
	slip = m_s->speed - slip/4.0f;
	if (slip > ABS_SLIP) {
		brake = brake - MIN(brake, (slip - ABS_SLIP)/ABS_RANGE);
	}
	return brake;
#endif

	return 1.0;
}


// TCL filter for accelerator pedal.
float Driver::filterTCL(float accel)
{
#ifdef SMS
	float slip = (this->*GET_DRIVEN_WHEEL_SPEED)() - m_s->speed;
	if (slip > TCL_SLIP) {
		accel = accel - MIN(accel, (slip - TCL_SLIP)/TCL_RANGE);
	}
	return accel;
#endif
	return 1.0;
}




// Hold car on the track.
float Driver::filterTrk(float accel)
{
#ifdef SMS
	tTrackSeg* seg = car->_trkPos.seg;

	if (m_s->speed < MAX_UNSTUCK_SPEED ||		// Too slow.
		pit->getInPit() ||							// Pit stop.
		m_s->toMiddle*speedangle > 0.0f)	// Speedvector points to the inside of the turn.
	{
		return accel;
	}

	if (seg->type == TR_STR) {
		float tm = fabs(m_s->toMiddle);
		float w = (seg->width - car->_dimension_y)/2.0f ;
		if (tm > w) {
			return 0.0f;
		} else {
			return accel;
		}
	} else {
		float sign = (seg->type == TR_RGT) ? -1.0f : 1.0f;
		if (m_s->toMiddle*sign > 0.0f) {
			return accel;
		} else {
			float tm = fabs(m_s->toMiddle);
			float w = seg->width/WIDTHDIV;
			if (tm > w) {
				return 0.0f;
			} else {
				return accel;
			}
		}
	}
#endif

	return 1.0;
}


// Compute the needed distance to brake.
float Driver::brakedist(float allowedspeed)
{
#ifdef SMS
	float c = mu*G;
	float d = (CA*mu + CW)/mass;
	float v1sqr = currentspeedsqr;
	float v2sqr = allowedspeed*allowedspeed;
	return -log((c + v2sqr*d)/(c + v1sqr*d))/(2.0f*d);
#endif

	return 1.0;
}

