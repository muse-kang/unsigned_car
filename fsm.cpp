#include "DInterface.h"
#include "fsm.h"

Fsm::Fsm() {
	lane = 1; // 0=���, -1=����, 1=����
	steering = 0; // 0=����, -1=��ȸ��, 1=��ȸ��
	vmax = 20;
}

int Fsm::GetLane() {
	return lane;
}

int Fsm::GetVMax() {
	return vmax;
}

void Fsm::Update(shared_use_st *s) {
	if (s->track_dist_straight < 100) {
		vmax = 50;
		if (s->track_dist_straight < 50) {
			vmax = 20;
			switch(s->track_curve_type) {
			case CURVE_TYPE_LEFT:
				lane = -1;
				break;
			case CURVE_TYPE_RIGHT:
				lane = 1;
				break;
			}
		}
	} else {
		if (s->toStart < 3100) {
			vmax = 20;
		} else if (s->toStart < 3500) {
			vmax = 20;
		} else {
			vmax = 20;
		}
	}
}
