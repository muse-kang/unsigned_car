#include "DInterface.h"
#include "fsm.h"
#include <stdio.h>
#include <math.h>

Fsm::Fsm() {
	stat = 0; // 0=���, 1=����(����), 2=����(����), 3=����(����), 4=stuckȸ��, 5=�߿�/ȸ��, 6=Ŀ���غ�, 7=�ڳʸ�
	lane = 1; // 0=���, -1=����, 1=����
	toLane = 1; // ���� ����
	steering = 0; // 0=����, -1=��ȸ��, 1=��ȸ��
	vmax = 20;
	t0 = GetTickCount();
	delta = 5000;
}

int Fsm::GetStat() {
	return stat;
}

void Fsm::SetStat(int stat) {
	this->stat = stat;
	if (stat == 4) {
		delta = 10000;
		t0 = GetTickCount();
	}
}

float Fsm::GetLane() {
	return lane;
}

int Fsm::GetVMax() {
	return vmax;
}

void Fsm::Update(shared_use_st *s) {
	DWORD t = GetTickCount();
	int i, minIdx, maxIdx;
	float minDist, maxDist;
	double d;

	/* ��� ���Ŀ� �ùķ����ͷκ��� ������ toStart ���� ����Ȯ�Ͽ� ����� �� ���� -_-++
	if (s->toStart < 100) {
		vmax = 20;
	} else if (s->toStart < 200) {
		vmax = 50;
	} else {
		vmax = 100;
	}
	*/

	switch(stat) {
	case 0: // ���
		if (t < t0 + 2000) {
			vmax = 20;
		} else if (t < t0 + 4000) {
			vmax = 40;
		} else if (t > t0 + delta) {
			stat = 1;
			vmax = 60;
			printf("Stat change: from %d to %d\n", 0, stat);
		}
		break;
	case 1: // ����
		minDist = maxDist = s->dist_cars[0];
		minIdx = maxIdx = 0;
		for (i = 2; i < 10; i += 2) {
			d = s->dist_cars[i];
			if (d > maxDist) {
				maxDist = d;
				maxIdx = i;
			}
			if (d < minDist) {
				minDist = d;
				minIdx = i;
			}
		}
		if (minDist < 50) { // �õ��غ��� ��ġ ���� �ʿ�
			d = s->dist_cars[minIdx + 1] - s->toMiddle;
			if (fabs((float) d) < 1.75f) {
				stat = 5;
				t0 = t;
				delta = 1500;
				fromLane = lane;
				if (lane != 0) {
					toLane = 0;
				} else {
					toLane = s->track_curve_type == CURVE_TYPE_LEFT ? -1 : 1;
				}
				printf("Stat change: from %d to %d, fromLane = %.1f, toLane = %.1f\n", 1, stat, fromLane, toLane);
			}
		} else 	if (s->track_dist_straight < 100) {
			vmax = 50;
			stat = 6;
			t0 = t;
			printf("Stat change: from %d to %d\n", 1, stat);
		}
		if (t > t0 + delta && vmax < 100) {
			vmax = 100;
			printf("Set vmax = 100\n");
		}
		break;
	case 4: // stuckȸ��
		if (t > t0 + delta) {
			t0 = t;
			delta = 3000;
			vmax = 50;
			stat = 1;
			printf("Stat change: from %d to %d\n", 4, stat);
		}
		break;
	case 5: // �߿�/ȸ��
		if (t < t0 + delta) {
			vmax = 20;
			lane = fromLane + (float) (t - t0) * (toLane - fromLane) / delta;
		} else {
			vmax = 50;
			stat = 1;
			lane = toLane;
			t0 = t;
			delta = 2000;
			printf("Stat change: from %d to %d\n", 5, stat);
		}
		break;
	case 6: // Ŀ���غ�
		if (s->track_dist_straight < 50) {
			t0 = t;
			delta = 3000;
			stat = 7;
			vmax = 20;
			fromLane = lane;
			switch(s->track_curve_type) {
			case CURVE_TYPE_LEFT:
				toLane = -1.5f;
				break;
			case CURVE_TYPE_RIGHT:
				toLane = 1.5f;
				break;
			}
			printf("Stat change: from %d to %d, fromLane = %.1f, toLane = %.1f\n", 6, stat, fromLane, toLane);
		}
		break;
	case 7: // �ڳʸ�
		if (t < t0 + delta) {
			lane = fromLane + (float) (t - t0) * (toLane - fromLane) / delta;
		} else {
			vmax = 100;
			stat = 1;
			lane = toLane;
			printf("Stat change: from %d to %d\n", 7, stat);
		}
		break;
	}
}
