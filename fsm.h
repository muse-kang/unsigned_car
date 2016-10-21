#include <windows.h>

class Fsm {
	int stat;
	float lane;
	float fromLane;
	float toLane;
	int steering;
	int vmax;
	DWORD t0;
	DWORD delta;
public:
	Fsm();
	int GetStat();
	void SetStat(int stat);
	float GetLane();
	int GetVMax();
	void Update(shared_use_st *m_s);
};
