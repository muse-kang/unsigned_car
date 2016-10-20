class Fsm {
	int lane;
	int steering;
	int vmax;
public:
	Fsm();
	int GetLane();
	int GetVMax();
	void Update(shared_use_st *m_s);
};
