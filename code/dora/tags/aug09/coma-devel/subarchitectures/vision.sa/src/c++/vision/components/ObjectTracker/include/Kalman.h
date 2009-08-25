
#ifndef _KALMAN_H_
#define _KALMAN_H_

#include <stdio.h>
#include <cwmtx/cwmtx.h>
//#include <lapackpp/gmd.h>

using namespace CwMtx;

class Kalman
{
private:
	CWSquareMatrix A, At, Pp, P, Q, R, I, S, Si;
	CWMatrix B, H, Ht, K;
	CWMatrix x, xp, z, a;
	
	void setT(float dt);
	
public:
	Kalman();
	~Kalman();

	void init();
	void setX(float* xnew);
	void run(float* zk, float dT, float xk[]);
};

#endif
