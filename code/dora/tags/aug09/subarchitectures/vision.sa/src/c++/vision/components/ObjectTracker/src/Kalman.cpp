
#include "Kalman.h"

void printMx(CWMatrix Mx){
	unsigned int m,n,r,c;
	
	m = Mx.getRows();
	n = Mx.getCols();
	
	for(r=0; r<m; r++){
		for(c=0; c<n; c++){
			printf("%f ", Mx[r][c]);
		}
		printf("\n");
	}
}

Kalman::Kalman(){
	A.dimension(12);	// linear system matrix
	At.dimension(12);	// linear system matrix
	Pp.dimension(12);	// predicted error covariance
	P.dimension(12);	// error covariance
	Q.dimension(12);	// process noise covariance
	R.dimension(6);		// measurement noise covariance
	I.dimension(12);	// eye matrix
	S.dimension(6);		// S = (H*Pp*Ht + R)
	Si.dimension(6);	// S^(-1)
	
	B.dimension(12,6);	// input matrix
	H.dimension(6,12);	// output matrix
	Ht.dimension(12,6);	// H_transposed
	K.dimension(12,6);	// kalman gain
	
	x.dimension(12,1);	// system state
	xp.dimension(12,1);	// predicted system state
	z.dimension(6,1);	// measurement
	a.dimension(6,1);	// acceleration
	
	init();	
}

Kalman::~Kalman(){
	A.deallocate();
	At.deallocate();
	Pp.deallocate();
	P.deallocate();
	Q.deallocate();
	R.deallocate();
	I.deallocate();
	S.deallocate();
	Si.deallocate();
	
	B.deallocate();
	H.deallocate();
	Ht.deallocate();
	K.deallocate();
	
	x.deallocate();
	xp.deallocate();
	z.deallocate();
	a.deallocate();
}

void Kalman::init(){
	float rR = 0.3;
	float rT = 0.15;
	
	float qR = 0.3; //0.08;
	float qT = 0.3;
	
	A.fill(0.0);	// linear system matrix
	At.fill(0.0);	// linear system matrix
	Pp.fill(0.0);	// predicted error covariance
	P.fill(0.0);	// error covariance
	Q.fill(0.0);	// process noise covariance
	R.fill(0.0);	// measurement noise covariance
	I.fill(0.0);	// eye matrix
	S.fill(0.0);	// S = (H*Pp*Ht + R)
	Si.fill(0.0);	// S^(-1)
	
	B.fill(0.0);	// input matrix
	H.fill(0.0);	// output matrix
	Ht.fill(0.0);	// H_transposed
	K.fill(0.0);	// kalman gain
	
	x.fill(0.0);	// system state
	xp.fill(0.0);	// predicted system state
	z.fill(0.0);	// measurement
	a.fill(0.0);	// acceleration
	
	A[0][0]= 1.0; A[1][1] = 1.0; A[2][2] = 1.0; A[3][3] = 1.0; A[4][4] = 1.0; A[5][5] = 1.0;
	A[6][6]= 1.0; A[7][7] = 1.0; A[8][8] = 1.0; A[9][9] = 1.0; A[10][10] = 1.0; A[11][11] = 1.0;
	At.storeTranspose(A);
	
	I[0][0]= 1.0; I[1][1] = 1.0; I[2][2] = 1.0; I[3][3] = 1.0; I[4][4] = 1.0; I[5][5] = 1.0;
	I[6][6]= 1.0; I[7][7] = 1.0; I[8][8] = 1.0; I[9][9] = 1.0; I[10][10] = 1.0; I[11][11] = 1.0;
	
	R[0][0] = rR; R[1][1] = rR; R[2][2] = rR; R[3][3] = rT; R[4][4] = rT; R[5][5] = rT; 
		
	Q[0][0] = qR; Q[1][1] = qR; Q[2][2] = qR; Q[3][3] = qT; Q[4][4] = qT; Q[5][5] = qT;
	Q[6][6] = qR; Q[7][7] = qR; Q[8][8] = qR; Q[9][9] = qT; Q[10][10] = qT; Q[11][11] = qT; 
	
	H[0][0] = 1.0; H[1][1] = 1.0; H[2][2] = 1.0; H[3][3] = 1.0; H[4][4] = 1.0; H[5][5] = 1.0; 
	Ht.storeTranspose(H);
}

// load delta_time intot matrices
void Kalman::setT(float dT){
	float t2 = dT * dT * 0.5;
	
	A[0][6]=dT;	A[1][7]=dT;	A[2][8]=dT; 
	A[3][9]=dT; A[4][10]=dT; A[0][11]=dT;
	At.storeTranspose(A);
	
	B[0][0]=t2; B[1][1]=t2;  B[2][2]=t2;  B[3][3]=t2;  B[4][4]=t2;  B[5][5]=t2;
	B[6][0]=dT; B[7][1]=dT;  B[8][2]=dT;  B[9][3]=dT;  B[10][4]=dT;  B[11][5]=dT;
}

// set internal state
void Kalman::setX(float* xnew){
	
	x[0][0] = xnew[0]; x[1][0] = xnew[1]; x[2][0] = xnew[2];
	x[3][0] = xnew[3]; x[4][0] = xnew[4]; x[5][0] = xnew[5];
	
	x[6][0] = 0.0; x[7][0] = 0.0; x[8][0] = 0.0;
	x[9][0] = 0.0; x[10][0] = 0.0; x[11][0] = 0.0;
	
}

void Kalman::run(float* zk, float dT, float xk[]){
	
	// Load arguments to kalman variables (CWMatrix)
	z[0][0]=zk[0]; z[1][0]=zk[1]; z[2][0]=zk[2]; 
	z[3][0]=zk[3]; z[4][0]=zk[4]; z[5][0]=zk[5];
	setT(dT);
	
	// Time Update "Predict"
	xp = (A*x) + (B*a);
	Pp = (A*(P*At)) + Q;
	
	// Measurement Update "Correct"
	S = (H*(Pp*Ht)) + R;
	Si.storeInverse(S);
	K = Pp*(Ht*Si);
	x = xp + (K*(z-(H*xp)));
	
	// estimate input (=acceleration)
	a = (x-xp)*(2/(dT*dT))*0.3;
	
	// Store result in array for output
	xk[0] = x[0][0]; xk[1] = x[1][0]; xk[2] = x[2][0];
	xk[3] = x[3][0]; xk[4] = x[4][0]; xk[5] = x[5][0];
}


