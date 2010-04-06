//#include "stdafx.h"
#include "random.h"
#include "math.h"

bool ExtRandom::deviate_available = false;

double ExtRandom::second_deviate = 0;

double ExtRandom::uniformRandom()
{
	double u;
	u = (double)rand() / (double)RAND_MAX;
	return u;
}

double ExtRandom::gaussRandom(double mu, double sd) 
{
	// Ref. Numerical Recipes in C - Chapter 7.2: Gaussian Deviates.
	if (deviate_available) {
		deviate_available = false;
		return mu + second_deviate * sd;
	} else {
		double v1, v2, rsq;
		do {
			v1 = 2.0 * uniformRandom() - 1.0;
			v2 = 2.0 * uniformRandom() - 1.0;
			rsq = v1*v1 + v2*v2;
		} while (rsq >= 1.0 || rsq == 0.0);
		double fac = sqrt(-2.0*log(rsq)/rsq);
		second_deviate = v1 * fac;
		deviate_available = true;
		return mu + v2 * fac * sd;
	}
}

double ExtRandom::expRandom(double lambda) 
{
	// Ref. Numerical Recipes in C - Chapter 7.2: Exponential Deviates.
	double num;
	do
	num = uniformRandom();
	while (num == 0.0);
	return -log(num) / lambda;
}

double ExtRandom::ComputeGauss(double mu, double sigma, double x)
{
	double re;
	re = x - mu;
	re *= re;
	re = -re/(2*sigma*sigma);
	re = exp(re);

	re = re/(sigma * sqrt(2*PI));

	return re;
}


double ExtRandom::ComputeLnGauss(double mu, double sigma, double v1, double v2, double x)
{
	double re;
	//double factor;

	if ( x < v1 || x > v2 ) // out of range
	{
		return -BigValue;
	}

	// factor = GaussianIntegral(mu, sigma,v1,v2);  // normalization factor
	//	re =  ComputeGauss(mu,sigma,x);
	//	re = re / factor;
	re = x - mu;
	re *=re;
	re = re / (2 * sigma * sigma);

	re = -re;

	//re = - log ( sigma * sqrt(2 * PI)) - re;

	return re;	
}

double ExtRandom::GaussianIntegral(double mu, double sigma, double v1, double v2)
{
	double re = 0;
	double x;
	double delta;
	delta = (v2-v1)/INTEGRALSTEP;
	int i;
	for ( i = 0; i < INTEGRALSTEP; i++ )
	{
		x = v1 + (double)i * delta;
		re = re + ComputeGauss(mu, sigma,x) * delta;
	}
	return re;
}

void ExtRandom::GaussianParaLearning(double &mu, double &sigma, double &v1, double &v2, vector<double> Tdata)
{
	int NumOfTData = Tdata.size();
	int i;
	vector<double>::iterator fiter = Tdata.begin();
	v1 = Tdata[0];
	v2 = Tdata[0];

	mu = 0;
	for ( i = 0; i < NumOfTData; i++ )
	{
		mu += *(fiter+i);
		if ( v1 > Tdata[i] )
		{
			v1 = Tdata[i];
		}

		if ( v2 < Tdata[i] )
		{
			v2 = Tdata[i];
		}
	}
	mu /= NumOfTData;

	sigma = 0;
	for ( i = 0; i < NumOfTData; i++ )
	{
		sigma += pow(*(fiter+i)-mu, 2);
	}
	sigma /= (double)(NumOfTData-1);
	sigma = sqrt(sigma);
}
