#ifndef MATLAB_ICE
#define MATLAB_ICE

#include <cast/slice/CDL.ice>

module Matlab
{
	sequence<long> longSeq;
	sequence<double> doubleSeq;

	struct Matrix
	{
		longSeq dimensions;
		doubleSeq data;
	}; 
	// struct Matrix

}; 
// module Matlab

#endif

