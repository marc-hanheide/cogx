////////////////////////////////////////////////////////////////////////////////
// Common host and device functions
////////////////////////////////////////////////////////////////////////////////

#ifndef _ALIGN_H_
#define _ALIGN_H_

//Round a / b to nearest higher integer value
inline int iDivUp(int a, int b){
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

//Round a / b to nearest lower integer value
inline int iDivDown(int a, int b){
    return a / b;
}

//Align a to nearest higher multiple of b
inline int iAlignUp(int a, int b){
    return (a % b != 0) ?  (a - a % b + b) : a;
}

//Align a to nearest lower multiple of b
inline int iAlignDown(int a, int b){
    return a - a % b;
}

#endif