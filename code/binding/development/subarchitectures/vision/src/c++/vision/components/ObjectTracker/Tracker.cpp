/** @file Tracker.cpp
 *  @brief Generic tracker header. 
 *  
 *  Any specialized traker should derive from this class.
 *
 *  @author Somboon Hongeng
 *  @date march 2007
 *  @bug No known bugs.
 */
#include "Tracker.h"

Tracker::Tracker()  : Visualization()
{
}

Tracker::~Tracker()
{
}

int Tracker::NumActiveObjects()
{   
    int count=0;
    for (int i=0; i<NumObjects(); i++) {
	if (IsObjectActive(i))
	    count++;
    }
    return count;
}
