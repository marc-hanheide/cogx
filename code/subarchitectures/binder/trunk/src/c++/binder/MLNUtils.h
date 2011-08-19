/**
 * @author Alen Vrecko
 * @date May 2009, 2011
 */

#ifndef MLN_UTILS_H
#define MLN_UTILS_H

#include <iostream>
#include <vector>
#include <string>

using namespace std;

template<class T> bool containsElement(vector<T> vec, T elem, typename vector<T>::iterator it)
{
	for (it=vec.begin(); it < vec.end(); it++ )
	  if(*it == elem)
		return true;
	
	return false;
}

template<class T> bool containsElement(vector<T> vec, T elem)
{
	typename vector<T>::iterator it;
	return containsElement<T>(vec, elem, it);
}

template<class T> bool containsElement(vector<T> vec, T elem, size_t i)
{
	for (i=0; i < vec.size(); i++ )
	  if(vec[i] == elem)
		return true;
	
	return false;
}


#endif

