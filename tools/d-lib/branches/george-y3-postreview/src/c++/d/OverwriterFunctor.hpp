/*
 * File: OverwriterFunctor.hpp
 * Description: functor to change specific fields of a data structure
 * Author: Dorian Galvez Lopez
 * Date: 2008-05-08
 *
 *
 */
 
#ifndef __OVERWRITER_FUNCTOR__
#define __OVERWRITER_FUNCTOR__


/* Class to be inherited to set a function to modify an itme on working memory
 */
template<class T>
class OverwriterFunctor {
public:
	OverwriterFunctor(){}
	/* Sets the desired fields of the data struct
	 */
	virtual void Set(T* data) const = 0;
};

#endif
