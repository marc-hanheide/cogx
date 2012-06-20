/*
 * File: Semaphore.h
 * Description: yet another sempahore implementation
 * Author: Dorian Galvez Lopez
 * Date: 2008-03-17
 *
 */

#ifndef __D_SEMAPHORE__
#define __D_SEMAPHORE__

#include <pthread.h>

class Semaphore {
public:
	Semaphore(int tokens = 0);
	virtual ~Semaphore();
	
	/* Sets the number of tokens to 0 and signals all the listeners
	 */
	void Reset();
	
	/* Sets the number of tokens +1 and wakes up one listener, if any
	 */
	void Signal();
	
	/* Waits for tokens sets by Signal
	 */
	void Wait();
	
	/* Gets a token if there is any, but does not wait if there is not.
	 * Return if could get any
	 * @return: true iif token got
	 */
	bool Get();
	
protected:
	int m_Tokens;

	pthread_mutex_t m_Mutex;
	pthread_cond_t m_MutexCond;	

};

#endif
