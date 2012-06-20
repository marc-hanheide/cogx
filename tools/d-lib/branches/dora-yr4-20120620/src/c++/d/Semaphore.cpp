/*
 * File: Semaphore.cpp
 * Description: yet another sempahore implementation
 * Author: Dorian Galvez Lopez
 * Date: 2008-03-17
 *
 */

#include "Semaphore.h"
#include <pthread.h>

Semaphore::Semaphore(int tokens){
	pthread_cond_init(&m_MutexCond, NULL);
	pthread_mutex_init(&m_Mutex, NULL);
	m_Tokens = tokens;
}

// ----------------------------------------------------------------------------

Semaphore::~Semaphore(){
	pthread_cond_broadcast(&m_MutexCond);
	pthread_mutex_unlock(&m_Mutex);
}

// ----------------------------------------------------------------------------

void Semaphore::Signal(){
	pthread_mutex_lock(&m_Mutex);
	if(m_Tokens < 0) m_Tokens = 1; // < 0 means reset
	else m_Tokens++;
	pthread_cond_signal(&m_MutexCond);
	pthread_mutex_unlock(&m_Mutex);
}

// ----------------------------------------------------------------------------

void Semaphore::Wait(){
	pthread_mutex_lock(&m_Mutex);
	if(m_Tokens < 0) m_Tokens = 0; // < 0 means reset
	while(m_Tokens == 0){
		pthread_cond_wait(&m_MutexCond, &m_Mutex);
	}
	m_Tokens--;
	pthread_mutex_unlock(&m_Mutex);
}

// ----------------------------------------------------------------------------

bool Semaphore::Get(){
	
	if(pthread_mutex_trylock(&m_Mutex) != 0) return false;
	
	if(m_Tokens > 0){
		m_Tokens--;
		pthread_mutex_unlock(&m_Mutex);
		return true;
	}else{
		pthread_mutex_unlock(&m_Mutex);
		return false;
	}
}

// ----------------------------------------------------------------------------

void Semaphore::Reset(){
	pthread_mutex_lock(&m_Mutex);
	m_Tokens = -1;
	pthread_cond_broadcast(&m_MutexCond);
	pthread_mutex_unlock(&m_Mutex);
}


