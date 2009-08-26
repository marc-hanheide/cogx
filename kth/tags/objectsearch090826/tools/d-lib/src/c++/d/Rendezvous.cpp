/*
 * File: Rendezvous.h
 * Description: synchronizes two components using the working memory.
 *  It makes the caller wait until the receiver responses.
 * Author: Dorian Galvez Lopez
 * Date: 2008-02-28
 *
 */

#include "Rendezvous.h"
#include <pthread.h>

using namespace cast;

// ----------------------------------------------------------------------------

Rendezvous::Rendezvous(cast::ManagedComponent &component):
	m_AttachedComponent(component)
{
	m_Recv = new MemberFunctionChangeReceiver<Rendezvous>
		(this, &Rendezvous::getWorkingMemoryChange);
	
	pthread_cond_init(&m_MutexCond, NULL);
	pthread_mutex_init(&m_Mutex, NULL);
}

// ----------------------------------------------------------------------------

Rendezvous::~Rendezvous(){
	// Remove filters
	m_AttachedComponent.removeChangeFilter(m_Recv, cdl::DELETERECEIVER);
}

// ----------------------------------------------------------------------------

cdl::WorkingMemoryChange Rendezvous::wait(){
	pthread_mutex_lock(&m_Mutex);
	while (m_Changes.size() == 0){
		pthread_cond_wait(&m_MutexCond, &m_Mutex);
	}
	cast::cdl::WorkingMemoryChange c = m_Changes.front();
	m_Changes.pop();
	pthread_mutex_unlock(&m_Mutex);
	
	return c;
}

// ----------------------------------------------------------------------------

void Rendezvous::getWorkingMemoryChange
	(const cast::cdl::WorkingMemoryChange & objID)
{
	pthread_mutex_lock(&m_Mutex);
	m_Changes.push(objID);
	pthread_cond_signal(&m_MutexCond);
	pthread_mutex_unlock(&m_Mutex);
}

// ----------------------------------------------------------------------------

void Rendezvous::sendManualChange(const cdl::WorkingMemoryChange &change){
	getWorkingMemoryChange(change);	
}

// ----------------------------------------------------------------------------




