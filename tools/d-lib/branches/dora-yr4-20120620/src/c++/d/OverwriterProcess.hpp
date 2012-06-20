/*
 * File: OverwriterProcess.hpp
 * Description: class to perform partial overwritings on working memories.
 *   This class makes up for the lack of this in CAST (for the moment).
 * Author: Dorian Galvez Lopez
 * Date: 2008-05-08
 *
 */

#ifndef __OVERWRITER_PROCESS__
#define __OVERWRITER_PROCESS__

#include <cast/architecture/ManagedProcess.hpp>
#include <cast/architecture/PrivilegedManagedProcess.hpp>
#include "OverwriterFunctor.hpp"

using namespace cast;

/* Class to help managed processes overwrite working memory
 */
class OverwriterProcess: 
	public cast::ManagedProcess
{
public:
	OverwriterProcess(const std::string &_id):
		WorkingMemoryAttachedComponent(_id),
		ManagedProcess(_id){}
		
		
	/* Overwrites an item on working memory
	 * @param T: type of data to write
	 * @param id: id to overwrite
	 * @param functor: object with the function to set the value of entry
	 * @param sync: blocking or not
	 */
	template <class T>
	void overwriteWorkingMemory(
		const std::string &id, 
		const OverwriterFunctor<T> &functor,
		const cdl::OperationMode &sync = cdl::NON_BLOCKING);

};

/* Class to help privileged managed processes overwrite working memory
 */
class PrivilegedOverwriterProcess: 
	public cast::PrivilegedManagedProcess
{
public:
	PrivilegedOverwriterProcess(const std::string &_id):
		WorkingMemoryAttachedComponent(_id),
		PrivilegedManagedProcess(_id){}
		
		
	/* Overwrites an item on working memory
	 * @param T: type of data to write
	 * @param id: id to overwrite
	 * @param functor: object with the function to set the value of entry
	 * @param sync: blocking or not
	 */
	template <class T>
	void overwriteWorkingMemory(
		const std::string &id, 
		const OverwriterFunctor<T> &functor,
		const cdl::OperationMode &sync = cdl::NON_BLOCKING);
	
	/* Overwrites an item on working memory
	 * @param T: type of data to write
	 * @param id: id to overwrite
	 * @param sa: sa where the entry is
	 * @param functor: object with the function to set the value of entry
	 * @param sync: blocking or not
	 */
	template <class T>
	void overwriteWorkingMemory(
		const std::string &id, 
		const std::string &sa,
		const OverwriterFunctor<T> &functor,
		const cdl::OperationMode &sync = cdl::NON_BLOCKING);

};

// ---------------------------------------------------------------------------

template<class T>
void PrivilegedOverwriterProcess::overwriteWorkingMemory(
	const std::string &id, 
	const OverwriterFunctor<T> &functor,
	const cdl::OperationMode &sync)
{	
	overwriteWorkingMemory<T>(id, subarchitectureID(), functor, sync);
}

// ---------------------------------------------------------------------------

template <class T>
void PrivilegedOverwriterProcess::overwriteWorkingMemory(
	const std::string &id, 
	const std::string &sa,
	const OverwriterFunctor<T> &functor,
	const cdl::OperationMode &sync)
{
	boost::shared_ptr<const CASTTypedData<T> > pvp;
	T *data;
	bool ok = false;
	
	while(!ok){
		pvp = getWorkingMemoryEntry<T>(id, sa);
		data = new T(*pvp->getData());
		functor.Set(data);
		
		try {
			overwriteWorkingMemory<T>(id, sa, data, sync);
			ok = true;
		}catch(ConsistencyException){
			delete data;
		}
	}
}

// ---------------------------------------------------------------------------

template <class T>
void OverwriterProcess::overwriteWorkingMemory(
	const std::string &id, 
	const OverwriterFunctor<T> &functor,
	const cdl::OperationMode &sync)
{
	boost::shared_ptr<const CASTTypedData<T> > pvp;
	T *data;
	bool ok = false;
	
	while(!ok){
		pvp = getWorkingMemoryEntry<T>(id);
		data = new T(*pvp->getData());
		functor.Set(data);
		
		try {
			overwriteWorkingMemoryHelper<T>(id,subarchitectureID(),data,sync);
			ok = true;
		}catch(ConsistencyException){
			delete data;
		}
	}
}

// ---------------------------------------------------------------------------

#endif

