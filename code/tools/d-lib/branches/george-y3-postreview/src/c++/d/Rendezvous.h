/*
 * File: Rendezvous.h
 * Description: synchronizes two components using the working memory.
 *  It makes the caller wait until the receiver responses.
 * Author: Dorian Galvez Lopez
 * Date: 2008-02-28
 *
 * The idea is that you can make a component write something to WM and
 * then make it wait until some specific change on WM occurs. This class
 * should allow you to avoid hardcoding all that in several functions in
 * your component. 
 *
 * Example:
 *   Rendezvous rv(*this);
 *   // note these invokes dont have the pReceiver parameter
 *   rv.addChangeFilter(SomeType, cdl::ADD, true);
 *   rv.wait();
 *   // this thread is blocked until an entry of SomeType is added
 *   // by other component.
 *   // At this point, someone added SomeType data to WM.
 *   // The filter is automatically removed when the Rendezvous object is
 *   // destroyed.
 *
 * Note: the Rendezvous object adds and removes filters on your component,
 *   so you must not use on it the same filters your component is listening
 *   to, because they will be removed. This is due to there is not a mechanism
 *   to remove specific filters in cast.
 *
 */

#ifndef __CAST_RENDEZ_VOUS__
#define __CAST_RENDEZ_VOUS__

#include <cast/architecture/ManagedComponent.hpp>
#include <queue>
#include <pthread.h>

using namespace cast;

class Rendezvous
{
public:

	Rendezvous(cast::ManagedComponent &component);

	/* This removes all the filters added by this object
	 */
	virtual ~Rendezvous();

	/* Blocks the calling thread until some event occurs on WM.
	 * You must call this only after adding some change listeners
	 * @return: change which woke this up
	 */
	cast::cdl::WorkingMemoryChange wait();
	
	/* Returns a pointer to the function that must listen to the changes on WM
	 * to signal the wait event
	 * @return: pointer to intern callback function
	 */
	inline WorkingMemoryChangeReceiver* getCallbackFunction(){ return m_Recv; }
	
	/* Sends a manual (fake) change to be listened by the Rendezvous.
	 * This may be useful if you want to listen for
	 * the same event from different sources on the same component. CAST does
	 * not allow that, so one Rendezvous could signal the other.
	 * @attention: the given change ALWAYS is got by the Rendezvous, although
	 *   it does not fit any change filter.
	 * @param change: the change to send to the current Rendezvous
	 */
	void sendManualChange(const cdl::WorkingMemoryChange &change);

	/* These functions are wrappers of the original ones, not to bother
	 * you adding the last parameter
	 */
	inline void addChangeFilter(const cdl::WorkingMemoryChangeFilter &_filter)
	{
		m_AttachedComponent.addChangeFilter(_filter, m_Recv);
	}
	 
	/* // Old CAST version
	
	inline void addChangeFilter(const cdl::WorkingMemoryOperation & _op,
		const bool & _local)
	{
		saveFilter(_op, _local);
		m_AttachedComponent.addChangeFilter(_op, _local, m_Recv);
	}
	
	inline void addChangeFilter(const bool & _local)
	{
		saveFilter(_local);
		m_AttachedComponent.addChangeFilter(_local, m_Recv);
	}
	
	inline void addChangeFilter(const std::string &_type, 
		const bool & _local)
	{
		saveFilter(_type, _local);
		m_AttachedComponent.addChangeFilter(_local, m_Recv);
	}
	
	inline void addChangeFilter(const std::string &_type, 
		const cdl::WorkingMemoryOperation & _op,
		const bool & _local)
	{
		saveFilter(_type, _op, _local);
		m_AttachedComponent.addChangeFilter(_type, _op, _local, m_Recv);
	}
	
	inline void addChangeFilter(const std::string & _type,
		const cdl::WorkingMemoryOperation & _op, 
		const std::string & _src, 
		const std::string & _changeID,
		const std::string & _changeSA, 
		const bool & _local)
	{
		saveFilter(_type, _op, _src, _changeID, _changeSA, _local);
		m_AttachedComponent.addChangeFilter
			(_type, _op, _src, _changeID, _changeSA, _local, m_Recv);
	}
	*/

private:

	/*
	struct tpFilter {
		std::string _type;
		cdl::WorkingMemoryOperation _op;
		std::string _src;
		std::string _changeID;
		std::string _changeSA;
		bool _local;
	};
	std::vector<tpFilter> m_Filters;
	*/

	cast::ManagedComponent& m_AttachedComponent;
	WorkingMemoryChangeReceiver *m_Recv;
	std::queue<cast::cdl::WorkingMemoryChange> m_Changes;
	// Mutex and condition variable to access to m_Changes and to wait 
	// for it to have items
	pthread_mutex_t m_Mutex;
	pthread_cond_t m_MutexCond;

	/* The callback function for all the change filters
	 */
	void getWorkingMemoryChange(const cast::cdl::WorkingMemoryChange & objID);
	
	/* Save filters on m_Filters
	 // Old CAST version
	void saveFilter(const cdl::WorkingMemoryOperation & _op,
		const bool & _local);
	void saveFilter(const bool & _local);
	void saveFilter(const std::string &_type, const bool & _local);
	void saveFilter(const std::string &_type,
		const cdl::WorkingMemoryOperation & _op, const bool & _local);
	void saveFilter(const std::string & _type,
		const cdl::WorkingMemoryOperation & _op, 
		const std::string & _src, 
		const std::string & _changeID,
		const std::string & _changeSA, 
		const bool & _local);
	*/
};

#endif
