/** @file Parallels.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/Parallels.h>
#include <Golem/Tools/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Job::Job(Parallels *pParallels, MSecTmU32 threadTimeOut) : pParallels(pParallels), threadTimeOut(threadTimeOut) {
	pRunnable = NULL;
	evJob.set(true);
}

Job::~Job() {
}

void Job::run() {
	for (;;) {
		if (pParallels->bStop)
			break;
		if (!evThread.wait(threadTimeOut))
			continue;
		if (pParallels->bStop)
			break;

		if (pRunnable != NULL) {
			pRunnable->run();
			pRunnable = NULL;
			
			CriticalSectionWrapper csw(pParallels->cs);
			pParallels->freeJobs.insert(this);
			pParallels->sm.release();
		}

		evThread.set(false);
		evJob.set(true);
	}
}

bool Job::startThread() {
	return thread.start(this);
}

bool Job::joinThread(MSecTmU32 timeOut) {
	evThread.set(true);
	return thread.join(timeOut);
}
	
bool Job::startJob(Runnable *pRunnable, MSecTmU32 timeOut) {
	if (!evJob.wait(timeOut))
		return false;
	
	this->pRunnable = pRunnable;
	evJob.set(false);
	evThread.set(true);
	return true;
}

bool Job::joinJob(MSecTmU32 timeOut) {
	return evJob.wait(timeOut);
}

bool Job::setThreadPriority(Thread::Priority priority) {
	return thread.setPriority(priority);
}

Thread::Priority Job::getThreadPriority() const {
	return thread.getPriority();
}

U32 Job::getThreadId() const {
	return thread.getThreadId();
}

//------------------------------------------------------------------------------

Parallels::Parallels(U32 numOfThreads, MSecTmU32 threadTimeOut) : sm((int)numOfThreads, (int)numOfThreads) {
	bStop = false;

	for (U32 i = 0; i < numOfThreads; i++) {
		Job::Ptr pJob(new Job(this, threadTimeOut));
		jobs.push_back(pJob);
		freeJobs.insert(pJob.get());
	}
}

Parallels::~Parallels() {
}

void Parallels::startThreads() {
	for (Job::Seq::iterator i = jobs.begin(); i != jobs.end(); i++)
		if (!(*i)->startThread())
			throw MsgParallelsThreadLaunch(Message::LEVEL_CRIT, "Parallels::startThreads(): Unable to launch Parallels thread");
}

bool Parallels::joinThreads(MSecTmU32 timeOut) {
	SysTimer tm;
	
	bool bRet = true;
	bStop = true;
	for (Job::Seq::iterator i = jobs.begin(); i != jobs.end(); i++) {
		MSecTmU32 t = tm.elapsed();
		if (!(*i)->joinThread(timeOut > t ? timeOut - t : MSecTmU32(0)))
			bRet = false;
	}
	
	return bRet;
}

Job* Parallels::startJob(Runnable *pRunnable, MSecTmU32 timeOut) {
	SysTimer tm;

	if (!sm.wait(timeOut))
		return NULL;

	Job *pJob;

	{
		CriticalSectionWrapper csw(cs);
		Job::Set::iterator pos = freeJobs.begin();
		if (pos == freeJobs.end())
			return NULL;
		pJob = *pos;
		freeJobs.erase(pos);
		if (pJob == NULL)
			return NULL;
	}

	MSecTmU32 t = tm.elapsed();
	if (!pJob->startJob(pRunnable, timeOut > t ? timeOut - t : MSecTmU32(0))) {
		CriticalSectionWrapper csw(cs);
		freeJobs.insert(pJob);
		return NULL;
	}

	return pJob;
}

bool Parallels::joinJob(Job *pJob, MSecTmU32 timeOut) {
	return pJob->joinJob(timeOut);
}

bool Parallels::joinJobs(MSecTmU32 timeOut) {
	SysTimer tm;
	
	bool bRet = true;
	for (Job::Seq::iterator i = jobs.begin(); i != jobs.end(); i++) {
		MSecTmU32 t = tm.elapsed();
		if (!(*i)->joinJob(timeOut > t ? timeOut - t : MSecTmU32(0)))
			bRet = false;
	}
	
	return bRet;
}

U32 Parallels::getNumOfThreads() const {
	return (U32)jobs.size();
}

//------------------------------------------------------------------------------
