/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef CAST_QUEUED_DATA_THREAD_HPP_
#define CAST_QUEUED_DATA_THREAD_HPP_


#include <queue>

namespace cast {

  template <class QueuedData>
  class QueuedDataThread : public omni_thread {
    
  public:
    
    QueuedDataThread() 
      : omni_thread(NULL, PRIORITY_NORMAL) {
      m_bRun = false;
      
      m_pChangeSemaphore = new omni_semaphore(0);
    
    }
    
    /**
     * Start the thread running. After this is called events can be
     * queued.
     */
    virtual void start() {
      m_bRun = true;
      start_undetached();
    }
    
    
    /**
     * Stop the thread running. After this is called events can no
     * longer be queued.
     */
    virtual void stop() {
      m_bRun = false;
      m_pChangeSemaphore->post();
    }

    virtual bool isRunning() {
      return m_bRun;
    }


    void queue(QueuedData & _data) {
      m_queueAccess.lock();
      m_queue.push(_data);
      m_queueAccess.unlock();
      
      m_pChangeSemaphore->post();
      
    }

    /**
     * Helper method for cleaning up threads
     *
     */
    
    //    template <class QueuedData>
    static void cleanup(QueuedDataThread<QueuedData> * _thread) {

      if(_thread) {      
	_thread->stop();

	int* rv;
	try {
	  _thread->join((void**)&rv);
	} 
	catch (const omni_thread_invalid& e) {
	  std::cerr<<" THREAD INVALID... was it started?"<<std::endl;
	}
      }
      
    }

  protected:


    /**
     * Destructor. Hidden to prevent the thread being destroyed.
     */
    virtual ~QueuedDataThread() {
      while(!m_queue.empty()) {
	m_queue.pop();	  
      }
      m_pChangeSemaphore->post();
      delete m_pChangeSemaphore;
    }

    virtual void nextInQueue(QueuedData & _data) = 0;


    /**
     * Run method of thread. 
     */
    virtual void* run_undetached(void *arg) {

      bool listEmpty = true;
      QueuedData data;

      while(isRunning()) {

	m_queueAccess.lock();
	listEmpty = m_queue.empty();
	m_queueAccess.unlock();
	
	while(!listEmpty) {

	  m_queueAccess.lock();  
	  data = m_queue.front();
	  m_queue.pop();
	  m_queueAccess.unlock();

	  nextInQueue(data);	  
	  
	  m_queueAccess.lock();
	  listEmpty = m_queue.empty();
	  m_queueAccess.unlock();

	}
        
	m_pChangeSemaphore->wait();
	
      }
 
      //necessary return bits
      int* rv = new int(0);
      return (void*)rv;
    }

    /**
     * The list of change structs ready to be written to the component.
     */
    std::queue<QueuedData> m_queue;
  
    ///Controls access to m_pChangeList
    omni_mutex m_queueAccess;

    ///Whether the thread should do anthing
    bool m_bRun;

    omni_semaphore * m_pChangeSemaphore;
  };

}

#endif
