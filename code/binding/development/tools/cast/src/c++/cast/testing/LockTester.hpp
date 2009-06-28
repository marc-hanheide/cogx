/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Michael Zillich, Nick Hawes
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

#ifndef LOCK_TESTER_HPP
#define LOCK_TESTER_HPP

#include <cast/testing/AbstractTester.hpp>

namespace cast {

  class LockTester : public AbstractTester {
  
  public:
    
    class LockingWriter : public AbstractTest,
			  public WorkingMemoryChangeReceiver {
    public:
      LockingWriter(AbstractTester & _tester, const int & _count) : 
	AbstractTest(_tester),
	m_count(_count){};
      
      virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc);
    protected:
      virtual void startTest();
    private:
      int m_count;
    };
    
    class LockingOverwriter : public AbstractTest,
			      public WorkingMemoryChangeReceiver {
    public:
      LockingOverwriter(AbstractTester & _tester, const int & _count) : 
	AbstractTest(_tester),
	m_count(_count)
      {};

      virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc);
    protected:
      virtual void startTest();
    private:
      int m_count;
    };
    
    class TryLockingOverwriter : public AbstractTest,
				 public WorkingMemoryChangeReceiver {
    public:
      TryLockingOverwriter(AbstractTester & _tester, const int & _count) : 
	AbstractTest(_tester),
	m_count(_count)
      {};

      virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc);
    protected:
      virtual void startTest();
    private:
      int m_count;
    };


    class Locker : public AbstractTest {
    public:
      Locker(AbstractTester & _tester, const cdl::WorkingMemoryPermissions & _permissions) : 
	AbstractTest(_tester),
	m_permissions(_permissions){};
    protected:
      virtual void startTest();
    private:
      cdl::WorkingMemoryPermissions m_permissions;
    };

    class Sneaker : public AbstractTest,
		    public WorkingMemoryChangeReceiver {
    public:
      Sneaker(AbstractTester & _tester, const cdl::WorkingMemoryOperation & _op) : 
	AbstractTest(_tester),
	m_op(_op)
      {};
      
      virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc);
    protected:
      virtual void startTest();
    private:
      cdl::WorkingMemoryOperation m_op;
    };


  protected:

    virtual void configure(const std::map<std::string,std::string> & _config);
    
    /**
     * Subarchitecture used as target for testing operations. Defaults
     * to own subarch.
     */
    std::string m_targetSubarch;

  private:
    friend class LockingWriter;
    friend class LockingOverwriter;
    friend class TryLockingOverwriter;

  };

} //namespace cast

#endif
