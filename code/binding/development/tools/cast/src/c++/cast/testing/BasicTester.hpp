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
 * Merchantability or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef BASIC_TESTER_HPP
#define BASIC_TESTER_HPP

#include <cast/testing/AbstractTester.hpp>

namespace cast {

  class BasicTester : public AbstractTester {
  
  public:

    class SingleComponentReadWriteTest : public AbstractTest {
    public:
      SingleComponentReadWriteTest(AbstractTester & _tester) : 
	AbstractTest(_tester){};
      
    protected:
      virtual void startTest();
    };

    class Receiver : public AbstractTest {
    public:
      Receiver(AbstractTester & _tester) : 
	AbstractTest(_tester){};
      
      void removeChangeFilter(WorkingMemoryChangeReceiver * _pReceiver,
			      const cdl::ReceiverDeleteCondition & _condition = cdl::DONOTDELETERECEIVER) {
	AbstractTest::removeChangeFilter(_pReceiver,_condition);
      }
      
  
    protected:
      virtual void startTest();
    };

    class TwoComponentReadWriteTest : public AbstractTest, 
				      public WorkingMemoryChangeReceiver {
    public:
      TwoComponentReadWriteTest(AbstractTester & _tester) : 
	AbstractTest(_tester){};
      

      virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc);
      
    protected:
      virtual void startTest();
    private:
      cdl::testing::CASTTestStructPtr m_wrote;
    };

    class Copier : public AbstractTest, 
		   public WorkingMemoryChangeReceiver {
    public:
      Copier(AbstractTester & _tester) : 
	AbstractTest(_tester){};
      virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc);
      
    protected:
      virtual void startTest();
    };

    class Replacer : public AbstractTest, 
		     public WorkingMemoryChangeReceiver {
    public:
      Replacer(AbstractTester & _tester) : 
	AbstractTest(_tester){};
      virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc);
      
    protected:
      virtual void startTest();
    };

    class ReAdder : public AbstractTest, 
		    public WorkingMemoryChangeReceiver {
    public:
      ReAdder(AbstractTester & _tester) : 
	AbstractTest(_tester){};
      virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc);
      
    protected:
      virtual void startTest();
    private:
      cdl::testing::CASTTestStructPtr m_wrote;
    };

    class Deleter : public AbstractTest, 
		    public WorkingMemoryChangeReceiver {
    public:
      Deleter(AbstractTester & _tester, int _modulo) : 
	AbstractTest(_tester), m_modulo(_modulo) {};
      virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc);
      
    protected:
      virtual void startTest();
    private:
      int m_modulo;
    };


    class Writer : public AbstractTest {
    public:
      Writer(AbstractTester & _tester, const int & _count) : 
	AbstractTest(_tester),
	m_count(_count){};
    protected:
      virtual void startTest();
    private:
      int m_count;
    };

    class Overwriter : public AbstractTest {
    public:
      Overwriter(AbstractTester & _tester, const int & _count, bool _safe) : 
	AbstractTest(_tester),
	m_count(_count),
	m_safe(_safe)
      {};
    protected:
      virtual void startTest();
    private:
      int m_count;
      bool m_safe;
    };


    
    class Counter : public AbstractTest, 
		    public WorkingMemoryChangeReceiver {
    public:
      Counter(AbstractTester & _tester, const int & _count) : 
	AbstractTest(_tester),
	m_expecting(_count){};
      virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc);
      
    protected:
      virtual void startTest();
    private:
      int m_expecting;
      
      typedef StringMap<int>::map CountMap;
      CountMap m_counts;

    };




  protected:



    virtual void configure(const std::map<std::string,std::string> & _config);

    
    /**
     * Subarchitecture used as target for testing operations. Defaults
     * to own subarch.
     */
    std::string m_targetSubarch;

  private:
    friend class SingleComponentReadWriteTest;
    friend class TwoComponentReadWriteTest;
    friend class Copier;
    friend class Overwriter;
    friend class Replacer;
    friend class Deleter;

  };

} //namespace cast

#endif
