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

#ifndef CAST_WORKING_MEMORY_CHANGE_RECEIVER_H_
#define CAST_WORKING_MEMORY_CHANGE_RECEIVER_H_

#include <cast/slice/CDL.hpp>

namespace cast {

  class WorkingMemoryChangeReceiver {


  public:
    WorkingMemoryChangeReceiver() : m_deleteOnRemoval(false) {};
    virtual ~WorkingMemoryChangeReceiver() {};
    virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) = 0;
    void deleteOnRemoval() const {m_deleteOnRemoval = true;};
    bool isDeletedOnRemoval() const {return m_deleteOnRemoval;};

  private:
    mutable bool m_deleteOnRemoval;
  };


#define CALL_MEMBER_FN(ptrToObject,ptrToMember)  ((ptrToObject)->*(ptrToMember))

  template <class T>
  class MemberFunctionChangeReceiver: 
    public WorkingMemoryChangeReceiver {
 
  public:

    typedef void (T::*CreatorMemberFunction)(const cdl::WorkingMemoryChange & _wmc);
  
    MemberFunctionChangeReceiver(T *_pCreator, CreatorMemberFunction _func) {
      m_pCreator = _pCreator;
      m_func = _func;
    }

    virtual ~MemberFunctionChangeReceiver(){};

    virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {
      CALL_MEMBER_FN(m_pCreator,m_func)(_wmc);
    }

  protected:

    T * m_pCreator;
    CreatorMemberFunction m_func;

  };

} //namespace cast

#endif
