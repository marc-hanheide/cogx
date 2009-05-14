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

#ifndef CONSISTENCY_EXCEPTION_H_
#define CONSISTENCY_EXCEPTION_H_

#include <cast/architecture/WMException.hpp>

namespace cast {

 /**
 * An exception that occurs if there is a problem with working memory read/write
 * consistency.
 * 
 * The working memory consistency model in CAST is based the "invalidate on
 * write" consistency model from the distributed shared memory literature.
 * Consistency is only relevant when overwriting a working memory entry,
 * additions and deletions are not effected by consistency. To make a consistent
 * overwrite, the writing component must have read the most recent version of
 * the working memory before overwriting it. This guarantees that no changes
 * will be lost during overwrites. If you attempt to overwrite a working memory
 * entry without having read the most recent version (or having read it at all)
 * you will get a {@link ConsistencyException}. Consistency checks are
 * performed in {@link LocalWorkingMemoryAttachedComponent} and
 * {@link WorkingMemoryAttachedComponent} and are triggered by
 * {@link WorkingMemoryReaderWriterProcess} and {@link PrivilegedManagedProcess}
 * overwriteWorkingMemory calls. Consistency state can be checked with
 * haveLatestVersion in {@link WorkingMemoryAttachedComponent}.
 * 
 */

class ConsistencyException : public WMException {
  std::string _what;

 public:
  ConsistencyException(const cast::cdl::WorkingMemoryAddress& _wma, 
		       const char *file, const char *function, int line,
		       const char *format, ...) throw() ;
  virtual ~ConsistencyException() throw() {}
  virtual const char* what() const throw() {return _what.c_str();}
  void set(const std::string &s) {_what = s;}
};

} //namespace cast

#endif
