/*
 s* CAST - The CoSy Architecture Schema Toolkit
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

#ifndef CAST_CAST_WORKING_MEMORY_H_
#define CAST_CAST_WORKING_MEMORY_H_

//for testing purposed... comment out to remove sync access to wm
#define SYNC_MEMORY_ACCESS


#include <cast/core/CASTWorkingMemoryInterface.hpp>

#include <cast/core/StringMap.hpp>
#include <cast/slice/CDL.hpp>

#include <map> 
#include <list>


namespace cast {

  typedef StringMap<cdl::WorkingMemoryEntryPtr >::map WMItemMap;
  typedef StringMap<int>::map VersionMap;
  typedef std::list < std::string > StringList;

  class CASTWorkingMemory: public CASTWorkingMemoryInterface {

  public:


    /**
     * Default constructor.
     */
    CASTWorkingMemory();

    /**
     * Empty virtual destructor.
     */
    virtual ~CASTWorkingMemory();

    /**
     * Get a collection of memory items with the given type.
     * 
     * @param _type
     *            The type to check.
     * @param _count
     *            The number of entries to return. If 0 all matching
     *            entries are returned.
     * @return A collection of matching entries.
     */
    virtual void getByType(const std::string & _type, 
			   const int & _count,
			   std::vector< cdl::WorkingMemoryEntryPtr > & _items);

    /**
     * Get allthis working memory entries with given type.
     * 
     * @param _type
     *            The type to check.
     * @return All matching items from working memory.
     */
    virtual void getByType(const std::string & _type,
			   std::vector< cdl::WorkingMemoryEntryPtr > & _items); 

    /**
     * Adds item to working memory with given id. Does not overwrite
     * data if id already exists.
     * 
     * @param _id
     *            The id of the entry.
     * @param _pData
     *            The data and type info. A shared pointer so that memory is managed for us
     * @return Returns true if the item is added (i.e. not a duplicate
     *         id)
     */
    virtual bool add(const std::string & _id, 
		     cdl::WorkingMemoryEntryPtr _pData);

    /**
     * Overwrites item with given id. Does not do anything if id does
     * not exist.
     * 
     * @param _id
     *            The id of the entry.
     * @param _pEntry
     *            The data and type info.
     * @return Returns true if the id exists for overwriting
     */
    virtual bool overwrite(const std::string &  _id, 
			   cdl::WorkingMemoryEntryPtr _pData);

    /**
     * Removes the item with the given id.
     * 
     * @param _id
     *            The id of the entry.
     * @return Returns true if the id is removed.
     */
    virtual cdl::WorkingMemoryEntryPtr remove(const std::string &  _id);

    /**
     * Get the item with the given id.
     * 
     * @param _id
     *            The specified item or null if it does not exist.
     */
    //virtual const CASTWorkingMemoryItem * get(const std::string &  _id);
    virtual cdl::WorkingMemoryEntryPtr get(const std::string &  _id);

    /**
     * See where an item with the given id exists in wm.
     * 
     * @param _id
     *            The specified item or null if it does not exist.
     */
    virtual bool contains(const std::string &  _id);

    /**
     * See if an item with the given id was previously on wm.
     * 
     * @param _id
     *            The specified item.
     */
    virtual bool hasContained(const std::string &  _id);

    virtual int getOverwriteCount(const std::string &  _id);


    /**
     * Gets the n most recent ids of entries with the given type.
     * 
     * @param _type
     *            The type to check.
     * @param _count
     *            The number of ids to return. If 0 all matching items
     *            are return.
     * @return All matching items.
     */
    virtual void getIDsByType(const std::string & _type,
			      const int & _count,
			      std::vector<std::string> &_ids);

    virtual int size() {return m_storage.size();}

    virtual void debug();

  protected:

    //need a hash_map for quick lookups
    WMItemMap m_storage;
    //and a vector to maintain orderings
    StringList m_ids;
    //a map containing the last version numbers of removed items
    VersionMap m_lastVersions;


#ifdef SYNC_MEMORY_ACCESS
    pthread_mutex_t m_sync;	   
#endif

    void lock() throw (CASTException);
    void unlock() throw (CASTException);

  };

} // namespace cast

#endif
