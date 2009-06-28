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

/**
 * 
 */
package cast.core;

import java.util.ArrayList;
import java.util.Collection;

import cast.cdl.WorkingMemoryEntry;

/**
 * An inteface used to define how other objects can access the data
 * items stored in a working memory object. This is for use by objects
 * that actually wrap such an object, not for components that are
 * connected to the working memory in a subarchitecture.
 * 
 * @author nah
 */
public interface CASTWorkingMemoryInterface {

    /**
     * Get a collection of memory items with the given ontological type.
     * 
     * @param _type
     *            The type to check.
     * @param _count
     *            The number of entries to return. If 0 all matching
     *            entries are returned.
     * @return A collection of matching entries.
     */
    public abstract Collection<WorkingMemoryEntry> getByType(
            String _type, int _count);

    /**
     * Get all working memory entries with given type.
     * 
     * @param _type
     *            The type to check.
     * @return All matching items from working memory.
     */
    public abstract Collection<WorkingMemoryEntry> getByType(
            String _type);

    /**
     * Adds item to working memory with given id. Does not overwrite
     * data if id already exists.
     * 
     * @param _id
     *            The id of the entry.
     * @param _type
     *            The ontological type of the entry.
     * @param _data
     *            The data itself.
     * @return Returns true if the item is added (i.e. not a duplicate
     *         id)
     */
    public abstract boolean add(String _id, WorkingMemoryEntry _data);

    /**
     * Overwrites item with given id. Does not do anything if id does
     * not exist.
     * 
     * @param _id
     *            The id of the entry.
     * @param _type
     *            The ontological type of the entry.
     * @param _data
     *            The data itself.
     * @return Returns true if the id exists for overwriting
     */
    public abstract boolean overwrite(String _id,
            WorkingMemoryEntry _data);

    /**
     * Removes the item with the given id.
     * 
     * @param _id
     *            The id of the entry.
     * @return Returns true if the id is removed.
     */
    public abstract WorkingMemoryEntry remove(String _id);

    /**
     * Get the item with the given id.
     * 
     * @param _id
     *            The specified item or null if it does not exist.
     */
    public abstract WorkingMemoryEntry get(String _id);

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
    public abstract ArrayList<String> getIDsByType(String _type,
            int _count);

    
    /**
     * Get the current size of the working memory.
     * 
     * @return The size of working memory.
     */
    public abstract int size();

    /**
     * Whether the wm currently contains an entry with the given id.
     * 
     * @param _queryID
     * @return
     */
    public abstract boolean contains(String _queryID);

    /**
     * Whether the wm has ever contained an entry with the given id.
     * 
     * @param _queryID
     * @return
     */
    public abstract boolean hasContained(String _queryID);

    
    /**
     * @param _queryID
     * @return
     */
    public abstract int getOverwriteCount(String _queryID);

}