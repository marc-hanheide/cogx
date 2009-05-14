/*
 * CAST - The CoSy Architecture Schema Toolkit Copyright (C) 2006-2007
 * Nick Hawes This library is free software; you can redistribute it
 * and/or modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either version
 * 2.1 of the License, or (at your option) any later version. This
 * library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. You should have
 * received a copy of the GNU Lesser General Public License along with
 * this library; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * 
 */
package cast.architecture.abstr;

import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.Vector;

import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeFilter;

/**
 * Map to store change filters against their objects. Updated 14-8-07 to store
 * lists of receivers against filters, rather than single receivers.
 * 
 * @author nah
 */
public class WorkingMemoryChangeFilterMap<T> {

	/**
	 * Data store.
	 */
	private final SortedMap<WorkingMemoryChangeFilter, Vector<T>> m_map;

	/**
	 * Whether all filters are local.
	 */
	private boolean m_localOnly;

	/**
	 * 
	 */
	public WorkingMemoryChangeFilterMap() {
		// m_map = Collections.synchronizedSortedMap(new
		// TreeMap<WorkingMemoryChangeFilter, Vector<T>>(
		// WorkingMemoryChangeFilterComparator.getComparator()));
		m_map = new TreeMap<WorkingMemoryChangeFilter, Vector<T>>(
				WorkingMemoryChangeFilterComparator.getComparator());
		m_localOnly = true;
	}

	/**
	 * 
	 */
	private final void updateFilters() {
		m_localOnly = true;
		for (WorkingMemoryChangeFilter filter : m_map.keySet()) {
			if (filter.m_restriction != FilterRestriction.LOCAL_SA) {
				m_localOnly = false;
				return;
			}
		}
	}

	public synchronized boolean allowsChange(WorkingMemoryChange _wmc) {
		for (WorkingMemoryChangeFilter filter : m_map.keySet()) {
			if (WorkingMemoryChangeFilterComparator.allowsChange(filter, _wmc)) {
				return true;
			}
		}
		return false;
	}

	/**
	 * Determines whether the filter set contains only local filters on whether
	 * it needs xarch changes too.
	 * 
	 * @return True if all filters match on local changes only.
	 */
	public boolean localFiltersOnly() {
		return m_localOnly;
	}

	/**
	 * @return
	 * 
	 */
	public synchronized Set<WorkingMemoryChangeFilter> filterSet() {
		return m_map.keySet();
	}

	/**
	 * Find all the change receivers that will accept the given change.
	 * 
	 * @param _wmc
	 * @return
	 */
	public synchronized void get(WorkingMemoryChange _wmc, Queue<T> _receivers) {

		// change for bug #35

		for (WorkingMemoryChangeFilter filter : m_map.keySet()) {
			if (WorkingMemoryChangeFilterComparator.allowsChange(filter, _wmc)) {
				Vector<T> receiverList = m_map.get(filter);
				if (receiverList == null) {
					// throw new RuntimeException(
					// "Alignment error in change filters. allowsChange reports
					// that the change allows the filter, but compareTo did not
					// return the filter.");
				}
				else if (receiverList.isEmpty()) {
					// throw new RuntimeException(
					// "Alignment error in change filters. allowsChange reports
					// that the change allows the filter, but filter list is
					// empty.");
				}
				else {
					// return receiverList;
					_receivers.addAll(receiverList);
				}
			}
			// System.out.println("receiver list size: " + _receivers.size());
		}
	}

	public synchronized Vector<T> get(WorkingMemoryChangeFilter _key) {
		return m_map.get(_key);
	}

	/**
	 * @param _key
	 * @param _value
	 * @return Previous list associated with specified key, or null if there was
	 *         no mapping for key. A null return can also indicate that the map
	 *         previously associated null with the specified key.
	 */
	public synchronized Vector<T> put(WorkingMemoryChangeFilter _key, T _value) {

		// get existing list of receivers for key
		Vector<T> existingList = get(_key);

		// if there is no existing list, create an empty one
		if (existingList == null) {
			existingList = new Vector<T>(1);
		}

		// add new receiver to empty list
		existingList.add(_value);

		// update local status
		m_localOnly = m_localOnly
				&& (_key.m_restriction == FilterRestriction.LOCAL_SA);

		// and add the list to the map
		return m_map.put(_key, existingList);
	}

	public synchronized Vector<T> remove(WorkingMemoryChangeFilter _key) {
		Vector<T> removed = m_map.remove(_key);
		updateFilters();
		return removed;
	}

	/**
	 * 
	 * Remove receiver from all filters.
	 * 
	 * @param _receiver
	 * @return
	 */
	public synchronized void remove(T _receiver,
			List<WorkingMemoryChangeFilter> _removed) {

		for (WorkingMemoryChangeFilter key : m_map.keySet()) {

			Vector<T> receivers = m_map.get(key);

			while (receivers.remove(_receiver)) {
			}

			if (receivers.isEmpty()) {
				_removed.add(key);
			}

		}

		// if the removal has caused a change, update local flag
		if (!_removed.isEmpty()) {

			for (WorkingMemoryChangeFilter filter : _removed) {
				m_map.remove(filter);
			}

			updateFilters();
		}
	}

	public synchronized int size() {
		int size = 0;
		for (WorkingMemoryChangeFilter key : m_map.keySet()) {
			size += m_map.get(key).size();
		}
		return size;
	}

}
