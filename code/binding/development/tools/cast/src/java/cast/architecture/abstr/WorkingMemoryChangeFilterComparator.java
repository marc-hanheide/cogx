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
package cast.architecture.abstr;

import java.util.Comparator;

import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeFilter;
import cast.cdl.WorkingMemoryOperation;

public class WorkingMemoryChangeFilterComparator implements
		Comparator<WorkingMemoryChangeFilter> {

	private static WorkingMemoryChangeFilterComparator m_comparator;

	public static WorkingMemoryChangeFilterComparator getComparator() {
		if (m_comparator == null) {
			m_comparator = new WorkingMemoryChangeFilterComparator();
		}
		return m_comparator;
	}

	/**
	 * 
	 */
	private WorkingMemoryChangeFilterComparator() {
		// must use get method
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Comparator#compare(java.lang.Object, java.lang.Object)
	 */
	public int compare(WorkingMemoryChangeFilter _f1,
			WorkingMemoryChangeFilter _f2) {

		// //System.out
		// .println("WorkingMemoryChangeFilterComparator.compare()");
		// //System.out.println(CAATUtils.toString(_f1));
		// //System.out.println(CAATUtils.toString(_f2));

		final int BEFORE = -1;
		final int EQUAL = 0;
		final int AFTER = 1;

		// same object
		if (_f1 == _f2) {
			return EQUAL;
		}

		
		// change id
		int comparison = _f1.m_address.m_id.compareTo(_f2.m_address.m_id);
		if (comparison != EQUAL) {
			return comparison;
		}

		// change sa
		comparison = _f1.m_address.m_subarchitecture
				.compareTo(_f2.m_address.m_subarchitecture);
		if (comparison != EQUAL) {
			return comparison;
		}

		// ontological type
		comparison = _f1.m_type.compareTo(_f2.m_type);
		if (comparison != EQUAL) {
			return comparison;
		}

		// operation
		if (_f1.m_operation.value() < _f2.m_operation.value()) {
			return BEFORE;
		}
		if (_f1.m_operation.value() > _f2.m_operation.value()) {
			return AFTER;
		}


		// xarch
		if ((_f1.m_restriction != FilterRestriction.LOCAL_SA)
				&& (_f2.m_restriction == FilterRestriction.LOCAL_SA)) {
			return BEFORE;
		}
		if ((_f1.m_restriction == FilterRestriction.LOCAL_SA)
				&& (_f2.m_restriction != FilterRestriction.LOCAL_SA)) {
			return AFTER;
		}

		// source of change
		comparison = _f1.m_src.compareTo(_f2.m_src);
		if (comparison != EQUAL) {
			return comparison;
		}

		// origin of filter
		comparison = _f1.m_origin.compareTo(_f2.m_origin);
		if (comparison != EQUAL) {
			return comparison;
		}

		
		// all comparisons have yielded equality
		// verify that compareTo is consistent with equals
		// (optional)
		// assert _o1.equals(_o2) : "compare inconsistent with
		// equals.";

		return EQUAL;
	}

	public static final boolean allowsChange(WorkingMemoryChangeFilter _filter,
			WorkingMemoryChange _change) {

		// System.out.println(CASTUtils.toString(_filter));
		// System.out.println(CASTUtils.toString(_change));

		// id options

		// if the filter id is not empty
		if (_filter.m_address.m_id.length() > 0) {
			// then ids much match
			if (!_filter.m_address.m_id.equals(_change.m_address.m_id)) {
				// System.out.println("id failed");
				return false;
			}

		}


		// subarchitecture options

		// if filter is set for a specific subarch
		if (_filter.m_address.m_subarchitecture.length() > 0) {
			// then subarchitectures must match
			if (!_filter.m_address.m_subarchitecture
					.equals(_change.m_address.m_subarchitecture)) {
				// System.out.println("sa failed");
				return false;
			}
		} else {
			// otherwises allow any subarchitecture in the change
		}

	

		// type options

		// if type set
		if (_filter.m_type.length() > 0) {
			// then they must match
			if (!_filter.m_type.equals(_change.m_type)) {
				// System.out.println("type failed");
				return false;
			}
		}

		
		// operation must be the same if not a wildcard
		if (_filter.m_operation != WorkingMemoryOperation.WILDCARD) {
			if (_filter.m_operation != _change.m_operation) {
				// System.out.println("operatopm failed");
				return false;
			}
		}
		
		// src options

		// if filter src set
		if (_filter.m_src.length() > 0) {
			// filter src must match
			if (!_filter.m_src.equals(_change.m_src)) {
				// System.out.println("src failed");
				return false;
			}

		}

		// if we get this far then we're fine

		return true;
	}

}