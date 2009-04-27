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
package cast.architecture;

import java.util.Comparator;

import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeFilter;
import cast.cdl.WorkingMemoryOperation;

public class WorkingMemoryChangeFilterComparator implements
		Comparator<WorkingMemoryChangeFilter> {

	private static WorkingMemoryChangeFilterComparator comparator;

	public static WorkingMemoryChangeFilterComparator getComparator() {
		if (comparator == null) {
			comparator = new WorkingMemoryChangeFilterComparator();
		}
		return comparator;
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
		int comparison = _f1.address.id.compareTo(_f2.address.id);
		if (comparison != EQUAL) {
			return comparison;
		}

		// change sa
		comparison = _f1.address.subarchitecture
				.compareTo(_f2.address.subarchitecture);
		if (comparison != EQUAL) {
			return comparison;
		}

		// ontological type
		comparison = _f1.type.compareTo(_f2.type);
		if (comparison != EQUAL) {
			return comparison;
		}

		// operation
		if (_f1.operation.value() < _f2.operation.value()) {
			return BEFORE;
		}
		if (_f1.operation.value() > _f2.operation.value()) {
			return AFTER;
		}


		// xarch
		if ((_f1.restriction != FilterRestriction.LOCALSA)
				&& (_f2.restriction == FilterRestriction.LOCALSA)) {
			return BEFORE;
		}
		if ((_f1.restriction == FilterRestriction.LOCALSA)
				&& (_f2.restriction != FilterRestriction.LOCALSA)) {
			return AFTER;
		}

		// source of change
		comparison = _f1.src.compareTo(_f2.src);
		if (comparison != EQUAL) {
			return comparison;
		}

		// origin of filter
		comparison = _f1.origin.compareTo(_f2.origin);
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
		if (_filter.address.id.length() > 0) {
			// then ids much match
			if (!_filter.address.id.equals(_change.address.id)) {
				// System.out.println("id failed");
				return false;
			}

		}


		// subarchitecture options

		// if filter is set for a specific subarch
		if (_filter.address.subarchitecture.length() > 0) {
			// then subarchitectures must match
			if (!_filter.address.subarchitecture
					.equals(_change.address.subarchitecture)) {
				// System.out.println("sa failed");
				return false;
			}
		} else {
			// otherwises allow any subarchitecture in the change
		}

	

		// type options

		// if type set
		if (_filter.type.length() > 0) {
			// then they must match
			if (!_filter.type.equals(_change.type)) {
				// System.out.println("type failed");
				return false;
			}
		}

		
		// operation must be the same if not a wildcard
		if (_filter.operation != WorkingMemoryOperation.WILDCARD) {
			if (_filter.operation != _change.operation) {
				// System.out.println("operatopm failed");
				return false;
			}
		}
		
		// src options

		// if filter src set
		if (_filter.src.length() > 0) {
			// filter src must match
			if (!_filter.src.equals(_change.src)) {
				// System.out.println("src failed");
				return false;
			}

		}

		// if we get this far then we're fine

		return true;
	}

}