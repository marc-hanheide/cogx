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
package cast.architecture.subarchitecture;

import cast.cdl.WorkingMemoryAddress;

/**
 * Exception that is thrown when an extry is not present on wm.
 * 
 * @author nah
 */
public class DoesNotExistOnWMException extends WMException {

	/**
	 * 
	 */
	private static final long serialVersionUID = 5438630015613929429L;

	public DoesNotExistOnWMException(WorkingMemoryAddress _address, String _message, Throwable _cause) {
		super(_address, _message, _cause);
	}

	public DoesNotExistOnWMException(WorkingMemoryAddress _address, String _message) {
		super(_address, _message);
	}

	public DoesNotExistOnWMException(WorkingMemoryAddress _address, Throwable _cause) {
		super(_address, _cause);
	}

	public DoesNotExistOnWMException(WorkingMemoryAddress _address) {
		super(_address);
	}

	
}
