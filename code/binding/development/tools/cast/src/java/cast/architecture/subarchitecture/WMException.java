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
 * Exception that is thrown when incorrectly altering working memory contents.
 * 
 * @author nah
 */
public class WMException extends SubarchitectureProcessException {

	private final WorkingMemoryAddress m_address;
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 5438630015613929429L;

	/**
	 * 
	 */
	public WMException(WorkingMemoryAddress _address) {
		this(_address,"");
	}

	public WorkingMemoryAddress address() {
		return m_address;
	}
	
	/**
	 * @param _message
	 */
	public WMException(WorkingMemoryAddress _address, String _message) {
		this(_address,_message,null);
	}

	/**
	 * @param _message
	 * @param _cause
	 */
	public WMException(WorkingMemoryAddress _address, String _message, Throwable _cause) {
		super(_message, _cause);
		m_address = _address;
	}

	/**
	 * @param _cause
	 */
	public WMException( WorkingMemoryAddress _address, Throwable _cause) {
		this (_address, "", _cause);
	}

	

}
