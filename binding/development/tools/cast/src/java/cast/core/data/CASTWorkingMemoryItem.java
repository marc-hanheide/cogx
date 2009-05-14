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
package cast.core.data;

import org.omg.CORBA.Any;

import balt.corba.data.RemoteDataTranslator;
import balt.corba.data.translation.FrameworkDataTranslatorException;

/**
 * Specialised typed object data type to store CORBA Anys. The Any type is
 * currently used as the internal representation for the working memory object.
 * 
 * @see org.omg.CORBA.Any
 * @author nah
 */
public class CASTWorkingMemoryItem<T> extends CASTTypedData<T> {

	/**
	 * Construct a new object with an ontological type and a data object.
	 * 
	 * @param _type
	 *            The ontological type of the data.
	 * @param _data
	 *            The data itself.
	 */
	public CASTWorkingMemoryItem(String _type, T _data) {
		this(_type, 0, _data);
	}

	public CASTWorkingMemoryItem(String _type, int _version, T _data) {
		super(_type, _version, _data);
	}

	public boolean isLocal() {
		if (Any.class.isAssignableFrom(getData().getClass())) {
			return false;
		} else {
			return true;
		}
	}

	/**
	 * Determine with this object equals the input object.
	 * 
	 * @param _obj
	 *            The input object for comparison.
	 */
	@Override
	public boolean equals(Object _obj) {
		if (_obj instanceof CASTWorkingMemoryItem) {
			CASTWorkingMemoryItem wmi = (CASTWorkingMemoryItem) _obj;
			return wmi.getType().equals(getType())
					&& wmi.getData().equals(getData());
		}
		return false;
	}

	/**
	 * @return
	 * @throws FrameworkDataTranslatorException
	 */
	public Any toAny() throws FrameworkDataTranslatorException {
		if (Any.class.isAssignableFrom(getData().getClass())) {
			return (Any) getData();
		} else {
			return RemoteDataTranslator.translateToAny(getData());
		}
	}

}
