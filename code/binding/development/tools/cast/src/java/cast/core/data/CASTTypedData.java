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
 * General classes useful for representing data.
 */
package cast.core.data;

/**
 * A simple class for wrapping up an object with an ontological type.
 * 
 * @author nah
 */
public class CASTTypedData<T> {

	// the stored object...
	private final T m_data;

	// /the ontological type of the data
	private final String m_type;

	/**
	 * The version number associated with this data. This equals the number of
	 * times it was overwritten in working memory.
	 */
	private int m_version;

	/**
	 * Construct a new object with an ontological type and a data object.
	 * 
	 * @param _type
	 *            The ontological type of the data.
	 * @param _data
	 *            The data itself.
	 */
	public CASTTypedData(String _type, T _data) {
		this(_type, 0, _data);
	}

	/**
	 * Construct a new object with an ontological type and a data object.
	 * 
	 * @param _type
	 *            The ontological type of the data.
	 * @param _data
	 *            The data itself.
	 */
	public CASTTypedData(String _type, int _version, T _data) {
		m_type = _type;
		m_version = _version;
		m_data = _data;
	}

	/**
	 * Determine with this object equals the input object. Compares the equality
	 * of the ontological types then the stored objects.
	 * 
	 * @param _obj
	 *            The input object for comparison.
	 * @return True if the objects are equals, else false.
	 */
	@Override
	public boolean equals(Object _obj) {
		if (_obj instanceof CASTTypedData) {
			CASTTypedData<?> ctd = (CASTTypedData<?>) _obj;
			return ctd.m_type.equals(m_type) && ctd.m_version == m_version
					&& ctd.m_data.equals(m_data);
		} else {
			return false;
		}
	}

	/**
	 * Get the data object.
	 * 
	 * @return The data object.
	 */
	public final T getData() {
		return m_data;
	}

	/**
	 * Get the ontological type of the data object.
	 * 
	 * @return The type of the data.
	 */
	public final String getType() {
		return m_type;
	}

	/**
	 * // * Set the data object. // * // *
	 * 
	 * @param _data // *
	 *            The new data object. //
	 */
	// public final void setData(T _data) {
	// m_data = _data;
	// }
	//
	// /**
	// * Set the ontological type of the data.
	// *
	// * @param _type
	// * The new ontological type.
	// */
	// public final void setType(String _type) {
	// m_type = _type;
	// }
	/**
	 * Returns a formatted representation of the object.
	 * 
	 * @return The object as in string format.
	 */
	@Override
	public String toString() {
		return "[" + m_type + "[" + m_data + "]";
	}

	public int getVersion() {
		return m_version;
	}

	public void setVersion(int _versionNumber) {
		m_version = _versionNumber;
	}
}
