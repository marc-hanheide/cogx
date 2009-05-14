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

/**
 * A simple class for wrapping up an object with an ontological type and an id.
 * 
 * @author nah
 */
public class CASTData<T> extends CASTTypedData<T> {

	// /The id of the data, used to locate it in working memory
	private final String m_id;

	

	/**
	 * Construct a new object with an an id, and ontological type and a data
	 * object.
	 * 
	 * @param _id
	 *            The ID of the object in working memory.
	 * @param _type
	 *            The ontological type of the data.
	 * @param _data
	 *            The data object itself.
	 */
	public CASTData(String _id, String _type, int _versionNumber, T _data) {
		super(_type, _versionNumber, _data);
		m_id = _id;
	}

	/**
	 * Construct a new object with an an id, and ontological type and a data
	 * object.
	 * 
	 * @param _id
	 *            The ID of the object in working memory.
	 * @param _type
	 *            The ontological type of the data.
	 * @param _data
	 *            The data object itself.
	 */
	public CASTData(String _id, String _type, T _data) {
		this(_id, _type, 0, _data);
	}

	/**
	 * Construct from a previously created typed object.
	 * 
	 * @param _id
	 *            The ID of the object in working memory.
	 * @param _data
	 *            A previously constructed typed object.
	 */
	public CASTData(String _id, CASTTypedData<T> _data) {
		this(_id, _data.getType(), _data.getVersion(), _data.getData());
	}

	/**
	 * Get the id of the object.
	 * 
	 * @return The object's id.
	 * 
 	 * @remark Interface change: Capitalisation: getId() -> getID().
	 */
	public final String getID() {
		return m_id;
	}

	// /**
	// * Set the id of the object.
	// *
	// * @param _id
	// * The object's id.
	// */
	// public final void setId(String _id) {
	// m_id = _id;
	// }

	/**
	 * Returns a formatted representation of the object.
	 * 
	 * @return The object as in string format.
	 */
	@Override
	public String toString() {
		return m_id + ": " + super.toString();
	}


}
