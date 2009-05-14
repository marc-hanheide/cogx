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

import java.util.*;

/**
 * Class that actually operates as a working memory. May eventually implement
 * this in C++ and import via the JNI to ease cross-language development.
 * 
 * @author nah
 */
public class CASTWorkingMemory implements CASTWorkingMemoryInterface {

	// the ontology for the data in working memory. Can be null.
	// private CAATOntology m_ontology;

	// the underlying data storage... a linkedhashmap keeps track of the
	// ordering of the keys so that we can get contents by recency
	private final LinkedHashMap<String, CASTWorkingMemoryItem> m_storage;

	private final HashMap<String,Integer> m_lastVersions;
	
	/**
	 * Create a new working memory object.
	 */
	public CASTWorkingMemory() {
		m_storage = new LinkedHashMap<String, CASTWorkingMemoryItem>(20);
		m_lastVersions = new HashMap<String, Integer>(20);
	}

	/**
	 * Get an array of memory items with the given type.
	 * 
	 * @param _type
	 *            The type to check.
	 * @param _count
	 *            The number of entries to return. If 0 all matching entries are
	 *            returned.
	 * @return An array of matching entries.
	 */
	public CASTWorkingMemoryItem[] getArrayByType(String _type, int _count) {
		Collection<CASTWorkingMemoryItem> temp = getByType(_type, _count);
		CASTWorkingMemoryItem[] wmel = new CASTWorkingMemoryItem[temp.size()];
		temp.toArray(wmel);
		return wmel;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.core.data.WorkingMemoryInterface#getCollectionByType(java.lang.String,
	 *      int)
	 */
	public Collection<CASTWorkingMemoryItem> getByType(String _type, int _count) {

		Collection<CASTWorkingMemoryItem> temp = new ArrayList<CASTWorkingMemoryItem>();
		int count = 0;

		// need to go backwards through the keys, which is pain!
		Set<String> keys = m_storage.keySet();
		String[] keyArray = new String[keys.size()];
		keys.toArray(keyArray);

		CASTWorkingMemoryItem item;

		for (int i = keyArray.length - 1; i >= 0; i--) {
			item = m_storage.get(keyArray[i]);
			if (item.getType().equals(_type)) {
				temp.add(item);
				count++;
			}
			if (count == _count) {
				break;
			}
		}
		return temp;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.core.data.WorkingMemoryInterface#getCollectionByType(java.lang.String)
	 */
	public Collection<CASTWorkingMemoryItem> getByType(String _type) {
		return getByType(_type, m_storage.size());
	}

	/**
	 * Returns a formatted representation of the object. If an ontology is set
	 * then the wm attempts to use this to format the contents of memory.
	 * 
	 * @return The object as in string format.
	 */
	@Override
	public String toString() {
		String out = "[\n";

		Set<String> keys = m_storage.keySet();
		CASTWorkingMemoryItem item;
		// if (m_ontology == null) {
		for (String id : keys) {
			item = m_storage.get(id);
			// can't do this is final version as data types are
			// unknown!
			out += id + ": [" + item.getType() + " ["
					+ "<set ontology for printing>" + "]]\n";
		}
		// }
		// else {
		// for (String id : keys) {
		// item = m_storage.get(id);
		// try {
		// // can't do this is final version as data types are
		// // unknown!
		// out += id
		// + ": ["
		// + item.getType()
		// + " ["
		// + RemoteDataTranslator.translateFromAny(item
		// .getData(), m_ontology
		// .ontologicalTypeToDataType(item.getType()))
		// + "]]\n";
		// }
		// catch (FrameworkDataTranslatorException e) {
		// System.err.println(e.getLocalizedMessage());
		// }
		// catch (CAATOntologyException e) {
		// System.err.println(e.getLocalizedMessage());
		// }
		// }
		// }
		out += "\n]";

		return out;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.core.data.WorkingMemoryInterface#add(java.lang.String,
	 *      java.lang.String, org.omg.CORBA.Any)
	 */
	public boolean add(String _id, CASTWorkingMemoryItem _data) {

		assert _data.getVersion() == 0 : " verion not 0: " + _data.getVersion();

		if (!m_storage.containsKey(_id)) {
			// System.out.println("adding at: " + _id);
			
			if(m_lastVersions.containsKey(_id)) {
//				System.out.println("CASTWorkingMemory.add(): reusing version");
				_data.setVersion(m_lastVersions.get(_id) + 1);
				m_lastVersions.remove(_id);
			}
			
			m_storage.put(_id, _data);
			return true;
		} else {
			return false;
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.core.data.WorkingMemoryInterface#overwrite(java.lang.String,
	 *      java.lang.String, org.omg.CORBA.Any)
	 */
	public boolean overwrite(String _id, CASTWorkingMemoryItem _data) {
		if (m_storage.containsKey(_id)) {
			// increment overwrite count
			_data.setVersion(m_storage.get(_id).getVersion() + 1);
			m_storage.put(_id, _data);
			return true;
		} else {
			System.err.println("data NOT overwritten");
			return false;
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.core.data.WorkingMemoryInterface#remove(java.lang.String)
	 */
	public CASTWorkingMemoryItem remove(String _id) {
		// System.out.println("looing for : " + _id);
		// System.out.println(this);
		CASTWorkingMemoryItem item = m_storage.remove(_id);
		if (item == null) {
			return null;
		} else {
			//store version
			m_lastVersions.put(_id, item.getVersion());
			return item;
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.core.data.WorkingMemoryInterface#get(java.lang.String)
	 */
	public CASTWorkingMemoryItem get(String _id) {
		return m_storage.get(_id);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.core.data.WorkingMemoryInterface#getIDsOfType(java.lang.String,
	 *      int)
	 */
	public ArrayList<String> getIDsByType(String _type, int _count) {

		if (_count == 0) {
			_count = m_storage.size();
		}

		ArrayList<String> ids = new ArrayList<String>();
		Set<String> keys = m_storage.keySet();
		String[] keyArray = new String[keys.size()];
		keys.toArray(keyArray);

		CASTWorkingMemoryItem item;

		int count = 0;

		for (int i = keyArray.length - 1; i >= 0; i--) {
			item = m_storage.get(keyArray[i]);
			if (item.getType().equals(_type)) {
				ids.add(keyArray[i]);
				count++;
			}
			if (count == _count) {
				break;
			}
		}
		return ids;
	}

	//
	// /*
	// * (non-Javadoc)
	// *
	// * @see caat.core.data.WorkingMemoryInterface#getOntology()
	// */
	// public CAATOntology getOntology() {
	// return m_ontology;
	// }
	//
	// /*
	// * (non-Javadoc)
	// *
	// * @see
	// caat.core.data.WorkingMemoryInterface#setOntology(caat.core.ontologies.CAATOntology)
	// */
	// public void setOntology(CAATOntology _ontology) {
	// m_ontology = _ontology;
	// }

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.core.data.CAATWorkingMemoryInterface#size()
	 */
	public int size() {
		return m_storage.size();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.core.data.CAATWorkingMemoryInterface#contains(java.lang.String)
	 */
	public boolean contains(String _queryID) {
		return m_storage.containsKey(_queryID);
	}

	/**
	 * * Whether the wm has ever contained an entry with the given id.
	 * 
	 * @param _queryID
	 * @return
	 */
	public boolean hasContained(String _queryID) {
		if(contains(_queryID)) {
			return true;
		}
		return m_lastVersions.containsKey(_queryID);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.data.CASTWorkingMemoryInterface#getOverwriteCount(java.lang.String)
	 */
	public int getOverwriteCount(String _queryID) {
		CASTWorkingMemoryItem item = m_storage.get(_queryID);
		if(item != null) {
			return item.getVersion();
		}
		
		Integer lastCount = m_lastVersions.get(_queryID);
		if(lastCount != null) {
			return lastCount;
		}
		
		return -1;
		
		
	}
}
