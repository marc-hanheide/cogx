package manipulation.itemMemory;

import java.util.HashMap;
import java.util.Set;
import java.util.UUID;

import manipulation.core.share.exceptions.ItemException;

/**
 * represents an item
 * 
 * @author Torben Toeniges
 * 
 */
public class Item {
	/**
	 * represents the status of the item in the memory
	 * 
	 * @author Torben Toeniges
	 * 
	 */
	public enum ItemStatus {
		/**
		 * item was updated
		 */
		UPDATED,
		/**
		 * item was added
		 * 
		 */
		ADDED,
		/**
		 * item was deleted
		 */
		DELETED
	}

	/**
	 * represents the intention of the item
	 * 
	 * @author Torben Toeniges
	 * 
	 */
	public enum ItemIntention {
		/**
		 * item to grasp
		 */
		GRASP_ME,
		/**
		 * item to avoid
		 */
		AVOID_ME
	}

	/**
	 * represents the status of the properties of the item
	 * 
	 * @author Torben Toeniges
	 * 
	 */
	public enum PropertyStatus {
		/**
		 * a new property
		 */
		NEW,
		/**
		 * an old property was updated
		 */
		UPDATE
	}

	/**
	 * represents the name of a property
	 * 
	 * @author Torben Toeniges
	 * 
	 */
	public enum PropertyName {
		/**
		 * position of the item in camera coordinates
		 */
		ITEM_IN_CAM_POSITION,
		/**
		 * rotation of the item in camera coordinates
		 */
		ITEM_IN_CAM_ROTATION,
		/**
		 * position of the item in robot coordinates
		 */
		ITEM_IN_ROB_POSITION,
		/**
		 * rotation of the item in robot coordinates
		 */
		ITEM_IN_ROB_ROTATION,
		/**
		 * last status of the item
		 */
		LAST_STATUS,
		/**
		 * vision model of the item
		 */
		MODEL,
		/**
		 * position of the item in world coordinates
		 */
		WORLD_POSITION,
		/**
		 * rotation of the item in world coordinates
		 */
		WORLD_ROTATION,
		/**
		 * intention of the item
		 */
		INTENTION,
		/**
		 * name of the item
		 */
		NAME, BLORT_NAME, WMA_ADDRESS
	}

	private HashMap<PropertyName, Object> properties = new HashMap<PropertyName, Object>();
	private UUID ID = null;

	/**
	 * constructor of an item
	 */
	public Item() {
		ID = UUID.randomUUID();
	}

	/**
	 * copy constructor of an item
	 * 
	 * @param item
	 *            to copy
	 */
	public Item(Item item) {
		this.properties = item.getProperties();
		ID = item.getID();

	}

	/**
	 * gets the unique id
	 * 
	 * @return unique item id
	 */
	public UUID getID() {
		return ID;
	}

	/**
	 * sets an attribute/property of the item
	 * 
	 * @param key
	 *            attribute key
	 * @param property
	 *            attribute value
	 * @return status of the added attribute
	 */
	public PropertyStatus setAttribute(PropertyName key, Object property) {
		PropertyStatus status;

		if (properties.containsKey(key)) {
			status = PropertyStatus.UPDATE;
		} else {
			status = PropertyStatus.NEW;
		}
		properties.put(key, property);
		return status;
	}

	/**
	 * gets an attribute / property of the item
	 * 
	 * @param key
	 *            key to specify the attribute to return
	 * @return attribute which is asked for
	 * @throws ItemException
	 */
	public Object getAttribute(PropertyName key) throws ItemException {
		if (properties.containsKey(key)) {
			return properties.get(key);
		} else
			throw new ItemException("Item properties do  not contain " + key
					+ ".");
	}

	/**
	 * gets all properties / attributes of the item
	 * 
	 * @return all properties of an item
	 */
	public HashMap<PropertyName, Object> getProperties() {
		return properties;
	}

	/**
	 * gets all keys of the attributes / properties of the item
	 * 
	 * @return all keys of the current item
	 */
	public Set<PropertyName> getAllAtributeKeys() {
		return properties.keySet();
	}

}
