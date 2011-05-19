package manipulation.itemMemory;

import java.util.LinkedList;
import java.util.ListIterator;
import java.util.Observable;

import manipulation.core.cogx.converter.CogXConverter;
import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Pose;
import manipulation.core.share.types.Vector2D;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.VisionModel;
import manipulation.itemMemory.Item.ItemStatus;
import manipulation.itemMemory.Item.PropertyName;
import mathlib.Functions;

import org.apache.log4j.Logger;

import cogx.Math.Pose3;
import cogx.Math.Vector3;

/**
 * represents an item memory
 * 
 * @author Torben Toeniges
 * 
 */
public class ItemMemory extends Observable {
	private Logger logger = Logger.getLogger(this.getClass());
	private LinkedList<Item> itemList;
	private LinkedList<Item> deletedItemList;

	/**
	 * constructor of the item memory
	 */
	public ItemMemory() {
		itemList = new LinkedList<Item>();
		deletedItemList = new LinkedList<Item>();
	}

	/**
	 * adds the relevant item to the queue of the other items
	 * 
	 * @param item
	 *            corresponding item
	 */
	public void addItemToQueue(Item item) {
		item.setAttribute(PropertyName.LAST_STATUS, ItemStatus.ADDED);
		itemList.addLast(item);
		setChanged();
		notifyObservers(item);
	}

	/**
	 * adds a visual model to an item
	 * 
	 * @param item
	 *            relevant item
	 * @param model
	 *            new model to add
	 * @throws InternalMemoryException
	 */
	public void addItemModel(Item item, VisionModel model)
			throws InternalMemoryException {
		boolean success = false;
		Item newItem = new Item(item);

		newItem.setAttribute(PropertyName.MODEL, model);
		
		ListIterator<Item> it = itemList.listIterator(0);
		while (it.hasNext()) {
			if (it.next().getID().equals(item.getID())) {
				itemList.set(it.previousIndex(), newItem);
				success = true;
				break;
			}
		}

		if (success) {
			setChanged();
			// TODO was soll ich den Observern sagen?
			notifyObservers(model);
		} else {
			throw new InternalMemoryException(
					"Cannot find the item in the memory");
		}

	}

	/**
	 * deletes an item from the memory
	 * 
	 * @param item
	 *            item to delete
	 * @throws InternalMemoryException
	 */
	public void deleteItem(Item item) throws InternalMemoryException {

		if (itemList.remove(item)) {
			item.setAttribute(PropertyName.LAST_STATUS, ItemStatus.DELETED);
			deletedItemList.add(item);
			setChanged();
			notifyObservers(item);
		} else {
			throw new InternalMemoryException("Cannot delete the item");
		}
	}

	/**
	 * updates the data of an item
	 * 
	 * @param item
	 *            relevant item
	 * @param newPositionData
	 *            new position data of the item
	 * @param newRotationData
	 *            new rotation data of the item
	 * @param manipulator
	 *            corresponding manipulator
	 * @param worldCoordinates
	 *            <code>true</code> if the data are worldcoordinates,
	 *            <code>false</code> if the data are cameracoordinates
	 * @throws InternalMemoryException
	 */
	public void updatePosition(Item item, Vector3D newPositionData,
			Matrix newRotationData, Manipulator manipulator,
			boolean worldCoordinates) throws InternalMemoryException {
		boolean success = false;
		Item newItem = new Item(item);

		if (!worldCoordinates) {
			newItem.setAttribute(PropertyName.ITEM_IN_CAM_POSITION,
					newPositionData);
			newItem.setAttribute(PropertyName.ITEM_IN_CAM_ROTATION,
					newRotationData);

			Vector3D itemInRobPosition = manipulator.getCalibrationConnector()
					.getCamPointInRob(newPositionData);

			Matrix itemInRobRotation = manipulator.getCalibrationConnector()
					.getCamRotationInRob(newRotationData);

			newItem.setAttribute(PropertyName.ITEM_IN_ROB_POSITION,
					itemInRobPosition);
			newItem.setAttribute(PropertyName.ITEM_IN_ROB_ROTATION,
					itemInRobRotation);

			Pose3 robPose = CogXConverter.convertToPose3(itemInRobPosition,
					itemInRobRotation);

			try {
				Vector2D position = manipulator.getBaseConnector()
						.getCurrentPosition().getPoint();

				Pose3 tRobotToWorld = Functions.pose3FromEuler(new Vector3(
						position.getX(), position.getY(), 0), 0.0, 0.0,
						manipulator.getBaseConnector().getCurrentPosition()
								.getAngle());

				Pose3 pInWorld = Functions.transform(tRobotToWorld, robPose);

				Pose pInWorldPos = CogXConverter.convertPose3ToPose(pInWorld);

				newItem.setAttribute(PropertyName.WORLD_POSITION,
						pInWorldPos.getTranslation());
				newItem.setAttribute(PropertyName.WORLD_ROTATION,
						pInWorldPos.getRotation());

			} catch (ExternalMemoryException e) {
				logger.error(e);
			}

		} else {
			newItem.setAttribute(PropertyName.WORLD_POSITION, newPositionData);
			newItem.setAttribute(PropertyName.WORLD_ROTATION, newRotationData);
		}

		ListIterator<Item> it = itemList.listIterator(0);
		while (it.hasNext()) {
			if (it.next().getID().equals(item.getID())) {
				itemList.set(it.previousIndex(), newItem);
				success = true;
				break;
			}
		}

		if (success) {
			setChanged();
			try {
				notifyObservers((Vector3D) newItem
						.getAttribute(PropertyName.WORLD_POSITION));
			} catch (ItemException e) {
				logger.error(e);
			}
		} else {
			throw new InternalMemoryException(
					"Cannot find the item in the memory");
		}
	}

	/**
	 * gets all items of the memory
	 * 
	 * @return all items of the memory
	 */
	public LinkedList<Item> getItemList() {
		return itemList;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		String returnString = "[ ";
		ListIterator<Item> it = itemList.listIterator(0);

		while (it.hasNext()) {
			returnString += it.next().toString();
		}

		returnString = returnString + "]";

		return returnString;
	}

}
