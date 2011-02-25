package manipulation.itemMemory;

import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Observable;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.MathException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Region;
import manipulation.core.share.types.Vector2D;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.ViewPoint;
import manipulation.core.share.types.ViewPoints;
import manipulation.core.share.types.VisionModel;
import manipulation.itemMemory.Item.ItemIntention;
import manipulation.itemMemory.Item.ItemStatus;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.math.MathOperation;

import org.apache.log4j.Logger;

/**
 * represents an item memory
 * 
 * @author ttoenige
 * 
 */
public class ItemMemory extends Observable {
	private Logger logger = Logger.getLogger(this.getClass());
	private LinkedList<Item> itemList;
	private LinkedList<Item> deletedItemList;

	/**
	 * different status to of the view point goal reaching behavior of the robot
	 * 
	 * @author ttoenige
	 * 
	 */
	public enum ReachingStatus {
		/**
		 * robot is on the way to the viewpoint
		 */
		ON_THE_WAY,
		/**
		 * the viewpoint is not reachable
		 */
		VP_NOT_REACHABLE,
		/**
		 * the robot reached the viewpoint
		 */
		VP_REACHED,
		/**
		 * something else happend
		 */
		OTHER
	}

	private ReachingStatus viewPointReachingStatus;

	/**
	 * constructor of the item memory
	 */
	public ItemMemory() {
		itemList = new LinkedList<Item>();
		deletedItemList = new LinkedList<Item>();
		viewPointReachingStatus = ReachingStatus.OTHER;
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
			if (it.next().equals(item)) {
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
		Vector3D oldPos = null;
		try {
			oldPos = (Vector3D) item.getAttribute(PropertyName.WORLD_POSITION);
		} catch (ItemException e2) {
			logger.error(e2);
		}

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

			Vector3D itemInWorldPosition = manipulator.getBaseConnector()
					.getRobotToWorldTranslation(itemInRobPosition);

			Matrix itemInWorldRotation = manipulator.getBaseConnector()
					.getRobotToWorldRotation(itemInRobRotation);

			newItem.setAttribute(PropertyName.WORLD_POSITION,
					itemInWorldPosition);
			newItem.setAttribute(PropertyName.WORLD_ROTATION,
					itemInWorldRotation);
		} else {
			newItem.setAttribute(PropertyName.WORLD_POSITION, newPositionData);
			newItem.setAttribute(PropertyName.WORLD_ROTATION, newRotationData);
		}

		try {
			if ((MathOperation.getDistance(newPositionData, oldPos) > 1000)) {
				logger.error("ERRRRR "
						+ MathOperation.getDistance(newPositionData, oldPos));
				Region potentionalSurrounding = manipulator.getMapAlgorithms()
						.updateSurroundingFromWorldCoordinates(newItem);

				newItem.setAttribute(PropertyName.SURROUNDING,
						potentionalSurrounding);

				ViewPoints vps = new ViewPoints(manipulator.getMapAlgorithms()
						.generatePotentialViewPoints(item));

				newItem.setAttribute(PropertyName.VIEW_POINTS, vps);

				newItem.setAttribute(PropertyName.BEST_VIEW_POINT, manipulator
						.getMapAlgorithms().getBestViewPoint(vps.getPoints()));
			}

			newItem.setAttribute(PropertyName.LAST_STATUS, ItemStatus.UPDATED);
			// TODO anders mit den fehlerabfang machen, avoid_me als eigene
			// klasse
		} catch (ItemException e) {
			// logger.error(e);
			// TODO nichts machen? zB wenn surrounding nicht da ist
		} catch (MathException e) {
			throw new InternalMemoryException(
					"Cannot update surrounding of Item " + item);
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

	public void updateSurrounding(Item item, Region region)
			throws InternalMemoryException {
		boolean success = false;
		Item newItem = new Item(item);

		newItem.setAttribute(PropertyName.SURROUNDING, region);

		ListIterator<Item> it = itemList.listIterator(0);
		while (it.hasNext()) {
			if (it.next().equals(item)) {
				itemList.set(it.previousIndex(), newItem);
				success = true;
				break;
			}
		}

		if (success) {
			setChanged();
			notifyObservers(region);
		} else {
			throw new InternalMemoryException(
					"Cannot find the item in the memory");
		}
	}

	public void updateViewPointErrorByDistance(Item item,
			Manipulator manipulator) throws ItemException,
			InternalMemoryException {
		boolean success = false;
		Item newItem = new Item(item);

		List<ViewPoint> viewPoints = ((ViewPoints) item
				.getAttribute(PropertyName.VIEW_POINTS)).getPoints();

		List<ViewPoint> newViewPoints = new LinkedList<ViewPoint>();

		for (ViewPoint viewPoint : viewPoints) {

			double error = MathOperation.getDistance(viewPoint.getPosition()
					.getPoint(), ((Vector3D) item
					.getAttribute(PropertyName.WORLD_POSITION))
					.forgetThirdDimension());

			ViewPoint newViewPoint = new ViewPoint(error, viewPoint
					.getPosition());

			newViewPoints.add(newViewPoint);
		}

		newItem.setAttribute(PropertyName.VIEW_POINTS, new ViewPoints(
				newViewPoints));

		newItem.setAttribute(PropertyName.BEST_VIEW_POINT, manipulator
				.getMapAlgorithms().getBestViewPoint(newViewPoints));

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
			notifyObservers(viewPoints);
		} else {
			throw new InternalMemoryException(
					"Cannot find the item in the memory");
		}
	}

	public void updateViewPointErrorByAngle(Item item, Manipulator manipulator)
			throws ItemException, InternalMemoryException {
		boolean success = false;
		Item newItem = new Item(item);

		List<ViewPoint> viewPoints = ((ViewPoints) item
				.getAttribute(PropertyName.VIEW_POINTS)).getPoints();

		List<ViewPoint> newViewPoints = new LinkedList<ViewPoint>();

		Matrix objectRotationInRob = (Matrix) item
				.getAttribute(PropertyName.ITEM_IN_ROB_ROTATION);

		Vector3D objectPositionInRob = (Vector3D) item
				.getAttribute(PropertyName.ITEM_IN_ROB_POSITION);

		Vector3D bestDirection = MathOperation.getMatrixVectorMultiplication(
				objectRotationInRob, objectPositionInRob);

		for (ViewPoint viewPoint : viewPoints) {

			Vector2D currentDirecion = MathOperation.getDirection(viewPoint
					.getPosition().getPoint(), ((Vector3D) item
					.getAttribute(PropertyName.WORLD_POSITION))
					.forgetThirdDimension());

			double error = MathOperation.getDistance(bestDirection
					.forgetThirdDimension(), currentDirecion);

			ViewPoint newViewPoint = new ViewPoint(error, viewPoint
					.getPosition());

			newViewPoints.add(newViewPoint);
		}

		newItem.setAttribute(PropertyName.VIEW_POINTS, new ViewPoints(
				newViewPoints));

		newItem.setAttribute(PropertyName.BEST_VIEW_POINT, manipulator
				.getMapAlgorithms().getBestViewPoint(newViewPoints));

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
			notifyObservers(viewPoints);
		} else {
			throw new InternalMemoryException(
					"Cannot find the item in the memory");
		}

	}

	public void updateViewPoints(Item item, ViewPoints viewPoints,
			ViewPoint bestVP, ViewPoints bestRotVPs)
			throws InternalMemoryException {
		boolean success = false;
		Item newItem = new Item(item);

		newItem.setAttribute(PropertyName.VIEW_POINTS, viewPoints);

		newItem.setAttribute(PropertyName.BEST_VIEW_POINT, bestVP);

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
			notifyObservers(viewPoints);
		} else {
			throw new InternalMemoryException(
					"Cannot find the item in the memory");
		}
	}

	public void removeViewPoint(Item item, ViewPoint viewPoint,
			Manipulator manipulator) throws ItemException,
			InternalMemoryException {
		logger.error("Remove VP");

		Item newItem = new Item(item);

		ViewPoints viewpointsStruct = ((ViewPoints) item
				.getAttribute(PropertyName.VIEW_POINTS));

		List<ViewPoint> viewPoints = viewpointsStruct.getPoints();

		List<ViewPoint> returnList = new LinkedList<ViewPoint>();

		int size = viewPoints.size();
		for (int i = 0; i < size; i++) {

			ViewPoint point = viewPoints.get(i);

			if (MathOperation.getDistance(viewPoints.get(i).getPosition()
					.getPoint(), viewPoint.getPosition().getPoint()) > manipulator
					.getConfiguration().getNearPoints()) {
				returnList.add(point);
			}
		}

		viewpointsStruct = new ViewPoints(returnList);

		newItem.setAttribute(PropertyName.VIEW_POINTS, viewpointsStruct);

		newItem.setAttribute(PropertyName.BEST_VIEW_POINT, manipulator
				.getMapAlgorithms().getBestViewPoint(returnList));

		boolean success = false;

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
			notifyObservers(viewPoints);
		} else {
			throw new InternalMemoryException(
					"Cannot find the item in the memory");
		}

	}

	public void removeRotationalViewPoint(Item item, ViewPoint viewPoint,
			Manipulator manipulator) throws ItemException,
			InternalMemoryException {
		logger.error("Remove Rot-VP");

		Item newItem = new Item(item);

		ViewPoints viewpointsStruct = ((ViewPoints) item
				.getAttribute(PropertyName.ROTATIONAL_VIEWPOINT));

		List<ViewPoint> viewPoints = viewpointsStruct.getPoints();

		List<ViewPoint> returnList = new LinkedList<ViewPoint>();

		int size = viewPoints.size();
		for (int i = 0; i < size; i++) {

			ViewPoint point = viewPoints.get(i);

			if (MathOperation.getDistance(viewPoints.get(i).getPosition()
					.getPoint(), viewPoint.getPosition().getPoint()) > manipulator
					.getConfiguration().getNearPoints()) {
				returnList.add(point);
			}
		}

		viewpointsStruct = new ViewPoints(returnList);

		newItem.setAttribute(PropertyName.ROTATIONAL_VIEWPOINT,
				viewpointsStruct);

		newItem.setAttribute(PropertyName.BEST_ROTATIONAL_VIEWPOINT,
				manipulator.getMapAlgorithms().getBestViewPoint(returnList));

		boolean success = false;

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
			notifyObservers(viewPoints);
		} else {
			throw new InternalMemoryException(
					"Cannot find the item in the memory");
		}

	}

	/**
	 * updates the reaching status of the viewpoint
	 * 
	 * @param status
	 *            new status
	 * @throws InternalMemoryException
	 */
	public void updateViewPointReachingStatus(ReachingStatus status)
			throws InternalMemoryException {
		this.viewPointReachingStatus = status;
		setChanged();
		notifyObservers(status);
	}

	public enum UpdateAction {
		UPDATE, GENERATE
	}

	public void updateRotationAwareViewPoints(Item item,
			Manipulator manipulator, UpdateAction ua)
			throws InternalMemoryException, ItemException, MathException, ExternalMemoryException {

		List<ViewPoint> rotationalVP = new LinkedList<ViewPoint>();
		Item newItem = new Item(item);

		switch (ua) {
		case GENERATE:
			rotationalVP = manipulator.getMapAlgorithms()
					.generateRotationalViewPoints(item, manipulator.getBaseConnector().getCurrentPosition());

			ViewPoints rotationalVPstuct = new ViewPoints(rotationalVP);

			newItem.setAttribute(PropertyName.ROTATIONAL_VIEWPOINT,
					rotationalVPstuct);

			break;
		case UPDATE:
			List<ViewPoint> bestRotVP = manipulator.getMapAlgorithms()
					.updateRotationViewPoints(item);

			ViewPoints besRotVPStruc = new ViewPoints(bestRotVP);

			newItem.setAttribute(PropertyName.ROTATIONAL_VIEWPOINT,
					besRotVPStruc);

			break;
		default:
			break;
		}

		double smallestError = Double.MAX_VALUE;
		ViewPoint bestRotationalVP = null;
		for (ViewPoint viewPoint : rotationalVP) {
			if (viewPoint.getError() < smallestError) {
				bestRotationalVP = viewPoint;
				smallestError = viewPoint.getError();
			}
		}

		newItem.setAttribute(PropertyName.BEST_ROTATIONAL_VIEWPOINT,
				bestRotationalVP);

		boolean success = false;

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
			// notifyObservers(viewPoints);
		} else {
			throw new InternalMemoryException(
					"Cannot find the item in the memory");
		}

	}

	public void deleteRotationalVP(Item item, ViewPoint vpToDelete)
			throws InternalMemoryException, ItemException {

		Item newItem = new Item(item);

		ViewPoints oldRotViewpoints = (ViewPoints) item
				.getAttribute(PropertyName.ROTATIONAL_VIEWPOINT);

		List<ViewPoint> newRotVPList = new LinkedList<ViewPoint>();

		for (ViewPoint viewPoint : oldRotViewpoints.getPoints()) {
			if (viewPoint.getPosition().getPoint().equals(
					vpToDelete.getPosition().getPoint())) {
				logger.error("delete rot vp");
			} else {
				newRotVPList.add(viewPoint);
			}
		}

		newItem.setAttribute(PropertyName.ROTATIONAL_VIEWPOINT, new ViewPoints(
				newRotVPList));

		double smallestError = Double.MAX_VALUE;
		ViewPoint bestRotationalVP = null;
		for (ViewPoint viewPoint : newRotVPList) {
			if (viewPoint.getError() < smallestError) {
				bestRotationalVP = viewPoint;
				smallestError = viewPoint.getError();
			}
		}

		newItem.setAttribute(PropertyName.BEST_ROTATIONAL_VIEWPOINT,
				bestRotationalVP);

		boolean success = false;

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
			// notifyObservers(viewPoints);
		} else {
			throw new InternalMemoryException(
					"Cannot find the item in the memory");
		}

	}

	/**
	 * gets the first item of the memory to grasp
	 * 
	 * @return first item to grasp
	 * @throws InternalMemoryException
	 */
	public Item getFirstGraspItem() throws InternalMemoryException {
		if (itemList.size() == 0)
			throw new InternalMemoryException("No items in the memory");

		for (int i = 0; i < itemList.size(); i++) {
			try {
				if (((ItemIntention) itemList.get(i).getAttribute(
						PropertyName.INTENTION)) == ItemIntention.GRASP_ME) {
					return itemList.get(i);
				}
			} catch (ItemException e) {
				logger.error(e);
			}
		}

		throw new InternalMemoryException("No grasp items available");
	}

	/**
	 * gets all items of the memory
	 * 
	 * @return all items of the memory
	 */
	public LinkedList<Item> getItemList() {
		return itemList;
	}

	/**
	 * gets the reaching status of the current viewpoint
	 * 
	 * @return reaching status of the current viewpoint of the current item
	 */
	public ReachingStatus getViewPointReachingStatus() {
		return this.viewPointReachingStatus;
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
