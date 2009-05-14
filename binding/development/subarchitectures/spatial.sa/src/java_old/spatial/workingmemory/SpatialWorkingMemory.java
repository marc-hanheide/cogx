/**
 * 
 */
package spatial.workingmemory;

import java.awt.Color;
import java.util.ArrayList;

import org.cognitivesystems.common.autogen.Math.Pose3D;

import spatial.autogen.SpatialLocation;
import spatial.autogen.SpatialRelationship;
import spatial.autogen.SpatialScene;
import spatial.ontology.SpatialOntologyFactory;
import spatial.util.SpatialDrawingUtils;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.architecture.subarchitecture.SubarchitectureWorkingMemory;
import cast.cdl.guitypes.Point2D;
import cast.core.CASTUtils;
import cast.core.data.CASTWorkingMemoryItem;

/**
 * @author nah
 */
public class SpatialWorkingMemory extends SubarchitectureWorkingMemory {

	private SpatialScene m_currentScene;

	/**
	 * @param _id
	 */
	public SpatialWorkingMemory(String _id) {
		super(_id);

		// determines whether this wm should broadcast to oher
		// sub-architectures
		setSendXarchChangeNotifications(true);

	}

	@Override
	protected boolean addToWorkingMemory(String _id, CASTWorkingMemoryItem _data) {
		boolean result = super.addToWorkingMemory(_id, _data);
		// log("Working memory contains [" + m_workingMemory.size()
		// + "] items");
		// log("Forwarding: " + isForwardingXarchChangeNotifications());

		try {

			// if a new scene is added, render it to the gui
			if (result
					&& _data.getType().equals(
							CASTUtils.typeName(SpatialScene.class))) {
				m_currentScene = getItem(_id, SpatialScene.class);
				redrawGraphicsNow();
			}
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			System.exit(1);
		}

		return result;
	}

	/**
	 * @param _scene
	 * @throws SubarchitectureProcessException
	 */
	private void drawScene(SpatialScene _scene)
			throws SubarchitectureProcessException {

		drawWorkspace();
		drawViewer(_scene.m_viewerPose);

//		// draw objects
//		ArrayList<SpatialLocation> objects = getW(_scene.m_locations,
//				SpatialLocation.class);
//		for (SpatialLocation object : objects) {
//			drawObject(object);
//		}
//
//		// draw each spatial relation
//		for (SpatialRelationship rel : _scene.m_leftRelationships) {
//			drawLeftRelationship(
//					getItem(rel.m_landmark, SpatialLocation.class), getItem(
//							rel.m_target, SpatialLocation.class), rel.m_value);
//		}
//		for (SpatialRelationship rel : _scene.m_rightRelationships) {
//			drawRightRelationship(
//					getItem(rel.m_landmark, SpatialLocation.class), getItem(
//							rel.m_target, SpatialLocation.class), rel.m_value);
//		}
//		for (SpatialRelationship rel : _scene.m_frontRelationships) {
//			drawFrontRelationship(
//					getItem(rel.m_landmark, SpatialLocation.class), getItem(
//							rel.m_target, SpatialLocation.class), rel.m_value);
//		}
//		for (SpatialRelationship rel : _scene.m_backRelationships) {
//			drawBackRelationship(
//					getItem(rel.m_landmark, SpatialLocation.class), getItem(
//							rel.m_target, SpatialLocation.class), rel.m_value);
//		}
//		for (SpatialRelationship rel : _scene.m_proximityRelationships) {
//			drawProxRelationship(
//					getItem(rel.m_landmark, SpatialLocation.class), getItem(
//							rel.m_target, SpatialLocation.class), rel.m_value);
//		}
	}

	/**
	 * @param _item
	 * @param _item2
	 * @param _value
	 * @param _light_gray
	 */
	private void drawFrontRelationship(SpatialLocation _landmark,
			SpatialLocation _target, float _value) {

		Point2D landmarkTR = SpatialDrawingUtils.topRight(_landmark.m_centroid);
		Point2D targetTR = SpatialDrawingUtils.topRight(_target.m_centroid);

		// proximal rels are rendered bottom left to bottom left
		SpatialDrawingUtils.drawLine(this, (float) landmarkTR.m_x,
				(float) landmarkTR.m_y, (float) targetTR.m_x,
				(float) targetTR.m_y, _value, Color.GREEN);

	}

	/**
	 * @param _item
	 * @param _item2
	 * @param _value
	 * @param _light_gray
	 */
	private void drawBackRelationship(SpatialLocation _landmark,
			SpatialLocation _target, float _value) {

		Point2D landmarkTL = SpatialDrawingUtils.topLeft(_landmark.m_centroid);
		Point2D targetTL = SpatialDrawingUtils.topLeft(_target.m_centroid);

		// proximal rels are rendered bottom left to bottom left
		SpatialDrawingUtils.drawLine(this, (float) landmarkTL.m_x,
				(float) landmarkTL.m_y, (float) targetTL.m_x,
				(float) targetTL.m_y, _value, Color.BLUE);

	}

	/**
	 * @param _landmark
	 * @param _target
	 * @param _value
	 * @param _light_gray
	 */
	private void drawLeftRelationship(SpatialLocation _landmark,
			SpatialLocation _target, float _value) {

		Point2D landmarkBL = SpatialDrawingUtils
				.bottomLeft(_landmark.m_centroid);
		Point2D targetBL = SpatialDrawingUtils.bottomLeft(_target.m_centroid);

		// proximal rels are rendered bottom left to bottom left
		SpatialDrawingUtils.drawLine(this, (float) landmarkBL.m_x,
				(float) landmarkBL.m_y, (float) targetBL.m_x,
				(float) targetBL.m_y, _value, Color.LIGHT_GRAY);
	}

	/**
	 * @param _item
	 * @param _item2
	 * @param _value
	 * @param _light_gray
	 */
	private void drawRightRelationship(SpatialLocation _landmark,
			SpatialLocation _target, float _value) {

		Point2D landmarkBR = SpatialDrawingUtils
				.bottomRight(_landmark.m_centroid);
		Point2D targetBR = SpatialDrawingUtils.bottomRight(_target.m_centroid);

		// proximal rels are rendered bottom left to bottom left
		SpatialDrawingUtils.drawLine(this, (float) landmarkBR.m_x,
				(float) landmarkBR.m_y, (float) targetBR.m_x,
				(float) targetBR.m_y, _value, Color.RED);
	}

	/**
	 * @param _landmark
	 * @param _target
	 * @param _value
	 * @param _light_gray
	 */
	private void drawProxRelationship(SpatialLocation _landmark,
			SpatialLocation _target, float _value) {

		// proximal rels are rendered centre to centre
		SpatialDrawingUtils.drawLine(this, _landmark.m_centroid.m_x,
				_landmark.m_centroid.m_y, _target.m_centroid.m_x,
				_target.m_centroid.m_y, _value, Color.PINK);

	}

	/**
	 * @param _object
	 */
	private void drawObject(SpatialLocation _object) {
		SpatialDrawingUtils.drawBBox(this, _object.m_centroid, Color.yellow);
	}

	/**
	 * 
	 */
	private void drawWorkspace() {
		// just draw the origin for the time being
		SpatialDrawingUtils.drawPoint(this, 0f, 0f, Color.YELLOW);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.components.InspectableComponent#redrawGraphics2D()
	 */
	@Override
	protected void redrawGraphics2D() {

		if (m_currentScene == null) {
			return;
		}

		try {
			drawScene(m_currentScene);
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}

	}

	/**
	 * @param _viewerPose
	 */
	private void drawViewer(Pose3D _viewerPose) {
		SpatialDrawingUtils.drawPose(this, _viewerPose, Color.GREEN);
	}

	@Override
	protected CASTWorkingMemoryItem<?> deleteFromWorkingMemory(String _id,
			String _component) {
		CASTWorkingMemoryItem<?> result = super.deleteFromWorkingMemory(_id,
				_component);
		return result;
	}

	//
	@Override
	protected boolean overwriteWorkingMemory(String _id,
			CASTWorkingMemoryItem<?> _data, String _component) {
		boolean result = super.overwriteWorkingMemory(_id, _data, _component);
		// if a new scene is added, render it to the gui
		if (result
				&& _data.getType().equals(
						CASTUtils.typeName(SpatialScene.class))) {
			redrawGraphicsNow();
		}
		return result;
	}
}
