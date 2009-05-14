/**
 * 
 */
package visionsa.components;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Properties;
import java.util.Random;
import java.util.StringTokenizer;

import org.cognitivesystems.common.autogen.Math.Pose3D;
import org.cognitivesystems.common.autogen.Math.Vector3D;
import org.cognitivesystems.common.autogen.VisualAttributes.Colour;
import org.cognitivesystems.common.autogen.VisualAttributes.Shape;

import Vision.BBox3D;
import Vision.Camera;
import Vision.IntWithConfidence;
import Vision.ObjectPropertyUpdates;
import Vision.SceneChanged;
import Vision.SceneObject;
import Vision.SceneObjectUpdate;
import Vision.StringWithConfidence;
import Vision.Surface;
import balt.management.ProcessLauncher;
import cast.architecture.subarchitecture.AlreadyExistsOnWMException;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.OperationMode;
import cast.cdl.testing.CAST_TEST_FAIL;
import cast.cdl.testing.CAST_TEST_PASS;
import cast.core.CASTUtils;
import cast.core.data.CASTData;

/**
 * Generates visual scenes for testing components further down the processing
 * chain. This component current supports two types of scene generation: fixed
 * scenes, and configurable scenes. To use previously fixed scenes, use the
 * --scene flag followed by an integer (currently 1 or 2). Otherwise use the
 * --file option and provide a file containing object definitions. An example is
 * included on svn: subarchitectures/vision/config/fake-example.cast using scene
 * subarchitectures/vision/config/scene-1.fv. The file should be structured as
 * follows: #camera position 0.22528142 -1.1281004 0.57014495 #camera
 * orientation -2.0106425 -0.19543846 0.20791425 #object lines # reqd 3d
 * position then optional features = {colour size shape label} + value string &
 * optional confidence 0.1 0.1 0, colour red 0.8, size small, label obj1 0.2
 * 
 * @author nah
 */
public class DummySceneObjectCreator extends ManagedProcess {

	/**
	 * Wrap all this up internally for the time being, because the world
	 * shouldn't see it and it should be replaced later.
	 */
	private static class VisualAttributeMap {

		private static HashMap<String, Integer> m_strint;

		static {
			m_strint = new HashMap<String, Integer>();

			int attribute = 1;

			// as used by visual binding
			m_strint.put("red", 1);
			m_strint.put("green", 2);
			m_strint.put("blue", 3);
			m_strint.put("yellow", 4);

			m_strint.put("triangle", 5);
			m_strint.put("square", 6);
			m_strint.put("circle", 7);

			attribute = 8;
			// unknown really
			m_strint.put("white", attribute++);
			m_strint.put("black", attribute++);
			m_strint.put("big", attribute++);
			m_strint.put("small", attribute++);

		}

		public static int attribute(String _value) {
			Integer i = m_strint.get(_value);

			if (i != null) {
				return i;
			}

			throw new RuntimeException("attribute: " + _value);
		}

		public static boolean isShape(String _shape) {
			throw new RuntimeException("isShape: " + _shape);
		}

		public static boolean isColour(String _colour) {
			throw new RuntimeException();
		}

		// public static boolean isColour(int _colour) {
		// throw new RuntimeException("isColour: " + _colour);
		// }

		// public static boolean isShape(int _shape) {
		// throw new RuntimeException();
		// }
		//
		// public static boolean isSize(int _size) {
		// throw new RuntimeException();
		// }

		public static boolean isSize(String _size) {
			throw new RuntimeException();
		}

	}

	private static final int CAM_NUM = 0;

	private static final char COMMENT_CHAR = '#';

	private static final String COLOUR_TAG = "colour";

	private static final String SIZE_TAG = "size";

	private static final String LABEL_TAG = "label";

	private static final String SHAPE_TAG = "shape";

	private static Random m_rand;

	private CASTData<SceneChanged> m_sceneChanged;

	private int m_scene;

	private File m_sceneFile;

	private Camera m_sceneCamera;

	private ArrayList<SceneObject> m_sceneObjects;

	private boolean m_parallel;

	private boolean m_changeBetween;

	private Hashtable<String, SceneObject> m_id2so;

	private boolean m_addingCamera;

	private boolean m_testMode;

	static {
		m_rand = new Random();
	}

	/**
	 * @param _id
	 */
	public DummySceneObjectCreator(String _id) {
		super(_id);
		m_id2so = new Hashtable<String, SceneObject>();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
	 */
	@Override
	public void configure(Properties _config) {
		super.configure(_config);

		String fileString = _config.getProperty("--file");
		if (fileString != null) {
			m_sceneFile = new File(fileString);

			m_parallel = false;
			String parallel = _config.getProperty("--parallel");
			if (parallel != null) {
				m_parallel = Boolean.parseBoolean(parallel);
			}

		}
		else {
			String sceneString = _config.getProperty("--scene");
			m_scene = 0;
			if (sceneString != null) {
				m_scene = Integer.parseInt(sceneString);
			}
		}

		String changeString = _config.getProperty("-cb");
		if (changeString != null) {
			m_changeBetween = Boolean.parseBoolean(changeString);
		}
		log("scene change between each object: " + m_changeBetween);

		String cameraString = _config.getProperty("--camera");
		if (cameraString != null) {
			m_addingCamera = Boolean.parseBoolean(cameraString);
		}
		else {
			m_addingCamera = true;
		}
		log("add camera: " + m_addingCamera);

		if (_config.containsKey("--test")) {
			log("in test mode");
			m_testMode = true;
		}
		else {
			m_testMode = false;
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.components.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		try {

			// sleep for a bit
			sleepProcess(10000);

			// file model
			if (m_sceneFile != null) {
				if (!m_sceneFile.exists()) {
					throw new FileNotFoundException(
							"scene file does not exist: " + m_sceneFile);
				}

				parseFile(m_sceneFile);

				//           
				addCamera(m_sceneCamera);

				// addCamera( // this is the camera as it
				// // currently stands on the b21
				// new Camera(CAM_NUM,
				// new Pose3D(new Vector3D(0.22528142f, -1.1281004f,
				// 0.57014495f), new Vector3D(-2.0106425f,
				// -0.19543846f, 0.20791425f)), 0, 0, 0, 0, 0, 0,
				// 0, 0, 0, 0, 0));
				addSceneChanged();

				if (m_parallel) {
					addObjectsInParallel(m_sceneObjects);
				}
				else {
					addObjectsInSerial(m_sceneObjects);
				}

			}
			else {
				addCamera( // this is the camera as it
				// currently stands on the b21
				new Camera(CAM_NUM, new Pose3D(new Vector3D(0.22528142f,
						-1.1281004f, 0.57014495f), new Vector3D(-2.0106425f,
						-0.19543846f, 0.20791425f)), 0, 0, 0, 0, 0, 0, 0, 0, 0,
						0, 0));

				addSceneChanged();

				switch (m_scene) {
					case 1 :
						redObject();
						break;
					case 2 :
						redAndBlueObjects();
						break;
					case 3 :
						sceneForSpaceTest();
						break;
					default :
						sceneForPlanningTest();
						// sceneForPlurals();
						break;
				}
			}

		}
		catch (Exception e) {
			e.printStackTrace();
			if (m_testMode) {
				System.exit(CAST_TEST_FAIL.value);
			}
		}

		if (m_testMode) {
			System.exit(CAST_TEST_PASS.value);
		}

	}

	/**
	 * @param _sceneObjects
	 */
	private void addObjectsInParallel(ArrayList<SceneObject> _sceneObjects) {
		// TODO Auto-generated method stub

	}

	/**
	 * @param _sceneObjects
	 * @throws SubarchitectureProcessException
	 */
	private void addObjectsInSerial(ArrayList<SceneObject> _sceneObjects)
			throws SubarchitectureProcessException {

		if (!m_changeBetween) {
			sceneChanging();
		}

		String id;
		for (SceneObject so : _sceneObjects) {

			if (m_changeBetween) {
				// scene is changing
				sceneChanging();
			}

			// for each object we add an object without it's features
			id = processObject(so.m_bbox.m_centroid.m_x,
					so.m_bbox.m_centroid.m_y, so.m_label.m_string,
					so.m_label.m_confidence, true);

			sleepProcess(3000);
			colourObject(id, so.m_color.m_int, so.m_color.m_confidence);

			sleepProcess(3000);
			shapeObject(id, so.m_shape.m_int, so.m_shape.m_confidence);

			// log("added: " + id + " " + so.m_color.m_int);

			// sleepProcess(200);
			// sizeObject(id, so.m_size.m_int, so.m_size.m_confidence);

			sleepProcess(1000);

			if (m_changeBetween) {
				// scene has changed
				sceneChanged();

				sleepProcess(200);
				// scene has been processed
				sceneProcessed();
			}

		}

		if (!m_changeBetween) {
			// scene has changed
			sceneChanged();

			sleepProcess(100);
			// scene has been processed
			sceneProcessed();
		}

	}

	private void sceneProcessed() throws SubarchitectureProcessException {
		log("scene processed");
		m_sceneChanged.getData().m_sceneChanging = false;
		m_sceneChanged.getData().m_sceneChanged = false;
		m_sceneChanged.getData().m_sceneProcessed = true;
		overwriteWorkingMemory(m_sceneChanged.getID(), m_sceneChanged.getData());
	}

	/**
	 * @param _sceneFile
	 * @throws IOException
	 * @throws SubarchitectureProcessException
	 */
	private void parseFile(File _sceneFile) throws IOException,
			SubarchitectureProcessException {
		// read it all in first, just to be easy
		BufferedReader reader = new BufferedReader(new FileReader(_sceneFile));
		ArrayList<String> lines = new ArrayList<String>();

		String line;
		while (reader.ready()) {
			line = reader.readLine().trim();
			if (line.length() > 0 && (line.charAt(0) != COMMENT_CHAR)) {
				lines.add(line);
			}
		}

		if (lines.size() < 3) {
			throw new SubarchitectureProcessException(
					"not enough lines in file. it should look like:\nx y z\nx y z\nobject desc");
		}

		// now go through doing stuff

		// first line should be camera position
		m_sceneCamera = new Camera(CAM_NUM, new Pose3D(parseVector3D(lines
				.get(0)), parseVector3D(lines.get(1))), 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0);

		log("camera at: " + lines.get(0) + ", " + lines.get(1));

		// the rest should be objects
		String[] tokens;
		StringTokenizer tags;
		float confidence;
		SceneObject so;
		m_sceneObjects = new ArrayList<SceneObject>(lines.size() - 2);
		String tag, value;

		for (int i = 2; i < lines.size(); i++) {
			// split on ','
			tokens = lines.get(i).split(",");

			if (tokens.length == 0) {
				throw new SubarchitectureProcessException(
						"all objects must at least have a position");
			}

			// parse the position
			Vector3D position = parseVector3D(tokens[0]);

			// create an empty scene object
			so = newSceneObject(position.m_x, position.m_y, position.m_z, 0.2f,
					0.2f, 0.2f, "", 0, 0, new String[0]);

			for (int j = 1; j < tokens.length; j++) {
				// now spilt on space
				tags = new StringTokenizer(tokens[j]);
				if (tags.countTokens() < 2) {
					throw new SubarchitectureProcessException(
							"each object property should have a tag, a value and optionally a confidence");
				}

				tag = tags.nextToken();
				value = tags.nextToken();
				confidence = 1f;

				// if 3 long, then a probability is at the end
				if (tags.hasMoreTokens()) {
					Float.parseFloat(tags.nextToken());
				}

				if (tag.equals(COLOUR_TAG)) {
					so.m_color.m_int = VisualAttributeMap.attribute(value);
					so.m_color.m_confidence = confidence;
				}
				else if (tag.equals(SIZE_TAG)) {
					so.m_size.m_int = VisualAttributeMap.attribute(value);
					so.m_size.m_confidence = confidence;
				}
				else if (tag.equals(SHAPE_TAG)) {
					println("parsing shape: " + value + " " + confidence);
					so.m_shape.m_int = VisualAttributeMap.attribute(value);
					so.m_shape.m_confidence = confidence;
				}
				else if (tag.equals(LABEL_TAG)) {
					so.m_label.m_string = value;
					so.m_label.m_confidence = confidence;
				}

			}

			log("parsed object: " + position.m_x + " " + position.m_y + " "
					+ position.m_z);
			m_sceneObjects.add(so);
		}

	}

	/**
	 * Parses a Vector3D from a string that looks like "x y z".
	 * 
	 * @param _string
	 * @return
	 * @throws SubarchitectureProcessException
	 */
	private Vector3D parseVector3D(String _string)
			throws SubarchitectureProcessException {
		StringTokenizer tokenizer = new StringTokenizer(_string);

		if (tokenizer.countTokens() != 3) {
			throw new SubarchitectureProcessException(
					"Pose3D requires 3 floats separated by spaces: \""
							+ _string + "\"");
		}
		return new Vector3D(Float.parseFloat(tokenizer.nextToken()), Float
				.parseFloat(tokenizer.nextToken()), Float.parseFloat(tokenizer
				.nextToken()));

	}

	/**
	 * @throws SubarchitectureProcessException
	 */
	private void addCamera(Camera _cam) throws SubarchitectureProcessException {
		if (m_addingCamera) {
			String data = newDataID();
			// mimic real one
			addToWorkingMemory(data, _cam, OperationMode.BLOCKING);
			overwriteWorkingMemory(data, _cam);
		}
	}

	private void reAddObject(String _id) throws AlreadyExistsOnWMException,
			SubarchitectureProcessException {
		addToWorkingMemory(_id, m_id2so.get(_id));
		updateObject(_id, true, false, false, false, false);
	}

	private void updateObject(String _id, boolean _position, boolean _shape,
			boolean _size, boolean _colour, boolean _class)
			throws AlreadyExistsOnWMException, SubarchitectureProcessException {
		SceneObjectUpdate up = new SceneObjectUpdate(_id,
				new ObjectPropertyUpdates(_position, _shape, _size, _colour,
						_class));
		addToWorkingMemory(newDataID(), up, OperationMode.BLOCKING);
	}

	/**
	 * @throws SubarchitectureProcessException
	 */
	private void addSceneChanged() throws SubarchitectureProcessException {

		SceneChanged sc = new SceneChanged(false, false, false, CAM_NUM);
		m_sceneChanged = new CASTData<SceneChanged>(newDataID(), CASTUtils
				.typeName(sc), sc);
		addToWorkingMemory(m_sceneChanged.getID(), m_sceneChanged.getData(),
				OperationMode.BLOCKING);

		// lock so we can do what we want with it
		// lockEntry(m_sceneChanged.getID(),
		// WorkingMemoryPermissions.LOCKED_OD);
	}

	private void sceneChanging() throws SubarchitectureProcessException {
		log("scene changing");
		m_sceneChanged.getData().m_sceneChanging = true;
		m_sceneChanged.getData().m_sceneChanged = false;
		m_sceneChanged.getData().m_sceneProcessed = false;
		overwriteWorkingMemory(m_sceneChanged.getID(), m_sceneChanged.getData());
	}

	private void sceneChanged() throws SubarchitectureProcessException {
		log("scene changed");
		m_sceneChanged.getData().m_sceneChanging = false;
		m_sceneChanged.getData().m_sceneChanged = true;
		m_sceneChanged.getData().m_sceneProcessed = false;
		overwriteWorkingMemory(m_sceneChanged.getID(), m_sceneChanged.getData());
	}

	private String processObject(float _x, float _y, String _label,
			float _labelConfidence, String _colour, float _colourConfidence,
			String _size, float _sizeConfidence, boolean _useLabels)
			throws SubarchitectureProcessException {

		return processObject(_x, _y, 0f, 0.05f, 0.05f, 0.05f, _label,
				_labelConfidence, _colour, _colourConfidence, _size,
				_sizeConfidence, _useLabels);

	}

	private void changePostition(float _x, float _y, SceneObject _object) {
		_object.m_bbox.m_centroid.m_x = _x;
		_object.m_bbox.m_centroid.m_y = _y;
	}

	private String processObject(float _x, float _y, String _label,
			float _labelConfidence, boolean _useLabels)
			throws SubarchitectureProcessException {

		String objID = newDataID();

		SceneObject obj1 = segmentSceneObject(_x, _y, 0, 0.05f, 0.05f, 0.1f);

		// ROI obj1ROI =
		// new ROI(new Image(0, 0, 0, new char[0]), new Image(0,
		// 0, 0, new char[0]), new Matrix(new int[0],
		// new double[0]), new Matrix(new int[0],
		// new double[0]), new Matrix(new int[0],
		// new double[0]), objID, new BBox2D(
		// new Vector2D(0, 0), new Vector2D(0, 0)), CAM_NUM,
		// NativeProcessLauncher.getBALTTime());

		if (_useLabels) {
			obj1.m_label.m_string = _label;
			obj1.m_label.m_confidence = _labelConfidence;
		}

		addToWorkingMemory(objID, obj1, OperationMode.BLOCKING);
		updateObject(objID, true, false, false, false, false);

		m_id2so.put(objID, obj1);

		// addToWorkingMemory(newDataID(), VisionOntology.ROI_TYPE,
		// obj1ROI, true);

		return objID;

	}

	private String processObject(float _x, float _y, float _z, float _width,
			float _depth, float _height, String _label, float _labelConfidence,
			String _colour, float _colourConfidence, String _size,
			float _sizeConfidence, boolean _useLabels)
			throws SubarchitectureProcessException {

		String objID = newDataID();

		if (!_useLabels) {
			_label = objID;
		}

		SceneObject obj = segmentSceneObject(_x, _y, _z, _width, _depth,
				_height);

		obj.m_color = new IntWithConfidence(VisualAttributeMap
				.attribute(_colour), _colourConfidence);

		obj.m_label = new StringWithConfidence(_label, _labelConfidence);

		// obj1.m_generic =

		// ROI obj1ROI =
		// new ROI(new Image(0, 0, 0, new char[0]), new Image(0,
		// 0, 0, new char[0]), new Matrix(new int[0],
		// new double[0]), new Matrix(new int[0],
		// new double[0]), new Matrix(new int[0],
		// new double[0]), objID, new BBox2D(
		// new Vector2D(0, 0), new Vector2D(0, 0)), CAM_NUM,
		// NativeProcessLauncher.getBALTTime());
		//
		// addToWorkingMemory(newDataID(), VisionOntology.ROI_TYPE,
		// obj1ROI);

		addToWorkingMemory(objID, obj);
		log("added scene object: " + _label);
		if (_colourConfidence > 0) {
			updateObject(objID, true, false, false, true, false);
		}
		else {
			updateObject(objID, true, false, false, false, false);
		}

		m_id2so.put(objID, obj);

		// waitKey();

		// sleep for a bit
		// sleepProcess(200);

		// colourObject(obj1, _colour, _colourConfidence);
		// log("coloured scene object: " + _label);
		// sizeObject(obj1, _size, _sizeConfidence);
		// log("sized scene object: " + _label);

		// overwriteWorkingMemory(objID,
		// 
		// obj1);

		return objID;

	}

	private void redObject() throws SubarchitectureProcessException,
			SubarchitectureProcessException {
		sceneChanging();
		sleepProcess(2000);

		processObject(-0.1f, -0.1f, "obj0", 0.2f, "red", 1f, "small", 1f, true);

		sleepProcess(20);
		sceneChanged();
	}

	private void redAndBlueObjects() throws SubarchitectureProcessException,
			SubarchitectureProcessException {
		sceneChanging();
		sleepProcess(2000);

		processObject(-0.1f, -0.1f, "obj0", 0.2f, "red", 1f, "small", 1f, true);

		sleepProcess(2000);

		processObject(0.1f, -0.1f, "obj1", 0.2f, "blue", 1f, "small", 1f, true);

		sleepProcess(20);
		sceneChanged();
	}

	private void sceneForPlanningTest() throws SubarchitectureProcessException,
			SubarchitectureProcessException {

		log("DummySceneObjectCreator.sceneForPlanningTest()");

		sceneChanging();
		sleepProcess(1000);

		long interObjectDelay = 1;

		String id1 = processObject(-0.38f, 0.017f, "thing", 0.2f, "red", 1f,
				"small", 1f, true);
		// sceneChanged();
		sleepProcess(interObjectDelay);
		log("added object: " + id1);

		String id2 = processObject(-0.42f, 0.22f, "thing", 0.2f, "blue", 1f,
				"small", 1f, true);
		log("added object: " + id2);
		// sceneChanged();
		sleepProcess(interObjectDelay);

		// String id3 = processObject(0.1f, 0.1f, "ball", 0.2f, "blue", 1f,
		// "small", 1f, true);
		// log("added object: " + id3);
		// sceneChanged();
		// sleepProcess(interObjectDelay);

		sceneProcessed();
		sleepProcess(60000);

		// with comsys
		// sleepProcess(44000);

		// sleepProcess(12000);
		//        
		// log("\n\n\n\n\n\n\nn");
		//
		// deleteObject(id1);
		// sleepProcess(interObjectDelay);
		// sceneProcessed();
		log("\n\n\n\n\n\n\n\nCHANGE SCENE");
		sceneProcessed();

		//
		// addObject(id1);
		// sceneProcessed();

	}

	private void sceneForSpaceTest() throws SubarchitectureProcessException,
			SubarchitectureProcessException {

		log("DummySceneObjectCreator.sceneForSpaceTest()");

		long interObjectDelay = 1000;
		long interSceneDelay = 4000;
		long sceneChangingDelay = 3000;

		//
		// add one object
		//
		sceneChanging();
		sleepProcess(sceneChangingDelay);

		String id1 = processObject(-0.38f, 0.017f, "thing", 0.2f, "red", 1f,
				"small", 1f, true);
		sleepProcess(interObjectDelay);
		sceneChanged();
		sceneProcessed();
		log("added object 1: " + id1);
		sleepProcess(interSceneDelay);

		//
		// add another object
		//
		sceneChanging();
		sleepProcess(sceneChangingDelay);

		String id2 = processObject(-0.42f, 0.22f, "thing", 0.2f, "red", 1f,
				"small", 1f, true);
		sleepProcess(interObjectDelay);
		sceneChanged();
		sceneProcessed();
		log("added object 2: " + id2);
		sleepProcess(interSceneDelay);

		//
		// add a third
		//
		sceneChanging();
		sleepProcess(sceneChangingDelay);
		String id3 = processObject(-0.22f, 0.22f, "thing", 0.2f, "red", 1f,
				"small", 1f, true);
		sleepProcess(interObjectDelay);
		sceneChanged();
		sceneProcessed();
		log("added object 3: " + id3);
		sleepProcess(interSceneDelay);

		// //
		// // move first a bit
		// //
		// sceneChanging();
		// sleepProcess(sceneChangingDelay);
		// changePostition(-0.38f, 0.057f, m_id2so.get(id1));
		// overwriteWorkingMemory(id1, m_id2so.get(id1),
		// OperationMode.BLOCKING);
		// updateObject(id1, true, false, false, false);
		// sleepProcess(interObjectDelay);
		// changePostition(-0.38f, 0.137f, m_id2so.get(id1));
		// overwriteWorkingMemory(id1, m_id2so.get(id1),
		// OperationMode.BLOCKING);
		// updateObject(id1, true, false, false, false);
		// sleepProcess(interObjectDelay);
		// changePostition(-0.38f, 0.157f, m_id2so.get(id1));
		// overwriteWorkingMemory(id1, m_id2so.get(id1),
		// OperationMode.BLOCKING);
		// updateObject(id1, true, false, false, false);
		// sleepProcess(interObjectDelay);
		// changePostition(-0.38f, 0.157f, m_id2so.get(id1));
		// overwriteWorkingMemory(id1, m_id2so.get(id1),
		// OperationMode.BLOCKING);
		// updateObject(id1, true, false, false, false);
		// sleepProcess(interObjectDelay);
		// sceneChanged();
		// sceneProcessed();
		// log("moved object 1: " + id1);
		// sleepProcess(interSceneDelay);
		//
		//		
		//		
		// //
		// // remove first
		// //
		// sceneChanging();
		// sleepProcess(sceneChangingDelay);
		// deleteFromWorkingMemory(id1, OperationMode.BLOCKING);
		// sleepProcess(interObjectDelay);
		// sceneChanged();
		// sceneProcessed();
		// log("removed object 1: " + id1);
		// sleepProcess(interSceneDelay);
		//
		// //
		// // remove third
		// //
		// sceneChanging();
		// sleepProcess(sceneChangingDelay);
		// deleteFromWorkingMemory(id3, OperationMode.BLOCKING);
		// sleepProcess(interObjectDelay);
		// sceneChanged();
		// sceneProcessed();
		// log("removed object 3: " + id3);
		// sleepProcess(interSceneDelay);
		//		
		// //
		// // remove second
		// //
		// sceneChanging();
		// sleepProcess(sceneChangingDelay);
		// deleteFromWorkingMemory(id2, OperationMode.BLOCKING);
		// sleepProcess(interObjectDelay);
		// sceneChanged();
		// sceneProcessed();
		// log("removed object 2: " + id2);
		// sleepProcess(interSceneDelay);
		//
		//		
		//		
		// //
		// // re-add object 3
		// //
		// sceneChanging();
		// sleepProcess(sceneChangingDelay);
		// reAddObject(id3);
		// sleepProcess(interObjectDelay);
		// sceneChanged();
		// sceneProcessed();
		// log("re-added object 3: " + id3);
		// sleepProcess(interSceneDelay);
		//		
		// //
		// // re-add object 1
		// //
		// sceneChanging();
		// sleepProcess(sceneChangingDelay);
		// reAddObject(id1);
		// sleepProcess(interObjectDelay);
		// sceneChanged();
		// sceneProcessed();
		// log("re-added object 1: " + id1);
		// sleepProcess(interSceneDelay);
		//
		// //
		// // re-add object 2
		// //
		// sceneChanging();
		// sleepProcess(sceneChangingDelay);
		// reAddObject(id2);
		// sleepProcess(interObjectDelay);
		// sceneChanged();
		// sceneProcessed();
		// log("re-added object 2: " + id2);
		// sleepProcess(interSceneDelay);

	}

	public static Shape randomShape() {
		return Shape.from_int(m_rand.nextInt(Shape._UNKNOWN_SHAPE));
	}

	public static SceneObject segmentSceneObject(float _x, float _y, float _z,
			float _width, float _depth, float _height) {

		return newSceneObject(_x, _y, _z, _width, _depth, _height, "", -1, -1,
				new String[0]);

	}

	public static void siftObject(SceneObject _so, String _label,
			float _confidence) {
		_so.m_label.m_string = _label;
		_so.m_label.m_confidence = _confidence;
	}

	public static void colourObject(SceneObject _so, String _colour,
			float _confidence) throws SubarchitectureProcessException {
		if (VisualAttributeMap.isColour(_colour)) {
			_so.m_color.m_int = VisualAttributeMap.attribute(_colour);
			_so.m_color.m_confidence = _confidence;
		}
		else {
			throw new SubarchitectureProcessException("not a colour string: "
					+ _colour);
		}
	}

	public void colourObject(String _soID, String _colour, float _confidence)
			throws SubarchitectureProcessException {
		if (VisualAttributeMap.isColour(_colour)) {

			// fetch requested object from wm
			CASTData<SceneObject> so = (CASTData<SceneObject>) getWorkingMemoryEntry(_soID);

			if (so != null) {
				// add (or overwrite) colour data
				colourObject(so.getData(), _colour, _confidence);

				// write back to wm
				overwriteWorkingMemory(_soID, so.getData());
				updateObject(_soID, false, false, false, true, false);

			}
			else {
				throw new SubarchitectureProcessException(
						"scene object does not exist: " + _soID);

			}

		}
	}

	private void colourObject(String _soID, int _colour, float _confidence)
			throws SubarchitectureProcessException {
		// if (VisualAttributeMap.isColour(_colour)) {

		// fetch requested object from wm
		SceneObject so = (SceneObject) getWorkingMemoryEntry(_soID).getData();

		if (so != null) {
			if (_confidence > 0) {
				so.m_color.m_int = _colour;
				so.m_color.m_confidence = _confidence;

				// write back to wm
				overwriteWorkingMemory(_soID, so, OperationMode.BLOCKING);
				updateObject(_soID, false, false, false, true, false);
			}
		}
		else {
			throw new SubarchitectureProcessException(
					"scene object does not exist: " + _soID);

		}

		// }
	}

	private void shapeObject(String _soID, int _shape, float _confidence)
			throws SubarchitectureProcessException {
		// if (VisualAttributeMap.isShape(_shape)) {

		// fetch requested object from wm
		SceneObject so = (SceneObject) getWorkingMemoryEntry(_soID).getData();

		if (so != null) {
			if (_confidence > 0) {
				so.m_shape.m_int = _shape;
				so.m_shape.m_confidence = _confidence;

				// write back to wm
				overwriteWorkingMemory(_soID, so, OperationMode.BLOCKING);
				updateObject(_soID, false, true, false, false, false);
			}
		}
		else {
			throw new SubarchitectureProcessException(
					"scene object does not exist: " + _soID);
		}
		// }
	}

	public static void shapeObject(SceneObject _so, String _shape,
			float _confidence) throws SubarchitectureProcessException {
		if (VisualAttributeMap.isShape(_shape)) {
			_so.m_shape.m_int = VisualAttributeMap.attribute(_shape);
			_so.m_shape.m_confidence = _confidence;
		}
		else {
			throw new SubarchitectureProcessException("not a shape string: "
					+ _shape);
		}
	}

	public static void sizeObject(SceneObject _so, String _size,
			float _confidence) throws SubarchitectureProcessException {
		if (VisualAttributeMap.isSize(_size)) {
			_so.m_size.m_int = VisualAttributeMap.attribute(_size);
			_so.m_size.m_confidence = _confidence;
		}
		else {
			throw new SubarchitectureProcessException("not a size string: "
					+ _size);
		}
	}

	public static SceneObject newSceneObject(float _x, float _y, float _z,
			float _width, float _depth, float _height, String _label,
			int _colour, int _shape, String[] _roiIDs) {

		return new SceneObject(ProcessLauncher.getBALTTime(),
				new BBox3D(new Vector3D(_x, _y, _z), new Vector3D(_width,
						_depth, _height)), new Pose3D(new Vector3D(0, 0, 0),
						new Vector3D(0, 0, 0)), new StringWithConfidence(
						_label, 1), new Surface[0], new IntWithConfidence(
						_colour, 0), new IntWithConfidence(_shape, 0),
				new IntWithConfidence(0, 0), new IntWithConfidence(0, 0),
				new IntWithConfidence(0, 0), new IntWithConfidence(0, 0),
				new IntWithConfidence(0, 0), new IntWithConfidence(0, 0),
				_roiIDs, _label);

	}

	public static Colour randomColour() {
		return Colour.from_int(m_rand.nextInt(Colour._UNKNOWN_COLOUR));
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _taskID) {
		// TODO Auto-generated method stub

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _taskID) {
		// TODO Auto-generated method stub

	}

}
