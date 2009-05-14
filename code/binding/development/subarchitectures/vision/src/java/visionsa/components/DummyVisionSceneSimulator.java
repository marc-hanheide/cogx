package visionsa.components;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Properties;
import java.util.Random;
import java.util.StringTokenizer;
import java.io.*;

import org.cognitivesystems.common.autogen.Math.Pose3D;
import org.cognitivesystems.common.autogen.Math.Vector3D;


import balt.management.ProcessLauncher;

import Vision.BBox3D;
import Vision.Camera;
import Vision.IntWithConfidence;
import Vision.ObjectPropertyUpdates;
import Vision.SceneChanged;
import Vision.SceneObject;
import Vision.SceneObjectUpdate;
import Vision.StringWithConfidence;
import Vision.Surface;
import cast.architecture.subarchitecture.AlreadyExistsOnWMException;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.OperationMode;
import cast.cdl.testing.CAST_TEST_FAIL;
import cast.cdl.testing.CAST_TEST_PASS;
import cast.core.CASTUtils;
import cast.core.data.CASTData;

public class DummyVisionSceneSimulator extends ManagedProcess {

	//Class Members

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
	
	private boolean m_cameraAdded = false;

	static {
		m_rand = new Random();
	}

	private String m_fileLoc;
	private File m_mainConfigFile;
	private File m_simuFile;
	private File m_sceneConfigFile;
	private 	Integer m_linesRead =0;
	//Constructor
	public DummyVisionSceneSimulator(String _id){
		super(_id);
		m_id2so = new Hashtable<String, SceneObject>();
	}
	
	/*
	 * All the dummy visual scene configurations files have been indexed in a --mconf file. 
	 * By specifying these index one at a time through --simfile simulator file a user can 
	 * instantiate that particular visual scene.
	 * 
	 * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
	 */
	@Override
	public void configure(Properties _config ){
		super.configure(_config);
	try{
		
		//String s_tmp = "";
		//Directory where I/O files are kept
		String fileLoc = _config.getProperty("--loc");
		if(fileLoc != null){
			m_fileLoc = fileLoc;
		}
	    //file containing all the visual configurations.
		String confFile = _config.getProperty("--mconf");
		if(confFile != null){
			//s_tmp.concat(confFile);
			m_mainConfigFile = new File(m_fileLoc+confFile);
			if (!m_mainConfigFile.exists()) {
				throw new FileNotFoundException("main configuration file does not exist: " + m_mainConfigFile);
			}
		}
		//file to readin user specified index from confFile, to simulate config specific vision scene
		String simuFile = _config.getProperty("--simfile");
		//System.out.println("simfile: " + simuFile );
		if(simuFile != null){
		    FileWriter _simuFile = new FileWriter(m_fileLoc+simuFile);
             BufferedWriter OutPutBuff = new BufferedWriter(_simuFile);
             OutPutBuff.flush();
             //OutPutBuff.write(" ") ;
             OutPutBuff.close();
             m_simuFile = new File(m_fileLoc+simuFile);;
		}
	
		String cameraString = _config.getProperty("--camera");
		if (cameraString != null) {
			m_addingCamera = Boolean.parseBoolean(cameraString);
		}
		else {
			m_addingCamera = true;
		}
		log("add camera: " + m_addingCamera);
	}
	catch( IOException io)
	{}
	//Scene change between each object
	}
	
	@Override
	protected void runComponent() {
		try {

			Boolean loop=true;
			// sleep for a bit
			sleepProcess(10000);
			while (m_status == ProcessStatus.RUN ) {
				while (loop==true ){
					loop=simulateVisualScene(m_simuFile, m_mainConfigFile);
				}
				loop=true;
				if(!m_cameraAdded){
					addCamera(m_sceneCamera);
					m_cameraAdded=true;
				}
				addSceneChanged();
				addObjectsInSerial(m_sceneObjects);
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
 * @param __simuFile, __confFile
 * @throws IOException
 * @throws SubarchitectureProcessException
 */
private boolean simulateVisualScene(File _simuFile, File _confFile) throws IOException,
		SubarchitectureProcessException {
	
	String line ="";
	String index="";	
	//Read the index value
	

	Integer NewLineCount =0;
	BufferedReader indexReader = new BufferedReader(new FileReader(_simuFile));
	while(indexReader.ready()){
		line = indexReader.readLine().trim();
		NewLineCount++;
		//	System.out.println("simfile length: " + line.length() + "string" + line );
	}
	//System.out.println("simfile length: " + line.length() );
	if(line.length() == 0){
		NewLineCount=0;
		m_linesRead =0;
		log("To simulate a visual scene append to the Simulation File the index value of corresponding SceneConfig File.");
		sleepProcess(10000);
		return true;
	}	
	else if(NewLineCount == m_linesRead){
		log("To simulate a visual scene append to the Simulation File the index value of corresponding SceneConfig File.");
		sleepProcess(10000);
		return true;
	}
	else if(NewLineCount > m_linesRead){
		m_linesRead++;
		index=line;
				
		//get the scene configuration filename corresponding to this index from the main configuration file.
		BufferedReader confReader = new BufferedReader(new FileReader(_confFile));
		HashMap<Integer,String> sceneConfigs = new HashMap<Integer,String>();

		while (confReader.ready()) {
			line = confReader.readLine().trim();
			//System.out.println("line: " + line );
			if (line.length() > 0 && (line.charAt(0) != COMMENT_CHAR)) {
				StringTokenizer LineRead = new StringTokenizer(line," ");
				sceneConfigs.put(Integer.valueOf(LineRead.nextToken()),LineRead.nextToken());
				//System.out.println("1: " + Integer.valueOf(LineRead.nextToken()) );
				//System.out.println("2: " + LineRead.nextToken() );
			//return false;
			}
		}

		//Get the visual scene config filename
		System.out.println("Index: " + index );
		String visScene = sceneConfigs.get(Integer.valueOf(index));
		System.out.println("filename: " + visScene );
		m_sceneConfigFile =new File(m_fileLoc+visScene);
		//s_tmp.concat(visScene);
		//System.out.println("config file: " + configfile);
		//get the visual scene configuration
		BufferedReader sceneReader = new BufferedReader(new FileReader(m_sceneConfigFile));
		ArrayList<String> lines = new ArrayList<String>();
		
		while (sceneReader.ready()) {
			line = sceneReader.readLine().trim();
			if (line.length() > 0 && (line.charAt(0) != COMMENT_CHAR)) {
				lines.add(line);
			}
		}

		if (lines.size() < 3) {
			throw new SubarchitectureProcessException(
				"Not enough lines in file. It should look like:\nx y z\nx y z\n object desc");
		}

		// And now go through creating vision and scene related stuff

		// first line in config file should be camera position followed by camera orientation
		m_sceneCamera = new Camera(CAM_NUM, new Pose3D(parseVector3D(lines.get(0)),
													parseVector3D(lines.get(1))),
													0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0);

		log("Camera at: " + lines.get(0) + ", " + lines.get(1));

		// the rest should be objects
		String[] tokens;
		StringTokenizer tags;
		float confidence;
		SceneObject so;
		m_sceneObjects = new ArrayList<SceneObject>(lines.size() - 2); //minus lines for camera setup
		String tag, value;

		for (int i = 2; i < lines.size(); i++) {
			//Split on ',' for various features
			//first is object position, then comes color, then size, then label followed by shape
			tokens = lines.get(i).split(",");

			if (tokens.length == 0) {
				throw new SubarchitectureProcessException(
					"all objects must at least have a position");
			}

			// parse the position
			Vector3D position = parseVector3D(tokens[0]);

			// create an empty scene object
			so = newSceneObject(position.m_x, position.m_y, position.m_z, 
								0.2f,0.2f, 0.2f, "", 0, 0, new String[0]);
	
			for (int j = 1; j < tokens.length; j++) {
				//now spilt on space for feature values
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
					log("parsing shape: " + value + " " + confidence);
					so.m_shape.m_int = VisualAttributeMap.attribute(value);
					so.m_shape.m_confidence = confidence;
				}
				else if (tag.equals(LABEL_TAG)) {
					so.m_label.m_string = value;
					so.m_label.m_confidence = confidence;
				}
			}

			log("parsed object: " + position.m_x + " " + position.m_y + " "	+ position.m_z);
			m_sceneObjects.add(so);
		}
		return false;
		}
	return true;
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

	private void sceneProcessed() throws SubarchitectureProcessException {
		log("scene processed");
		m_sceneChanged.getData().m_sceneChanging = false;
		m_sceneChanged.getData().m_sceneChanged = false;
		m_sceneChanged.getData().m_sceneProcessed = true;
		overwriteWorkingMemory(m_sceneChanged.getID(), m_sceneChanged.getData());
	}

	/*private String processObject(float _x, float _y, String _label,
			float _labelConfidence, String _colour, float _colourConfidence,
			String _size, float _sizeConfidence, boolean _useLabels)
			throws SubarchitectureProcessException {

		return processObject(_x, _y, 0f, 0.05f, 0.05f, 0.05f, _label,
				_labelConfidence, _colour, _colourConfidence, _size,
				_sizeConfidence, _useLabels);

	}*/
	
	
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
	
	/*private String processObject(float _x, float _y, float _z, float _width,
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

	}*/
	
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
	private void updateObject(String _id, boolean _position, boolean _shape,
			boolean _size, boolean _colour, boolean _class)
			throws AlreadyExistsOnWMException, SubarchitectureProcessException {
		SceneObjectUpdate up = new SceneObjectUpdate(_id,
				new ObjectPropertyUpdates(_position, _shape, _size, _colour,
						_class));
		addToWorkingMemory(newDataID(), up, OperationMode.BLOCKING);
	}
	
	public static SceneObject segmentSceneObject(float _x, float _y, float _z,
			float _width, float _depth, float _height) {

		return newSceneObject(_x, _y, _z, _width, _depth, _height, "", -1, -1,
				new String[0]);

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
	
	/*public static void shapeObject(SceneObject _so, String _shape,
			float _confidence) throws SubarchitectureProcessException {
		if (VisualAttributeMap.isShape(_shape)) {
			_so.m_shape.m_int = VisualAttributeMap.attribute(_shape);
			_so.m_shape.m_confidence = _confidence;
		}
		else {
			throw new SubarchitectureProcessException("not a shape string: "
					+ _shape);
		}
	}*/

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
	
	
	@Override
	protected void taskAdopted(String _taskID) {
		// TODO Auto-generated method stub

	}

	@Override
	protected void taskRejected(String _taskID) {
		// TODO Auto-generated method stub

	}

}
