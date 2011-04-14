package manipulation.visualisation;

import java.awt.Component;
import java.awt.Container;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.Item.ItemIntention;
import manipulation.itemMemory.Item.ItemName;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.runner.cogx.CogXRunner;
import manipulation.slice.CloseGripperCommand;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.LinearGraspApproachCommand;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.OpenGripperCommand;
import manipulation.slice.PutDownCommand;
import manipulation.slice.SimulateGraspCommand;
import manipulation.slice.StopCommand;

import org.apache.log4j.Logger;

import VisionData.Face;
import VisionData.GeometryModel;
import VisionData.Vertex;
import VisionData.VisualObject;
import VisionData.VisualObjectView;
import cast.AlreadyExistsOnWMException;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cogx.Math.Matrix33;
import cogx.Math.Pose3;
import cogx.Math.Rect2;
import cogx.Math.Sphere3;
import cogx.Math.Vector2;
import cogx.Math.Vector3;

/**
 * GUI to use for the calibration procedure
 * 
 * @author ttoenige
 * 
 */
public class CogXTestGUI extends JPanel implements ActionListener {

	private Logger logger = Logger.getLogger(this.getClass());

	private static final long serialVersionUID = 1489792232139115240L;

	private Manipulator manipulator;

	private JTextField txtItemXPosition;
	private JTextField txtItemYPosition;
	private JTextField txtItemZPosition;

	private JButton btnPutDown;

	private JButton btnFarArm;

	private JButton btnLinGraspApp;

	private JButton btnSimGrasp;

	private JButton btnLinBaseApp;

	private JButton btnStopCmd;

	private JButton btnMoveHomeCmd;

	private JButton btnOpenGripperCmd;

	private JButton btnCloseGripperCmd;

	private JButton btnRecognize;

	/**
	 * constructor for the cogx test GUI, displays the GUI and can be used test
	 * the implemented planning actions
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param mem
	 *            corresponding item memory
	 */
	public CogXTestGUI(Manipulator manipulator) {
		this.manipulator = manipulator;

		guiSetup();
	}

	private VisualObject initVisualObject() {
		Pose3 initPos = new Pose3();
		initPos.pos = new Vector3(0, 0, 0);
		initPos.rot = new Matrix33(0, 0, 0, 0, 0, 0, 0, 0, 0);
		String[] intStringArray = new String[1];
		intStringArray[0] = "";
		double[] initDoubleArray = new double[1];
		initDoubleArray[0] = 0;
		Vertex[] initVertex = new Vertex[1];
		initVertex[0] = new Vertex(new Vector3(0, 0, 0), new Vector3(0, 0, 0),
				new Vector2(0, 0));
		Face[] initFace = new Face[1];
		int[] initIntArray = new int[1];
		initIntArray[0] = 0;
		initFace[0] = new Face(initIntArray);
		CASTTime initTime = ((CogXRunner) manipulator.getRunner())
				.getTimeServer().getCASTTime();
		Sphere3 initSphere = new Sphere3(new Vector3(0, 0, 0), 0);
		VisualObjectView initVisualObjView = new VisualObjectView(new Rect2(
				new Vector2(0, 0), 0, 0), 0, 0);
		VisualObjectView[] initVisualObjViewArray = new VisualObjectView[1];
		initVisualObjViewArray[0] = initVisualObjView;
		GeometryModel initGeomModel = new GeometryModel(initVertex, initFace);
		String compID = ((CogXRunner) manipulator.getRunner()).getComponentID();
		VisualObject visObj = new VisualObject(initPos, intStringArray, 0,
				initSphere, initTime, compID, initVisualObjViewArray,
				initGeomModel, 0, intStringArray, initDoubleArray, 0, 0,
				intStringArray, initDoubleArray, initDoubleArray, 0, 0,
				intStringArray, initDoubleArray, initDoubleArray, 0, 0, "", "");

		return visObj;
	}

	/**
	 * constructor to show the GUI without any functionality
	 */
	public CogXTestGUI() {
		guiSetup();
	}

	private static void addComponent(Container cont, GridBagLayout gbl,
			Component c, int x, int y, int width, int height, double weightx,
			double weighty) {
		GridBagConstraints gbc = new GridBagConstraints();
		gbc.fill = GridBagConstraints.BOTH;
		gbc.gridx = x;
		gbc.gridy = y;
		gbc.gridwidth = width;
		gbc.gridheight = height;
		gbc.weightx = weightx;
		gbc.weighty = weighty;
		gbl.setConstraints(c, gbc);
		cont.add(c);
	}

	private void guiSetup() {
		JFrame gui = new JFrame("Test GUI");
		gui.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		gui.setLocation(100, 200);

		Container pane = gui.getContentPane();
		GridBagLayout gbl = new GridBagLayout();
		pane.setLayout(gbl);

		txtItemXPosition = new JTextField("", 7);
		txtItemYPosition = new JTextField("", 7);
		txtItemZPosition = new JTextField("", 7);

		btnPutDown = new JButton("put down command");
		btnPutDown.setActionCommand("putDown");
		btnPutDown.addActionListener(this);

		btnFarArm = new JButton("far arm movement");
		btnFarArm.setActionCommand("farArm");
		btnFarArm.addActionListener(this);

		btnLinGraspApp = new JButton("linear grasping approach");
		btnLinGraspApp.setActionCommand("linGraspApp");
		btnLinGraspApp.addActionListener(this);

		btnSimGrasp = new JButton("simulate grasping");
		btnSimGrasp.setActionCommand("simulateGrasp");
		btnSimGrasp.addActionListener(this);

		btnLinBaseApp = new JButton("linear base approach");
		btnLinBaseApp.setActionCommand("linBaseApp");
		btnLinBaseApp.addActionListener(this);

		btnStopCmd = new JButton("stop arm");
		btnStopCmd.setActionCommand("stopCmd");
		btnStopCmd.addActionListener(this);

		btnMoveHomeCmd = new JButton("move arm to home position");
		btnMoveHomeCmd.setActionCommand("moveHome");
		btnMoveHomeCmd.addActionListener(this);

		btnOpenGripperCmd = new JButton("open gripper");
		btnOpenGripperCmd.setActionCommand("openGripper");
		btnOpenGripperCmd.addActionListener(this);

		btnCloseGripperCmd = new JButton("close gripper");
		btnCloseGripperCmd.setActionCommand("closeGripper");
		btnCloseGripperCmd.addActionListener(this);

		btnRecognize = new JButton("recognize");
		btnRecognize.setActionCommand("recognize");
		btnRecognize.addActionListener(this);

		// cont, gbl, comp, x, y, width, height, weightx, weighty
		addComponent(pane, gbl, new JLabel("x:"), 0, 0, 1, 1, 0, 0);
		addComponent(pane, gbl, txtItemXPosition, 1, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("y:"), 3, 0, 1, 1, 0, 0);
		addComponent(pane, gbl, txtItemYPosition, 4, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("z:"), 6, 0, 1, 1, 0, 0);
		addComponent(pane, gbl, txtItemZPosition, 7, 0, 2, 1, 0, 0);

		addComponent(pane, gbl, btnPutDown, 0, 1, 12, 1, 0, 0);

		addComponent(pane, gbl, btnFarArm, 0, 2, 12, 1, 0, 0);

		addComponent(pane, gbl, btnLinGraspApp, 0, 3, 12, 1, 0, 0);

		addComponent(pane, gbl, btnSimGrasp, 0, 4, 12, 1, 0, 0);

		addComponent(pane, gbl, btnLinBaseApp, 0, 5, 12, 1, 0, 0);

		addComponent(pane, gbl, btnStopCmd, 0, 6, 12, 1, 0, 0);

		addComponent(pane, gbl, btnMoveHomeCmd, 0, 7, 12, 1, 0, 0);

		addComponent(pane, gbl, btnOpenGripperCmd, 0, 8, 12, 1, 0, 0);

		addComponent(pane, gbl, btnCloseGripperCmd, 0, 9, 12, 1, 0, 0);

		addComponent(pane, gbl, btnRecognize, 0, 10, 12, 1, 0, 0);

		gui.pack();
		gui.setVisible(true);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * java.awt.event.ActionListener#actionPerformed(java.awt.event.ActionEvent)
	 */
	@Override
	public void actionPerformed(ActionEvent e) {
		if (e.getActionCommand().equals("putDown")) {
			logger.error("putDown pressed");

			String visObjID = ((CogXRunner) manipulator.getRunner())
					.newDataID();
			PutDownCommand putDownCommand = new PutDownCommand();

			Pose3 pos = new Pose3();
			pos.pos = new Vector3(
					Double.parseDouble(txtItemXPosition.getText()),
					Double.parseDouble(txtItemYPosition.getText()),
					Double.parseDouble(txtItemZPosition.getText()));
			pos.rot = new Matrix33(0, 0, 0, 0, 0, 0, 0, 0, 0);

			VisualObject visObj = initVisualObject();
			visObj.pose = pos;

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(
						visObjID, visObj);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}

			putDownCommand.basedObjectAddr = new WorkingMemoryAddress(visObjID,
					((CogXRunner) manipulator.getRunner())
							.getSubarchitectureID());

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						putDownCommand);

			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}

		} else if (e.getActionCommand().equals("farArm")) {
			logger.error("farArm pressed");
			FarArmMovementCommand farArmMovementCom = new FarArmMovementCommand();

			if (!txtItemXPosition.getText().isEmpty()) {
				String visObjID = ((CogXRunner) manipulator.getRunner())
						.newDataID();

				Pose3 pos = new Pose3();

				pos.pos = new Vector3(Double.parseDouble(txtItemXPosition
						.getText()), Double.parseDouble(txtItemYPosition
						.getText()), Double.parseDouble(txtItemZPosition
						.getText()));

				pos.rot = new Matrix33(0, 0, 0, 0, 0, 0, 0, 0, 0);

				VisualObject visObj = initVisualObject();
				visObj.pose = pos;

				try {
					((CogXRunner) manipulator.getRunner()).addToWorkingMemory(
							visObjID, visObj);
				} catch (AlreadyExistsOnWMException e1) {
					logger.error(e1);
				}

				farArmMovementCom.targetObjectAddr = new WorkingMemoryAddress(
						visObjID,
						((CogXRunner) manipulator.getRunner())
								.getSubarchitectureID());

			} else {
				Item it = manipulator.getItemMemory().getItemList().getFirst();
				try {
					WorkingMemoryAddress wma = (WorkingMemoryAddress) it
							.getAttribute(PropertyName.WMA_ADDRESS);

					farArmMovementCom.targetObjectAddr = wma;
				} catch (ItemException e1) {
					logger.error(e1);
				}

			}

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						farArmMovementCom);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("linGraspApp")) {
			logger.error("linGraspApp pressed");

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			LinearGraspApproachCommand linGraspApproachCom = new LinearGraspApproachCommand();
			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						linGraspApproachCom);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("simulateGrasp")) {
			logger.error("simulateGrasp pressed");

			String visObjID = ((CogXRunner) manipulator.getRunner())
					.newDataID();
			SimulateGraspCommand simulateGraspCommand = new SimulateGraspCommand();

			Pose3 pos = new Pose3();
			pos.pos = new Vector3(
					Double.parseDouble(txtItemXPosition.getText()),
					Double.parseDouble(txtItemYPosition.getText()),
					Double.parseDouble(txtItemZPosition.getText()));
			pos.rot = new Matrix33(0, 0, 0, 0, 0, 0, 0, 0, 0);

			VisualObject visObj = initVisualObject();
			visObj.pose = pos;

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(
						visObjID, visObj);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}

			simulateGraspCommand.targetObjectAddr = new WorkingMemoryAddress(
					visObjID,
					((CogXRunner) manipulator.getRunner())
							.getSubarchitectureID());

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						simulateGraspCommand);

			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("stopCmd")) {
			logger.error("stopCmd pressed");

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			StopCommand stopCommand = new StopCommand();
			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						stopCommand);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("moveHome")) {
			logger.error("moveHome pressed");

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			MoveArmToHomePositionCommand moveHomeCmd = new MoveArmToHomePositionCommand();
			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						moveHomeCmd);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("openGripper")) {
			logger.error("openGripper pressed");

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			OpenGripperCommand openGripperCmd = new OpenGripperCommand();
			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						openGripperCmd);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("closeGripper")) {
			logger.error("closeGripper pressed");

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			CloseGripperCommand closeGripperCmd = new CloseGripperCommand();
			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						closeGripperCmd);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("recognize")) {
			logger.error("recognize pressed");

			Item item = new Item();
			item.setAttribute(PropertyName.NAME, ItemName.FROSTIES_SMALL);
			item.setAttribute(PropertyName.WORLD_POSITION,
					new Vector3D(0, 0, 0));
			item.setAttribute(PropertyName.WORLD_ROTATION, new Matrix(1, 0, 0,
					0, 1, 0, 0, 0, 1));
			item.setAttribute(PropertyName.INTENTION, ItemIntention.GRASP_ME);

			try {
				boolean success = manipulator.getCamConnector()
						.recognizeTrackItem(item);

				logger.error("TRACKINGSUCCESS: " + success);

			} catch (ExternalMemoryException e1) {
				logger.error("Geht nicht!");
				logger.error(e1);
			}

		}
	}

	/**
	 * main function to show the GUI without any functionality
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		new CogXTestGUI();
	}

}
