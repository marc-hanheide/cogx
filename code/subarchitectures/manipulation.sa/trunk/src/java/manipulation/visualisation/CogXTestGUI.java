package manipulation.visualisation;

import java.awt.Component;
import java.awt.Container;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.DecimalFormat;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ItemException;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.runner.cogx.CogXRunner;
import manipulation.slice.CloseGripperCommand;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.FineArmMovementCommand;
import manipulation.slice.GetCurrentArmPose;
import manipulation.slice.ManipulationCommand;
import manipulation.slice.ManipulationCommandStatus;
import manipulation.slice.GraspingStatus;
import manipulation.slice.ManipulationCompletion;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.MoveArmToPose;
import manipulation.slice.OpenGripperCommand;
import manipulation.slice.PutDownCommand;
import manipulation.slice.SimulateFarArmMovementCommand;
import manipulation.slice.SimulateMoveToPose;
import manipulation.slice.StopCommand;

import org.apache.log4j.Logger;

import VisionData.Face;
import VisionData.GeometryModel;
import VisionData.Vertex;
import VisionData.VisualObject;
import VisionData.VisualObjectView;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cogx.Math.Matrix33;
import cogx.Math.Pose3;
import cogx.Math.Rect2;
import cogx.Math.Sphere3;
import cogx.Math.Vector2;
import cogx.Math.Vector3;

/**
 * GUI to use for the calibration procedure
 * 
 * @author Torben Toeniges
 * 
 */
public class CogXTestGUI extends JPanel implements ActionListener {

	private Logger logger = Logger.getLogger(this.getClass());

	private static final long serialVersionUID = 1489792232139115240L;

	private Manipulator manipulator;

	private JTextField txtItemXPosition;
	private JTextField txtItemYPosition;
	private JTextField txtItemZPosition;

	private JTextField txtrot00;
	private JTextField txtrot01;
	private JTextField txtrot02;

	private JTextField txtrot10;
	private JTextField txtrot11;
	private JTextField txtrot12;

	private JTextField txtrot20;
	private JTextField txtrot21;
	private JTextField txtrot22;

	private JButton btnPutDown;

	private JButton btnFarArm;

	private JButton btnFineArm;

	private JButton btnSimFarArmMovement;

	private JButton btnStopCmd;

	private JButton btnMoveHomeCmd;

	private JButton btnOpenGripperCmd;

	private JButton btnCloseGripperCmd;

	private JButton btnMoveArmToPosCmd;

	private JButton btnGetPosCmd;

	private JButton btnSimulatePosCmd;

	private DecimalFormat df = new DecimalFormat("0.00");

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
		VisualObject visObj = new VisualObject();
/*
		VisualObject visObj = new VisualObject(initPos, intStringArray, 0,
				initSphere, initTime, compID, initVisualObjViewArray,
				initGeomModel, 0, intStringArray, initDoubleArray, 0, 0,
				intStringArray, initDoubleArray, initDoubleArray, 0, 0,
				intStringArray, initDoubleArray, initDoubleArray, 0, 0, "", "",
				initDoubleArray);
*/
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
		JFrame gui = new JFrame("Manipulation test GUI");
		gui.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		gui.setLocation(100, 200);

		Container pane = gui.getContentPane();
		GridBagLayout gbl = new GridBagLayout();
		pane.setLayout(gbl);

		txtItemXPosition = new JTextField("", 7);
		txtItemYPosition = new JTextField("", 7);
		txtItemZPosition = new JTextField("", 7);

		txtrot00 = new JTextField("", 7);
		txtrot01 = new JTextField("", 7);
		txtrot02 = new JTextField("", 7);

		txtrot10 = new JTextField("", 7);
		txtrot11 = new JTextField("", 7);
		txtrot12 = new JTextField("", 7);

		txtrot20 = new JTextField("", 7);
		txtrot21 = new JTextField("", 7);
		txtrot22 = new JTextField("", 7);

		btnPutDown = new JButton("put down command");
		btnPutDown.setActionCommand("putDown");
		btnPutDown.addActionListener(this);

		btnFarArm = new JButton("far arm movement");
		btnFarArm.setActionCommand("farArm");
		btnFarArm.addActionListener(this);

		btnFineArm = new JButton("fine movement + grasping");
		btnFineArm.setActionCommand("fineArm");
		btnFineArm.addActionListener(this);

		btnSimFarArmMovement = new JButton("simulate far arm movement");
		btnSimFarArmMovement.setActionCommand("simulateFar");
		btnSimFarArmMovement.addActionListener(this);

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

		btnMoveArmToPosCmd = new JButton("move arm to pose");
		btnMoveArmToPosCmd.setActionCommand("btnMoveArmToPosCmd");
		btnMoveArmToPosCmd.addActionListener(this);

		btnGetPosCmd = new JButton("get pose");
		btnGetPosCmd.setActionCommand("btnGetPosCmd");
		btnGetPosCmd.addActionListener(this);

		btnSimulatePosCmd = new JButton("simulate move arm to pose");
		btnSimulatePosCmd.setActionCommand("btnSimulatePosCmd");
		btnSimulatePosCmd.addActionListener(this);

		// cont, gbl, comp, x, y, width, height, weightx, weighty
		addComponent(pane, gbl, new JLabel("x:"), 0, 0, 1, 1, 0, 0);
		addComponent(pane, gbl, txtItemXPosition, 1, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("y:"), 3, 0, 1, 1, 0, 0);
		addComponent(pane, gbl, txtItemYPosition, 4, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("z:"), 6, 0, 1, 1, 0, 0);
		addComponent(pane, gbl, txtItemZPosition, 7, 0, 2, 1, 0, 0);

		addComponent(pane, gbl, new JLabel("r00:"), 0, 1, 1, 1, 0, 0);
		addComponent(pane, gbl, txtrot00, 1, 1, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("r01:"), 3, 1, 1, 1, 0, 0);
		addComponent(pane, gbl, txtrot01, 4, 1, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("r02:"), 6, 1, 1, 1, 0, 0);
		addComponent(pane, gbl, txtrot02, 7, 1, 2, 1, 0, 0);

		addComponent(pane, gbl, new JLabel("r10:"), 0, 2, 1, 1, 0, 0);
		addComponent(pane, gbl, txtrot10, 1, 2, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("r11:"), 3, 2, 1, 1, 0, 0);
		addComponent(pane, gbl, txtrot11, 4, 2, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("r12:"), 6, 2, 1, 1, 0, 0);
		addComponent(pane, gbl, txtrot12, 7, 2, 2, 1, 0, 0);

		addComponent(pane, gbl, new JLabel("r20:"), 0, 3, 1, 1, 0, 0);
		addComponent(pane, gbl, txtrot20, 1, 3, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("r21:"), 3, 3, 1, 1, 0, 0);
		addComponent(pane, gbl, txtrot21, 4, 3, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("r22:"), 6, 3, 1, 1, 0, 0);
		addComponent(pane, gbl, txtrot22, 7, 3, 2, 1, 0, 0);

		addComponent(pane, gbl, btnPutDown, 0, 4, 12, 1, 0, 0);

		addComponent(pane, gbl, btnFarArm, 0, 5, 12, 1, 0, 0);

		addComponent(pane, gbl, btnFineArm, 0, 6, 12, 1, 0, 0);

		addComponent(pane, gbl, btnSimFarArmMovement, 0, 7, 12, 1, 0, 0);

		addComponent(pane, gbl, btnStopCmd, 0, 9, 12, 1, 0, 0);

		addComponent(pane, gbl, btnMoveHomeCmd, 0, 10, 12, 1, 0, 0);

		addComponent(pane, gbl, btnOpenGripperCmd, 0, 11, 12, 1, 0, 0);

		addComponent(pane, gbl, btnCloseGripperCmd, 0, 12, 12, 1, 0, 0);

		addComponent(pane, gbl, btnMoveArmToPosCmd, 0, 13, 12, 1, 0, 0);

		addComponent(pane, gbl, btnGetPosCmd, 0, 14, 12, 1, 0, 0);

		addComponent(pane, gbl, btnSimulatePosCmd, 0, 15, 12, 1, 0, 0);

		gui.pack();
		gui.setVisible(true);
	}

	private WorkingMemoryChangeReceiver wmcr;

	private void commandChanged(WorkingMemoryChange _wmc,
			WorkingMemoryChangeReceiver wmcr) {

		try {

			ManipulationCommand cmd = ((CogXRunner) manipulator.getRunner())
					.getMemoryEntry(_wmc.address, ManipulationCommand.class);

			Pose3 pose = null;

			if (cmd instanceof SimulateFarArmMovementCommand) {
				pose = ((SimulateFarArmMovementCommand) cmd).simulatedReachablePose;
			} else if (cmd instanceof SimulateMoveToPose) {
				pose = ((SimulateMoveToPose) cmd).simulatedReachablePose;
			} else if (cmd instanceof FarArmMovementCommand) {
				pose = ((FarArmMovementCommand) cmd).reachedPose;
			} else if (cmd instanceof MoveArmToPose) {
				pose = ((MoveArmToPose) cmd).reachedPose;
			} else if (cmd instanceof GetCurrentArmPose) {
				pose = ((GetCurrentArmPose) cmd).currentPose;
			}

			if (cmd.comp == ManipulationCompletion.SUCCEEDED && pose != null) {
				txtItemXPosition.setText(df.format(pose.pos.x));
				txtItemYPosition.setText(df.format(pose.pos.y));
				txtItemZPosition.setText(df.format(pose.pos.z));

				txtrot00.setText(df.format(pose.rot.m00));
				txtrot01.setText(df.format(pose.rot.m01));
				txtrot02.setText(df.format(pose.rot.m02));

				txtrot10.setText(df.format(pose.rot.m10));
				txtrot11.setText(df.format(pose.rot.m11));
				txtrot12.setText(df.format(pose.rot.m12));

				txtrot20.setText(df.format(pose.rot.m20));
				txtrot21.setText(df.format(pose.rot.m21));
				txtrot22.setText(df.format(pose.rot.m22));

				((CogXRunner) manipulator.getRunner()).removeChangeFilter(wmcr);
			}
		} catch (CASTException e) {
			logger.error(e);
		}
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
			logger.debug("putDown pressed");

			PutDownCommand putDownCom = new PutDownCommand();
			putDownCom.comp = ManipulationCompletion.COMPINIT;
			putDownCom.status = ManipulationCommandStatus.NEW;

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

				putDownCom.basedObjectAddr = new WorkingMemoryAddress(visObjID,
						((CogXRunner) manipulator.getRunner())
								.getSubarchitectureID());

			} else {
				Item it = manipulator.getItemMemory().getItemList().getFirst();
				try {
					WorkingMemoryAddress wma = (WorkingMemoryAddress) it
							.getAttribute(PropertyName.WMA_ADDRESS);

					putDownCom.basedObjectAddr = wma;
				} catch (ItemException e1) {
					logger.error(e1);
				}

			}

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						putDownCom);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}

		} else if (e.getActionCommand().equals("farArm")) {
			logger.debug("farArm pressed");
			FarArmMovementCommand farArmMovementCom = new FarArmMovementCommand();
			farArmMovementCom.comp = ManipulationCompletion.COMPINIT;
			farArmMovementCom.status = ManipulationCommandStatus.NEW;
			farArmMovementCom.reachedPose = new Pose3(new Vector3(0, 0, 0),
          new Matrix33(1, 0, 0, 0, 1, 0, 0, 0, 1));

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

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					commandChanged(_wmc, wmcr);
				}
			};

			((CogXRunner) manipulator.getRunner()).addChangeFilter(
					ChangeFilterFactory.createIDFilter(id), wmcr);
		} else if (e.getActionCommand().equals("fineArm")) {
			logger.debug("linGraspApp pressed");

			FineArmMovementCommand linGraspApp = new FineArmMovementCommand();
			linGraspApp.comp = ManipulationCompletion.COMPINIT;
			linGraspApp.status = ManipulationCommandStatus.NEW;
			linGraspApp.graspStatus = GraspingStatus.GRASPINGSTATUSINIT;

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

				linGraspApp.targetObjectAddr = new WorkingMemoryAddress(
						visObjID,
						((CogXRunner) manipulator.getRunner())
								.getSubarchitectureID());

			} else {
				Item it = manipulator.getItemMemory().getItemList().getFirst();
				try {
					WorkingMemoryAddress wma = (WorkingMemoryAddress) it
							.getAttribute(PropertyName.WMA_ADDRESS);

					linGraspApp.targetObjectAddr = wma;
				} catch (ItemException e1) {
					logger.error(e1);
				}

			}

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						linGraspApp);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}

		} else if (e.getActionCommand().equals("simulateFar")) {
			logger.debug("simulateGrasp pressed");

			SimulateFarArmMovementCommand simGCmd = new SimulateFarArmMovementCommand();
			simGCmd.comp = ManipulationCompletion.COMPINIT;
			simGCmd.status = ManipulationCommandStatus.NEW;
			simGCmd.simulatedReachablePose = new Pose3(new Vector3(0, 0, 0),
          new Matrix33(1, 0, 0, 0, 1, 0, 0, 0, 1));

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

				simGCmd.targetObjectAddr = new WorkingMemoryAddress(visObjID,
						((CogXRunner) manipulator.getRunner())
								.getSubarchitectureID());

			} else {
				Item it = manipulator.getItemMemory().getItemList().getFirst();
				try {
					WorkingMemoryAddress wma = (WorkingMemoryAddress) it
							.getAttribute(PropertyName.WMA_ADDRESS);

					simGCmd.targetObjectAddr = wma;
				} catch (ItemException e1) {
					logger.error(e1);
				}

			}

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						simGCmd);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					commandChanged(_wmc, wmcr);
				}
			};

			((CogXRunner) manipulator.getRunner()).addChangeFilter(
					ChangeFilterFactory.createIDFilter(id), wmcr);

		} else if (e.getActionCommand().equals("stopCmd")) {
			logger.debug("stopCmd pressed");

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			StopCommand stopCommand = new StopCommand();
			stopCommand.comp = ManipulationCompletion.COMPINIT;
			stopCommand.status = ManipulationCommandStatus.NEW;

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						stopCommand);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("moveHome")) {
			logger.debug("moveHome pressed");

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			MoveArmToHomePositionCommand moveHomeCmd = new MoveArmToHomePositionCommand();
			moveHomeCmd.comp = ManipulationCompletion.COMPINIT;
			moveHomeCmd.status = ManipulationCommandStatus.NEW;

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						moveHomeCmd);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("openGripper")) {
			logger.debug("openGripper pressed");

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			OpenGripperCommand openGripperCmd = new OpenGripperCommand();
			openGripperCmd.comp = ManipulationCompletion.COMPINIT;
			openGripperCmd.status = ManipulationCommandStatus.NEW;

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						openGripperCmd);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("closeGripper")) {
			logger.debug("closeGripper pressed");

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			CloseGripperCommand closeGripperCmd = new CloseGripperCommand();
			closeGripperCmd.comp = ManipulationCompletion.COMPINIT;
			closeGripperCmd.status = ManipulationCommandStatus.NEW;
			closeGripperCmd.graspStatus = GraspingStatus.GRASPINGSTATUSINIT;

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						closeGripperCmd);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("btnMoveArmToPosCmd")) {
			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			MoveArmToPose moveArmToPose = new MoveArmToPose();
			moveArmToPose.comp = ManipulationCompletion.COMPINIT;
			moveArmToPose.status = ManipulationCommandStatus.NEW;

			moveArmToPose.targetPose = new Pose3(new Vector3(
					Double.parseDouble(txtItemXPosition.getText()),
					Double.parseDouble(txtItemYPosition.getText()),
					Double.parseDouble(txtItemZPosition.getText())),
					new Matrix33(Double.parseDouble(txtrot00.getText()), Double
							.parseDouble(txtrot01.getText()), Double
							.parseDouble(txtrot02.getText()), Double
							.parseDouble(txtrot10.getText()), Double
							.parseDouble(txtrot11.getText()), Double
							.parseDouble(txtrot12.getText()), Double
							.parseDouble(txtrot20.getText()), Double
							.parseDouble(txtrot21.getText()), Double
							.parseDouble(txtrot22.getText())));
			moveArmToPose.reachedPose = new Pose3(new Vector3(0, 0, 0),
          new Matrix33(1, 0, 0, 0, 1, 0, 0, 0, 1));

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						moveArmToPose);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					commandChanged(_wmc, wmcr);
				}
			};

			((CogXRunner) manipulator.getRunner()).addChangeFilter(
					ChangeFilterFactory.createIDFilter(id), wmcr);
		} else if (e.getActionCommand().equals("btnGetPosCmd")) {
			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			GetCurrentArmPose getPose = new GetCurrentArmPose();
			getPose.comp = ManipulationCompletion.COMPINIT;
			getPose.status = ManipulationCommandStatus.NEW;
      getPose.currentPose = new Pose3(new Vector3(0, 0, 0),
          new Matrix33(1, 0, 0, 0, 1, 0, 0, 0, 1));

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						getPose);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					commandChanged(_wmc, wmcr);
				}
			};

			((CogXRunner) manipulator.getRunner()).addChangeFilter(
					ChangeFilterFactory.createIDFilter(id), wmcr);
		} else if (e.getActionCommand().equals("btnSimulatePosCmd")) {
			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			SimulateMoveToPose simMoveToPose = new SimulateMoveToPose();
			simMoveToPose.comp = ManipulationCompletion.COMPINIT;
			simMoveToPose.status = ManipulationCommandStatus.NEW;

			simMoveToPose.targetPose = new Pose3(new Vector3(
					Double.parseDouble(txtItemXPosition.getText()),
					Double.parseDouble(txtItemYPosition.getText()),
					Double.parseDouble(txtItemZPosition.getText())),
					new Matrix33(Double.parseDouble(txtrot00.getText()), Double
							.parseDouble(txtrot01.getText()), Double
							.parseDouble(txtrot02.getText()), Double
							.parseDouble(txtrot10.getText()), Double
							.parseDouble(txtrot11.getText()), Double
							.parseDouble(txtrot12.getText()), Double
							.parseDouble(txtrot20.getText()), Double
							.parseDouble(txtrot21.getText()), Double
							.parseDouble(txtrot22.getText())));
			simMoveToPose.simulatedReachablePose = new Pose3(new Vector3(0, 0, 0),
          new Matrix33(1, 0, 0, 0, 1, 0, 0, 0, 1));

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						simMoveToPose);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					commandChanged(_wmc, wmcr);
				}
			};

			((CogXRunner) manipulator.getRunner()).addChangeFilter(
					ChangeFilterFactory.createIDFilter(id), wmcr);
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
