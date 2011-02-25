package manipulation.visualisation;

import java.awt.Component;
import java.awt.Container;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.HashMap;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.exceptions.MathException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Region;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.SensorData.SensorPosition;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.ItemMemory;
import manipulation.itemMemory.Item.ItemIntention;
import manipulation.itemMemory.Item.ItemName;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.math.MathOperation;
import manipulation.strategies.Strategy.Name;

import org.apache.log4j.Logger;

/**
 * GUI to execute the task and to trigger some basic functionalities
 * 
 * @author ttoenige
 * 
 */
public class ExecutionGUI extends JPanel implements ActionListener {

	private Logger logger = Logger.getLogger(this.getClass());

	private static final long serialVersionUID = -3602880992645130049L;

	private Manipulator manipulator;
	private ItemMemory itemMemory;

	private JTextField txtGoToX;
	private JTextField txtGoToY;
	private JTextField txtGoToAngle;
	private JButton btnGoToWait;
	private JButton btnGoToNotWait;

	private JTextField txtItemXPosition;
	private JTextField txtItemYPosition;
	private JTextField txtItemZPosition;
	private JTextField txtItemName;
	private JButton btnAddToItemMem;
	private JButton btnDeleteItem;

	private JButton btnGetMap;
	private JButton btnUpdatePositions;

	private JButton btnRecognize;
	private JButton btnResetTracker;
	private JButton btnStopTracking;

	private JButton btnArmMoveGlobal;
	private JButton btnArmMoveStep;
	private JButton btnArmMoveStopReactive;
	private JButton btnArmGoHome;

	private JButton btnArmGetPos;
	private JButton btnArmGetRot;

	private JButton btnExecuteMobileStrategy;
	private JButton btnExecuteCalibrationStrategy;

	private JButton btnCloseGripper;
	private JButton btnOpenGripper;
	private JButton btnFreezeGripper;

	private JButton btnTest;

	private MapPanel mapPanel;

	private double x = -0.2;

	/**
	 * Constructor of the GUI
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 */
	public ExecutionGUI(Manipulator manipulator) {
		this.manipulator = manipulator;
		guiSetup();
	}

	/**
	 * constructor to show the GUI without any functionality
	 */
	public ExecutionGUI() {
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
		JFrame gui = new JFrame("Execution Window");
		gui.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		gui.setLocation(100, 200);

		Container pane = gui.getContentPane();
		GridBagLayout gbl = new GridBagLayout();
		pane.setLayout(gbl);

		txtGoToX = new JTextField("", 7);
		txtGoToY = new JTextField("", 7);
		txtGoToAngle = new JTextField("", 7);
		btnGoToWait = new JButton("stop");
		btnGoToWait.setActionCommand("btnGoToWait");
		btnGoToWait.addActionListener(this);
		btnGoToNotWait = new JButton("goTo not wait");
		btnGoToNotWait.setActionCommand("btnGoToNotWait");
		btnGoToNotWait.addActionListener(this);

		txtItemXPosition = new JTextField("", 7);
		txtItemYPosition = new JTextField("", 7);
		txtItemZPosition = new JTextField("", 7);
		txtItemName = new JTextField("", 10);
		btnAddToItemMem = new JButton("add");
		btnAddToItemMem.setActionCommand("btnAddToItemMem");
		btnAddToItemMem.addActionListener(this);
		btnDeleteItem = new JButton("delete first grasp Item");
		btnDeleteItem.setActionCommand("btnDeleteItem");
		btnDeleteItem.addActionListener(this);

		btnGetMap = new JButton("Get Map");
		btnGetMap.setActionCommand("btnGetMap");
		btnGetMap.addActionListener(this);

		btnUpdatePositions = new JButton("Update Positions");
		btnUpdatePositions.setActionCommand("btnUpdatePositions");
		btnUpdatePositions.addActionListener(this);

		btnRecognize = new JButton("Recognize");
		btnRecognize.setActionCommand("btnRecognize");
		btnRecognize.addActionListener(this);
		btnResetTracker = new JButton("Reset Tracker");
		btnResetTracker.setActionCommand("btnResetTracker");
		btnResetTracker.addActionListener(this);
		btnStopTracking = new JButton("Stop Tracking");
		btnStopTracking.setActionCommand("btnStopTracking");
		btnStopTracking.addActionListener(this);

		btnArmMoveGlobal = new JButton("ArmAutoTest1");
		btnArmMoveGlobal.setActionCommand("btnArmMoveGlobal");
		btnArmMoveGlobal.addActionListener(this);
		btnArmMoveStep = new JButton("ArmAutoTest2");
		btnArmMoveStep.setActionCommand("btnArmMoveStep");
		btnArmMoveStep.addActionListener(this);
		btnArmMoveStopReactive = new JButton("Stop Reactive");
		btnArmMoveStopReactive.setActionCommand("btnArmMoveStopReactive");
		btnArmMoveStopReactive.addActionListener(this);
		btnArmGoHome = new JButton("Go Home");
		btnArmGoHome.setActionCommand("btnArmGoHome");
		btnArmGoHome.addActionListener(this);

		btnArmGetPos = new JButton("Get Position");
		btnArmGetPos.setActionCommand("btnArmGetPos");
		btnArmGetPos.addActionListener(this);
		btnArmGetRot = new JButton("Get Rotation");
		btnArmGetRot.setActionCommand("btnArmGetRot");
		btnArmGetRot.addActionListener(this);

		btnExecuteMobileStrategy = new JButton("Execute mobile Manipulation");
		btnExecuteMobileStrategy.setActionCommand("execMobMan");
		btnExecuteMobileStrategy.addActionListener(this);
		btnExecuteCalibrationStrategy = new JButton(
				"Execute Calibration procedure");
		btnExecuteCalibrationStrategy.setActionCommand("execCalib");
		btnExecuteCalibrationStrategy.addActionListener(this);

		btnCloseGripper = new JButton("Close Gripper");
		btnCloseGripper.setActionCommand("btnCloseGripper");
		btnCloseGripper.addActionListener(this);
		btnOpenGripper = new JButton("Open Gripper");
		btnOpenGripper.setActionCommand("btnOpenGripper");
		btnOpenGripper.addActionListener(this);
		btnFreezeGripper = new JButton("Freeze Gripper");
		btnFreezeGripper.setActionCommand("btnFreezeGripper");
		btnFreezeGripper.addActionListener(this);

		btnTest = new JButton("Test whatever");
		btnTest.setActionCommand("btnTest");
		btnTest.addActionListener(this);

		// cont, gbl, comp, x, y, width, height, weightx, weighty
		addComponent(pane, gbl, new JLabel("Goto Position:"), 0, 0, 10, 1, 0, 0);

		addComponent(pane, gbl, new JLabel("x:"), 0, 1, 1, 1, 0, 0);
		addComponent(pane, gbl, txtGoToX, 1, 1, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("y:"), 3, 1, 1, 1, 0, 0);
		addComponent(pane, gbl, txtGoToY, 4, 1, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("angle:"), 6, 1, 1, 1, 0, 0);
		addComponent(pane, gbl, txtGoToAngle, 7, 1, 2, 1, 0, 0);
		addComponent(pane, gbl, btnGoToWait, 9, 1, 2, 1, 0, 0);
		addComponent(pane, gbl, btnGoToNotWait, 11, 1, 2, 1, 0, 0);

		addComponent(pane, gbl, new JLabel("Add Item to Object memory:"), 0, 2,
				10, 1, 0, 0);

		addComponent(pane, gbl, new JLabel("x:"), 0, 3, 1, 1, 0, 0);
		addComponent(pane, gbl, txtItemXPosition, 1, 3, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("y:"), 3, 3, 1, 1, 0, 0);
		addComponent(pane, gbl, txtItemYPosition, 4, 3, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("z:"), 6, 3, 1, 1, 0, 0);
		addComponent(pane, gbl, txtItemZPosition, 7, 3, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("name:"), 0, 4, 3, 1, 0, 0);
		addComponent(pane, gbl, txtItemName, 3, 4, 6, 1, 0, 0);
		addComponent(pane, gbl, btnAddToItemMem, 9, 4, 2, 1, 0, 0);
		addComponent(pane, gbl, btnDeleteItem, 11, 4, 2, 1, 0, 0);

		addComponent(pane, gbl, btnGetMap, 0, 5, 5, 1, 0, 0);
		addComponent(pane, gbl, btnUpdatePositions, 5, 5, 5, 1, 0, 0);

		addComponent(pane, gbl, btnRecognize, 0, 6, 5, 1, 0, 0);
		addComponent(pane, gbl, btnResetTracker, 5, 6, 5, 1, 0, 0);
		addComponent(pane, gbl, btnStopTracking, 10, 6, 2, 1, 0, 0);

		addComponent(pane, gbl, btnArmMoveGlobal, 0, 7, 5, 1, 0, 0);
		addComponent(pane, gbl, btnArmMoveStep, 5, 7, 5, 1, 0, 0);
		addComponent(pane, gbl, btnArmMoveStopReactive, 10, 7, 2, 1, 0, 0);
		addComponent(pane, gbl, btnArmGoHome, 12, 7, 2, 1, 0, 0);

		addComponent(pane, gbl, btnArmGetPos, 0, 8, 2, 1, 0, 0);
		addComponent(pane, gbl, btnArmGetRot, 2, 8, 2, 1, 0, 0);

		addComponent(pane, gbl, btnExecuteMobileStrategy, 0, 9, 5, 1, 0, 0);
		addComponent(pane, gbl, btnExecuteCalibrationStrategy, 5, 9, 5, 1, 0, 0);

		addComponent(pane, gbl, btnCloseGripper, 0, 10, 5, 1, 0, 0);
		addComponent(pane, gbl, btnOpenGripper, 5, 10, 5, 1, 0, 0);
		addComponent(pane, gbl, btnFreezeGripper, 10, 10, 2, 1, 0, 0);

		addComponent(pane, gbl, btnTest, 0, 11, 12, 1, 0, 0);

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
		// TODO Fehlerabfang bei Eingabe
		if (e.getActionCommand().equals("btnGoToWait")) {
			manipulator.getBaseConnector().stop();
		} else if (e.getActionCommand().equals("btnGoToNotWait")) {
			logger.debug("pressed goTo Button");
			logger.debug("Input: x: " + txtGoToX.getText() + " y: "
					+ txtGoToY.getText() + " angle: " + txtGoToAngle.getText());

			BasePositionData baseNavData = new BasePositionData(Double
					.parseDouble(txtGoToX.getText()), Double
					.parseDouble(txtGoToY.getText()), Double
					.parseDouble(txtGoToAngle.getText()));
			try {

				manipulator.getBaseConnector().goTo(baseNavData);
			} catch (ExternalMemoryException e1) {
				logger.error(e1);
			}

		} else if (e.getActionCommand().equals("btnAddToItemMem")) {

			logger.debug("pressed addToMem Button");
			logger.debug("Input: x: " + txtItemXPosition.getText() + " y: "
					+ txtItemYPosition.getText() + " z: "
					+ txtItemZPosition.getText() + " name: "
					+ txtItemName.getText());

			if (txtItemName.getText().equals("FROSTIES_SMALL")) {
				Vector3D translationData = new Vector3D(Double
						.parseDouble(txtItemXPosition.getText()), Double
						.parseDouble(txtItemYPosition.getText()), Double
						.parseDouble(txtItemZPosition.getText()));

				Matrix rotationData = new Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);

				try {
					Item item = new Item();
					item.setAttribute(PropertyName.NAME,
							ItemName.FROSTIES_SMALL);
					item.setAttribute(PropertyName.WORLD_POSITION,
							translationData);
					item
							.setAttribute(PropertyName.WORLD_ROTATION,
									rotationData);
					item.setAttribute(PropertyName.INTENTION,
							ItemIntention.GRASP_ME);

					Region surrounding = manipulator.getMapAlgorithms()
							.updateSurroundingFromWorldCoordinates(item);
					item.setAttribute(PropertyName.SURROUNDING, surrounding);

				} catch (MathException e1) {

					logger.error(e1);
				} catch (ItemException e2) {
					// TODO Auto-generated catch block
					e2.printStackTrace();
				}

			} else {
				logger.error("unknown name");
			}
		} else if (e.getActionCommand().equals("btnGetMap")) {
			logger.debug("pressed btnGetMap Button");

			mapPanel.generateNewMap();

		} else if (e.getActionCommand().equals("btnUpdatePositions")) {
			logger.debug("pressed btnUpdatePositions Button");

			mapPanel.repaint();

		} else if (e.getActionCommand().equals("btnRecognize")) {
			logger.debug("pressed btnRecognize");
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
				logger.error(e1);
			}

		} else if (e.getActionCommand().equals("btnStopTracking")) {
			logger.debug("pressed btnStopTracking");

			try {
				manipulator.getCamConnector().stopTracking();
			} catch (ExternalMemoryException e1) {
				logger.error(e);
			}

		} else if (e.getActionCommand().equals("btnResetTracker")) {
			logger.debug("pressed btnResetTracker");

			try {
				manipulator.getCamConnector().resetTracker();
			} catch (ExternalMemoryException e1) {
				logger.error(e1);
			}

		} else if (e.getActionCommand().equals("btnArmMoveGlobal")) {
			logger.debug("pressed btnArmMoveGlobal");

			try {
				Matrix rotation1 = MathOperation
						.getRotationAroundX(MathOperation.getRadiant(Double
								.parseDouble(txtGoToX.getText())));

				Matrix rotation2 = MathOperation
						.getRotationAroundY(MathOperation.getRadiant(Double
								.parseDouble(txtGoToY.getText())));

				Matrix rotation3 = MathOperation
						.getRotationAroundZ(MathOperation.getRadiant(Double
								.parseDouble(txtGoToAngle.getText())));

				Matrix result = MathOperation.getMatrixMatrixMultiplication(
						MathOperation.getMatrixMatrixMultiplication(rotation1,
								rotation2), rotation3);

				manipulator.getArmConnector()
						.reach(
								new Vector3D(
										Double.parseDouble(txtItemXPosition
												.getText()), Double
												.parseDouble(txtItemYPosition
														.getText()), Double
												.parseDouble(txtItemZPosition
														.getText())), result);

			} catch (ManipulatorException e1) {
				logger.error(e1);
			}

			x += 0.1;

		} else if (e.getActionCommand().equals("btnArmMoveStep")) {
			logger.debug("pressed btnArmMoveStep");

			Matrix rotation = MathOperation.getRotationAroundX(-Math.PI / 2);
			try {
				manipulator.getArmConnector()
						.reach(
								new Vector3D(
										Double.parseDouble(txtItemXPosition
												.getText()), Double
												.parseDouble(txtItemYPosition
														.getText()), Double
												.parseDouble(txtItemZPosition
														.getText())), rotation);
			} catch (NumberFormatException e1) {
				logger.error(e1);
			} catch (ManipulatorException e1) {
				logger.error(e1);
			}

		} else if (e.getActionCommand().equals("btnArmMoveStopReactive")) {
			logger.debug("pressed btnArmMoveStopReactive");

			try {
				manipulator.getArmConnector().stopArm();
			} catch (ManipulatorException e1) {
				logger.error(e1);
			}

		} else if (e.getActionCommand().equals("btnArmGoHome")) {
			logger.debug("pressed btnArmGoHome");
			try {
				manipulator.getArmConnector().goHome();
			} catch (ManipulatorException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("btnDeleteItem")) {
			try {
				itemMemory.deleteItem(itemMemory.getFirstGraspItem());
			} catch (InternalMemoryException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
		} else if (e.getActionCommand().equals("btnArmGetPos")) {
			logger.debug("pressed btnArmGetPos");
			try {
				logger
						.debug(manipulator.getArmConnector()
								.getCurrentPosition());
			} catch (ManipulatorException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("btnArmGetRot")) {
			logger.debug("pressed btnArmGetRot");
			try {
				logger
						.debug(manipulator.getArmConnector()
								.getCurrentRotation());
			} catch (ManipulatorException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("execMobMan")) {
			logger.debug("pressed btnExecuteMobileStrategy");
			// TODO nicht so toll hier nen thread zu starten
			Thread t = new Thread(new Runnable() {
				@Override
				public void run() {
					manipulator.getRunner().startStrategy(
							Name.MOBILE_MANIPULATION_NEW);
				}
			});
			t.start();
			mapPanel = new MapPanel(manipulator);
		} else if (e.getActionCommand().equals("execCalib")) {
			logger.debug("pressed btnExecuteCalibration");
			// TODO nicht so toll hier nen thread zu starten
			Thread t = new Thread(new Runnable() {
				@Override
				public void run() {
					manipulator.getRunner().startStrategy(Name.CALIBRATION);
				}
			});
			t.start();

		} else if (e.getActionCommand().equals("btnCloseGripper")) {
			logger.debug("pressed btnCloseGripper");
			manipulator.getArmConnector().closeGripper(7);
		} else if (e.getActionCommand().equals("btnOpenGripper")) {
			logger.debug("pressed btnOpenGripper");
			manipulator.getArmConnector().openGripper();
		} else if (e.getActionCommand().equals("btnFreezeGripper")) {
			logger.debug("pressed btnFreezeGripper");
			manipulator.getArmConnector().freezeGripper();
		} else if (e.getActionCommand().equals("btnTest")) {
			logger.debug("pressed btnTest");

			// manipulator.getBaseConnector().startExplore();

			logger.error(manipulator.getArmConnector().isGraspingObject());

			HashMap<SensorPosition, Integer> sensors = manipulator
					.getArmConnector().receiveGripperSensorData();

			logger.error(sensors.get(SensorPosition.INFRARED_MIDDLE));
		}

	}

	/**
	 * shows the gui without any functionality
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		new ExecutionGUI();
	}

}
