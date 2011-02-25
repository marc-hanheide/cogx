package manipulation.visualisation;

import java.awt.Component;
import java.awt.Container;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.LinkedList;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.CalibrationException;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.exceptions.MathException;
import manipulation.core.share.types.BorderPoint;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Region;
import manipulation.core.share.types.Vector2D;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.ViewPoint;
import manipulation.core.share.types.ViewPoints;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.ItemMemory;
import manipulation.itemMemory.Item.ItemIntention;
import manipulation.itemMemory.Item.ItemName;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.math.MathOperation;

import org.apache.log4j.Logger;

/**
 * GUI to use for the calibration procedure
 * 
 * @author ttoenige
 * 
 */
public class CalibrationGUI extends JPanel implements ActionListener {
	
	private Logger logger = Logger.getLogger(this.getClass());
	
	private static final long serialVersionUID = 1489792232139115240L;

	private Manipulator manipulator;
	private ItemMemory memory;

	private JTextField armXPosition;
	private JTextField armYPosition;
	private JTextField armZPosition;
	private JButton btnMoveArmTo;

	private JButton btnRecognize;
	private JButton btnResetTracker;
	private JButton btnStopTracking;

	private JButton btnGetPairs;

	private JButton btnResetAll;
	private JButton btnResetLast;

	private JButton btnCalculateCalibration;

	private JButton btngetCalibRotation;
	private JButton btngetCalibTranslation;

	private JButton btnConvertCamToRob;

	/**
	 * constructor for the calibration GUI, displays the GUI and can be used
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param mem
	 *            corresponding item memory
	 */
	public CalibrationGUI(Manipulator manipulator, ItemMemory mem) {
		this.manipulator = manipulator;
		this.memory = mem;

		manipulator.getBaseConnector().setMoving(true);

		guiSetup();
	}

	/**
	 * constructor to show the GUI without any functionality
	 */
	public CalibrationGUI() {
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
		JFrame gui = new JFrame("Learning GUI");
		gui.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		gui.setLocation(100, 200);

		Container pane = gui.getContentPane();
		GridBagLayout gbl = new GridBagLayout();
		pane.setLayout(gbl);

		armXPosition = new JTextField("", 7);
		armYPosition = new JTextField("", 7);
		armZPosition = new JTextField("", 7);
		btnMoveArmTo = new JButton("move arm to (x,y,z)");
		btnMoveArmTo.setActionCommand("btnMoveArmTo");
		btnMoveArmTo.addActionListener(this);

		btnRecognize = new JButton("Recognize");
		btnRecognize.setActionCommand("btnRecognize");
		btnRecognize.addActionListener(this);
		btnResetTracker = new JButton("Reset Tracker");
		btnResetTracker.setActionCommand("btnResetTracker");
		btnResetTracker.addActionListener(this);
		btnStopTracking = new JButton("Stop Tracking");
		btnStopTracking.setActionCommand("btnStopTracking");
		btnStopTracking.addActionListener(this);

		btnGetPairs = new JButton("get pair coordinates");
		btnGetPairs.setActionCommand("getPair");
		btnGetPairs.addActionListener(this);

		btnResetAll = new JButton("reset all pairs");
		btnResetAll.setActionCommand("btnResetAll");
		btnResetAll.addActionListener(this);
		btnResetLast = new JButton("delete last pair");
		btnResetLast.setActionCommand("btnResetLast");
		btnResetLast.addActionListener(this);

		btnCalculateCalibration = new JButton("calculate");
		btnCalculateCalibration.setActionCommand("btnCalculateCalibration");
		btnCalculateCalibration.addActionListener(this);

		btngetCalibRotation = new JButton("get calibrated rotation");
		btngetCalibRotation.setActionCommand("btngetCalibRotation");
		btngetCalibRotation.addActionListener(this);
		btngetCalibTranslation = new JButton("get calibrated translation");
		btngetCalibTranslation.setActionCommand("btngetCalibTranslation");
		btngetCalibTranslation.addActionListener(this);

		btnConvertCamToRob = new JButton("convert cam to roboter");
		btnConvertCamToRob.setActionCommand("btnConvertCamToRob");
		btnConvertCamToRob.addActionListener(this);

		// cont, gbl, comp, x, y, width, height, weightx, weighty
		addComponent(pane, gbl, new JLabel("x:"), 0, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, armXPosition, 2, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("y:"), 4, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, armYPosition, 6, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("z:"), 8, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, armZPosition, 10, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, btnMoveArmTo, 12, 0, 2, 1, 0, 0);

		addComponent(pane, gbl, btnRecognize, 0, 1, 5, 1, 0, 0);
		addComponent(pane, gbl, btnResetTracker, 5, 1, 5, 1, 0, 0);
		addComponent(pane, gbl, btnStopTracking, 10, 1, 4, 1, 0, 0);

		addComponent(pane, gbl, btnGetPairs, 0, 2, 14, 1, 0, 0);

		addComponent(pane, gbl, btnResetAll, 0, 3, 7, 1, 0, 0);
		addComponent(pane, gbl, btnResetLast, 7, 3, 7, 1, 0, 0);

		addComponent(pane, gbl, btnCalculateCalibration, 0, 4, 14, 1, 0, 0);

		addComponent(pane, gbl, btngetCalibRotation, 0, 5, 7, 1, 0, 0);
		addComponent(pane, gbl, btngetCalibTranslation, 7, 5, 7, 1, 0, 0);

		addComponent(pane, gbl, btnConvertCamToRob, 0, 6, 14, 1, 0, 0);

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
		if (e.getActionCommand().equals("getPair")) {
			logger.debug("button get pair pressed");
			try {
				Vector3D visionPoint = ((Vector3D) memory.getFirstGraspItem()
						.getAttribute(PropertyName.ITEM_IN_CAM_POSITION));
				Vector3D armPoint = manipulator.getArmConnector()
						.getCurrentPosition();
				manipulator.getCalibrationConnector().addCamToRobPair(
						visionPoint, armPoint);
			} catch (ItemException e1) {
				logger.error(e1);
			} catch (InternalMemoryException e1) {
				logger.error(e1);
			} catch (ManipulatorException e2) {
				logger.error(e2);
			}
		} else if (e.getActionCommand().equals("btnRecognize")) {
			logger.debug("pressed btnRecognize");
			Item item = new Item();
			item.setAttribute(PropertyName.NAME, ItemName.FROSTIES_SMALL);
			item.setAttribute(PropertyName.WORLD_POSITION,
					new Vector3D(1, 1, 1));
			item.setAttribute(PropertyName.WORLD_ROTATION, new Matrix(1, 0, 0,
					0, 1, 0, 0, 0, 1));
			item.setAttribute(PropertyName.INTENTION, ItemIntention.GRASP_ME);

			List<Vector2D> fakeList = new LinkedList<Vector2D>();
			List<BorderPoint> fakeList1 = new LinkedList<BorderPoint>();
			Region fakeRegion = new Region(fakeList, fakeList1);

			item.setAttribute(PropertyName.SURROUNDING, fakeRegion);

			List<ViewPoint> fakeList3 = new LinkedList<ViewPoint>();

			ViewPoints vps = new ViewPoints(fakeList3);
			item.setAttribute(PropertyName.VIEW_POINTS, vps);

			item.setAttribute(PropertyName.BEST_VIEW_POINT, manipulator
					.getMapAlgorithms().getBestViewPoint(vps.getPoints()));

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
		} else if (e.getActionCommand().equals("btnMoveArmTo")) {
			logger.debug("pressed btnMoveArmTo");

			Matrix rotation = MathOperation.getRotationAroundX(-Math.PI / 4);
			try {
				manipulator.getArmConnector().reach(
						new Vector3D(
								Double.parseDouble(armXPosition.getText()),
								Double.parseDouble(armYPosition.getText()),
								Double.parseDouble(armZPosition.getText())),
						rotation);
			} catch (NumberFormatException e1) {
				logger.error(e1);
			} catch (ManipulatorException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("btnResetAll")) {
			logger.debug("pressed btnResetAll");
			manipulator.getCalibrationConnector().resetCamToRobAllPairs();
		} else if (e.getActionCommand().equals("btnResetLast")) {
			logger.debug("pressed btnResetLast");
			manipulator.getCalibrationConnector().resetCamToRobLastPair();
		} else if (e.getActionCommand().equals("btnCalculateCalibration")) {
			logger.debug("pressed btnCalculateCalibration");

			// manipulator.getCalibrationConnector().testCamToRobAdd();

			try {
				manipulator.getCalibrationConnector()
						.calculateCamToRobCalibration();
			} catch (CalibrationException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("btngetCalibRotation")) {
			logger.debug("pressed btngetCalibRotation");
			try {
				logger.error(manipulator.getCalibrationConnector()
						.getCamToRobRotation());
			} catch (CalibrationException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("btngetCalibTranslation")) {
			logger.debug("pressed btngetCalibTranslation");
			try {
				logger.error(manipulator.getCalibrationConnector()
						.getCamToRobTranslation());
			} catch (CalibrationException e1) {
				logger.error(e1);
			}

		} else if (e.getActionCommand().equals("btnConvertCamToRob")) {
			logger.debug("pressed btnConvertCamToRob");

			Vector3D visionPoint;
			try {
				visionPoint = ((Vector3D) memory.getFirstGraspItem()
						.getAttribute(PropertyName.ITEM_IN_CAM_POSITION));

				Vector3D newTranslation = manipulator.getCalibrationConnector()
						.getCamPointInRob(visionPoint);

				logger.error("Calculated robot coordinates: " + newTranslation);
			} catch (ItemException e1) {
				logger.error(e1);
			} catch (InternalMemoryException e1) {
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
		new CalibrationGUI();
	}

}
