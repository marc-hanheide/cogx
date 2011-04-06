package manipulation.visualisation;

import java.awt.Component;
import java.awt.Container;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;

import manipulation.core.share.Manipulator;
import manipulation.runner.cogx.CogXRunner;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.LinearBaseMovementApproachCommand;
import manipulation.slice.LinearGraspApproachCommand;
import manipulation.slice.PutDownCommand;
import manipulation.slice.SimulateGraspCommand;

import org.apache.log4j.Logger;

import cast.AlreadyExistsOnWMException;

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

	private JButton btnPutDown;
	private JButton btnFarArm;
	private JButton btnLinGraspApp;
	private JButton btnSimGrasp;
	private JButton btnLinBaseApp;

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

		// cont, gbl, comp, x, y, width, height, weightx, weighty
		addComponent(pane, gbl, btnPutDown, 0, 0, 12, 1, 0, 0);
		addComponent(pane, gbl, btnFarArm, 0, 1, 12, 1, 0, 0);
		addComponent(pane, gbl, btnLinGraspApp, 0, 2, 12, 1, 0, 0);
		addComponent(pane, gbl, btnSimGrasp, 0, 3, 12, 1, 0, 0);
		addComponent(pane, gbl, btnLinBaseApp, 0, 4, 12, 1, 0, 0);

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

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			PutDownCommand putDownCommand = new PutDownCommand();
			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						putDownCommand);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("farArm")) {
			logger.error("farArm pressed");

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			FarArmMovementCommand farArmMovementCom = new FarArmMovementCommand();
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

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			SimulateGraspCommand simulateGraspCommand = new SimulateGraspCommand();
			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						simulateGraspCommand);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("linBaseApp")) {
			logger.error("linBaseApp pressed");

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			LinearBaseMovementApproachCommand linBaseMovCommand = new LinearBaseMovementApproachCommand();
			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						linBaseMovCommand);
			} catch (AlreadyExistsOnWMException e1) {
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
