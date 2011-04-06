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
import manipulation.slice.GraspCommand;
import manipulation.slice.PutDownCommand;

import org.apache.log4j.Logger;

import cogx.Math.Matrix33;
import cogx.Math.Pose3;
import cogx.Math.Vector3;

import VisionData.VisualObject;

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

	private JButton btnGrasp;

	private JButton btnPutDown;

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

		btnGrasp = new JButton("grasp command");
		btnGrasp.setActionCommand("grasp");
		btnGrasp.addActionListener(this);

		btnPutDown = new JButton("put down command");
		btnPutDown.setActionCommand("putDown");
		btnPutDown.addActionListener(this);

		// cont, gbl, comp, x, y, width, height, weightx, weighty
		addComponent(pane, gbl, btnGrasp, 0, 0, 12, 1, 0, 0);
		addComponent(pane, gbl, btnPutDown, 0, 1, 12, 1, 0, 0);

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
		if (e.getActionCommand().equals("grasp")) {
			String id = ((CogXRunner) manipulator.getRunner()).newDataID();

			GraspCommand graspCommand = new GraspCommand();
			VisualObject vo = new VisualObject();
			Pose3 pos = new Pose3(new Vector3(0, 1, 2), new Matrix33());
			vo.pose = pos;

			graspCommand.targetObject = vo;

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						graspCommand);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e1);
			}
		} else if (e.getActionCommand().equals("putDown")) {
			String id = ((CogXRunner) manipulator.getRunner()).newDataID();

			PutDownCommand putDownCommand = new PutDownCommand();

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						putDownCommand);
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
