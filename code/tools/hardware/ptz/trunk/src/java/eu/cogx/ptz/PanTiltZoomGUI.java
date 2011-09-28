package eu.cogx.ptz;

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

import org.apache.log4j.Logger;

import ptz.GetPTZPoseCommand;
import ptz.PTZCompletion;
import ptz.PTZPose;
import ptz.SetPTZPoseCommand;
import cast.AlreadyExistsOnWMException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;

/**
 * GUI to use for the calibration procedure
 * 
 * @author Torben Toeniges
 * 
 */
public class PanTiltZoomGUI extends JPanel implements ActionListener {

	private Logger logger = Logger.getLogger(this.getClass());

	private static final long serialVersionUID = 1489792232139115240L;

	private PanTiltZoomServer server;

	private JTextField txtPan;
	private JTextField txtTilt;
	private JTextField txtZoom;

	private JButton btnMovePTZ;

	private JButton btnGetPTZValue;

	private DecimalFormat df = new DecimalFormat("0.00");

	public PanTiltZoomGUI(PanTiltZoomServer server) {
		this.server = server;

		guiSetup();
	}

	/**
	 * constructor to show the GUI without any functionality
	 */
	public PanTiltZoomGUI() {
		guiSetup();
	}

	private enum ComType {
		GET, MOVE
	}

	private WorkingMemoryChangeReceiver wmcr;

	private void commandChanged(WorkingMemoryChange _wmc,
			WorkingMemoryChangeReceiver wmcr, ComType type) {

		try {
			if (type == ComType.GET) {
				GetPTZPoseCommand cmd = server.getMemoryEntry(_wmc.address,
						GetPTZPoseCommand.class);

				if (cmd.comp == PTZCompletion.SUCCEEDED) {
					txtPan.setText(df.format((cmd.pose.pan/Math.PI)*180));
					txtTilt.setText(df.format((cmd.pose.tilt/Math.PI)*180));
					txtZoom.setText(df.format(cmd.pose.zoom));

					logger.debug("get ptz position successful");
				}
			} else if (type == ComType.MOVE) {
				SetPTZPoseCommand cmd = server.getMemoryEntry(_wmc.address,
						SetPTZPoseCommand.class);

				if (cmd.comp == PTZCompletion.SUCCEEDED) {
					txtPan.setText(df.format((cmd.pose.pan/Math.PI)*180));
					txtTilt.setText(df.format((cmd.pose.tilt/Math.PI)*180));
					txtZoom.setText(df.format(cmd.pose.zoom));

					logger.debug("set ptz position successful");
				}
			}

			server.removeChangeFilter(wmcr);
		} catch (SubarchitectureComponentException e) {
			logger.error(e);
		}
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
		JFrame gui = new JFrame("PTZ Command Test GUI");
		gui.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		gui.setLocation(100, 200);

		Container pane = gui.getContentPane();
		GridBagLayout gbl = new GridBagLayout();
		pane.setLayout(gbl);

		txtPan = new JTextField("", 7);
		txtTilt = new JTextField("", 7);
		txtZoom = new JTextField("0", 7);

		btnMovePTZ = new JButton("move the PTZ to the define value");
		btnMovePTZ.setActionCommand("move");
		btnMovePTZ.addActionListener(this);

		btnGetPTZValue = new JButton("get PTZ value");
		btnGetPTZValue.setActionCommand("get");
		btnGetPTZValue.addActionListener(this);

		// cont, gbl, comp, x, y, width, height, weightx, weighty
		addComponent(pane, gbl, new JLabel("pan:"), 0, 0, 1, 1, 0, 0);
		addComponent(pane, gbl, txtPan, 1, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("tilt:"), 3, 0, 1, 1, 0, 0);
		addComponent(pane, gbl, txtTilt, 4, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("zoom:"), 6, 0, 1, 1, 0, 0);
		addComponent(pane, gbl, txtZoom, 7, 0, 2, 1, 0, 0);

		addComponent(pane, gbl, btnMovePTZ, 0, 1, 12, 1, 0, 0);

		addComponent(pane, gbl, btnGetPTZValue, 0, 2, 12, 1, 0, 0);

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
		if (e.getActionCommand().equals("move")) {
			SetPTZPoseCommand cmd = new SetPTZPoseCommand();



			PTZPose pose = new PTZPose(
					Double.parseDouble(txtPan.getText())/180*Math.PI,
					Double.parseDouble(txtTilt.getText())/180*Math.PI,
					Double.parseDouble(txtZoom.getText()) 
					);

			cmd.pose = pose;
			cmd.comp = PTZCompletion.COMPINIT;

			String id = server.newDataID();

			try {
				server.addToWorkingMemory(id, cmd);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e);
			}

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					commandChanged(_wmc, wmcr, ComType.MOVE);
				}
			};

			server.addChangeFilter(ChangeFilterFactory.createIDFilter(id), wmcr);

		} else if (e.getActionCommand().equals("get")) {
			GetPTZPoseCommand cmd = new GetPTZPoseCommand();
			cmd.comp = PTZCompletion.COMPINIT;
			cmd.pose = new PTZPose(Double.MAX_VALUE, Double.MAX_VALUE,
					Double.MAX_VALUE);

			String id = server.newDataID();

			try {
				server.addToWorkingMemory(id, cmd);
			} catch (AlreadyExistsOnWMException e1) {
				logger.error(e);
			}

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					commandChanged(_wmc, wmcr, ComType.GET);
				}
			};

			server.addChangeFilter(ChangeFilterFactory.createIDFilter(id), wmcr);

		}
	}

	/**
	 * main function to show the GUI without any functionality
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		new PanTiltZoomGUI();
	}

}
