package spatial.manual;

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

import org.apache.log4j.Logger;

import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Priority;
import SpatialData.StatusError;
import manipulation.slice.ManipulationPose;
import cast.AlreadyExistsOnWMException;
import cast.architecture.ManagedComponent;
import cast.core.CASTData;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;

/**
 * GUI to use for the calibration procedure
 * 
 * @author Torben Toeniges
 * 
 */
public class ManualNavGUI extends JPanel implements ActionListener {

	private Logger logger = Logger.getLogger(this.getClass());

	private static final long serialVersionUID = 1489792232139115240L;

	private ManagedComponent component;

	private JTextField txtXPosition;
	private JTextField txtYPosition;
	private JTextField txtThetaPosition;

	private JTextField txtPlaceID;
	private JTextField txtPoseID;

	private JButton btnGotoPos;
	private JButton btnGotoGPose;
	private JButton btnGotoPlace;

	/**
	 * constructor for the cogx test GUI, displays the GUI and can be used test
	 * the implemented planning actions
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param mem
	 *            corresponding item memory
	 */
	public ManualNavGUI(ManagedComponent component) {
		this.component = component;

		guiSetup();
	}

	/**
	 * constructor to show the GUI without any functionality
	 */
	public ManualNavGUI() {
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
		JFrame gui = new JFrame("Spatial GUI");
		gui.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);

		Container pane = gui.getContentPane();
		GridBagLayout gbl = new GridBagLayout();
		pane.setLayout(gbl);

		txtXPosition = new JTextField("0", 7);
		txtYPosition = new JTextField("0", 7);
		txtThetaPosition = new JTextField("0", 7);

		txtPlaceID = new JTextField("0", 7);
		txtPoseID = new JTextField("0:0", 7);

		btnGotoPos = new JButton("goto position");
		btnGotoPos.setActionCommand("gotoPos");
		btnGotoPos.addActionListener(this);

		btnGotoPlace = new JButton("go place");
		btnGotoPlace.setActionCommand("gotoPlace");
		btnGotoPlace.addActionListener(this);

		btnGotoGPose = new JButton("goto pose");
		btnGotoGPose.setActionCommand("gotoPose");
		btnGotoGPose.addActionListener(this);

		// cont, gbl, comp, x, y, width, height, weightx, weighty
		addComponent(pane, gbl, new JLabel("x:"), 0, 0, 1, 1, 0, 0);
		addComponent(pane, gbl, txtXPosition, 1, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("y:"), 3, 0, 1, 1, 0, 0);
		addComponent(pane, gbl, txtYPosition, 4, 0, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("Theta:"), 6, 0, 1, 1, 0, 0);
		addComponent(pane, gbl, txtThetaPosition, 7, 0, 2, 1, 0, 0);

		addComponent(pane, gbl, new JLabel("place:"), 0, 1, 1, 1, 0, 0);
		addComponent(pane, gbl, txtPlaceID, 1, 1, 2, 1, 0, 0);
		addComponent(pane, gbl, new JLabel("pose:"), 3, 1, 1, 1, 0, 0);
		addComponent(pane, gbl, txtPoseID, 4, 1, 2, 1, 0, 0);

		addComponent(pane, gbl, btnGotoPos, 0, 4, 12, 1, 0, 0);

		addComponent(pane, gbl, btnGotoPlace, 0, 5, 12, 1, 0, 0);
		addComponent(pane, gbl, btnGotoGPose, 0, 6, 12, 1, 0, 0);
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
		if (e.getActionCommand().equals("gotoPos")) {
			logger.debug("gotoPos pressed");
			NavCommand nc = createNavCommand();
			nc.cmd=CommandType.GOTOPOSITION;
			try {
				component.addToWorkingMemory(component.newDataID(), nc);
			} catch (AlreadyExistsOnWMException e1) {
				component.logException(e1);
			}

			
		} else if (e.getActionCommand().equals("gotoPose")) {
			logger.debug("gotoPose pressed");
            String id = txtPoseID.getText();
            try {
                CASTData<ManipulationPose> data = (CASTData<ManipulationPose>) component.getWorkingMemoryEntry(id, "manipulation.sa");
                ManipulationPose pose = data.getData();
                NavCommand nc = createPoseNavCommand(pose.robotPose.x, pose.robotPose.y, pose.robotPose.z);
                nc.cmd=CommandType.GOTOPOSITION;
				component.addToWorkingMemory(component.newDataID(), nc);
            } catch (DoesNotExistOnWMException e1) {
				component.logException(e1);
			} catch (AlreadyExistsOnWMException e1) {
				component.logException(e1);
			} catch (UnknownSubarchitectureException e1) {
				component.logException(e1);
            }

			
		} else if (e.getActionCommand().equals("gotoPlace")) {
			logger.debug("gotoPlace pressed");
			NavCommand nc = createNavCommand();
			nc.cmd=CommandType.GOTOPLACE;
			try {
				component.addToWorkingMemory(component.newDataID(), nc);
			} catch (AlreadyExistsOnWMException e1) {
				component.logException(e1);
			}


		}
	}

	private NavCommand createNavCommand() {
		NavCommand nc = new NavCommand(CommandType.GOTOPOSITION,
				Priority.NORMAL, null, null, null, null, null, StatusError.NONE,
				Completion.COMMANDPENDING);
		nc.pose=new double[3];
		nc.pose[0]=Double.parseDouble(txtXPosition.getText());
		nc.pose[1]=Double.parseDouble(txtYPosition.getText());
		nc.pose[2]=Math.PI*Double.parseDouble(txtThetaPosition.getText())/180;

		nc.tolerance=new double[3];
		nc.tolerance[0]=0.1;
		nc.tolerance[1]=0.1;
		nc.tolerance[2]=Math.PI*10.0/180.0;
		
		nc.destId=new long[1];
		nc.destId[0]=Long.parseLong(txtPlaceID.getText());
		nc.distance=new double[0];
		nc.angle=new double[0];
		return nc;
	}

	private NavCommand createPoseNavCommand(double x, double y, double theta) {
		NavCommand nc = new NavCommand(CommandType.GOTOPOSITION,
				Priority.NORMAL, null, null, null, null, null, StatusError.NONE,
				Completion.COMMANDPENDING);
		nc.pose=new double[3];
        
		nc.pose[0]=x;
		nc.pose[1]=y;
		nc.pose[2]=theta;
        logger.debug("goto pose: " + x + ", " + y + ", " + theta);

		nc.tolerance=new double[3];
		nc.tolerance[0]=0.05;
		nc.tolerance[1]=0.05;
		nc.tolerance[2]=Math.PI*5.0/180.0;
		
		nc.destId=new long[1];
		nc.destId[0]=0;
		nc.distance=new double[0];
		nc.angle=new double[0];
		return nc;
	}

	/**
	 * main function to show the GUI without any functionality
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		new ManualNavGUI();
	}

}
