package vision.components;

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

import VisionData.Person;
import VisionData.PeopleDetectionCommand;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEventQueue;

/**
 * GUI to use for the calibration procedure
 * 
 * @author hanheidm
 * 
 */
public class PersonDetectorGUI extends JPanel implements ActionListener {

	/**
	 * 
	 */
	private static final long serialVersionUID = -1527036976990471692L;

	private Logger logger = Logger.getLogger(this.getClass());

	private PersonDetectorRunner server;

	private JTextField txtPersonProb;

	private JButton btnGetPTZValue;

	public PersonDetectorGUI(PersonDetectorRunner server) {
		this.server = server;

		guiSetup();
	}

	/**
	 * constructor to show the GUI without any functionality
	 */
	public PersonDetectorGUI() {
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
		JFrame gui = new JFrame("PersonDetect Command Test GUI");
		gui.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		gui.setLocation(100, 200);

		Container pane = gui.getContentPane();
		GridBagLayout gbl = new GridBagLayout();
		pane.setLayout(gbl);

		txtPersonProb = new JTextField("", 7);

		btnGetPTZValue = new JButton("query Person Detector");
		btnGetPTZValue.setActionCommand("get");
		btnGetPTZValue.addActionListener(this);

		// cont, gbl, comp, x, y, width, height, weightx, weighty
		addComponent(pane, gbl, new JLabel("prob:"), 0, 0, 1, 1, 0, 0);
		addComponent(pane, gbl, txtPersonProb, 1, 0, 2, 1, 0, 0);

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
		PeopleDetectionCommand cmd = new PeopleDetectionCommand();
		String id = server.newDataID();
		WMEventQueue cmdEventQueue = new WMEventQueue();
		WMEventQueue personEventQueue = new WMEventQueue();
		server.addChangeFilter(ChangeFilterFactory.createAddressFilter(id,
				server.getSubarchitectureID(), WorkingMemoryOperation.DELETE),
				cmdEventQueue);
		server.addChangeFilter(ChangeFilterFactory.createTypeFilter(Person.class, WorkingMemoryOperation.ADD),
				personEventQueue);
		try {
			txtPersonProb.setText("???"); 
			server.addToWorkingMemory(id, cmd);
			logger.debug("waiting for command to be executed");
			cmdEventQueue.take();
			logger.debug("waiting for result to arrive");
			WorkingMemoryChange personEvent = personEventQueue.take();
			Person person = server.getMemoryEntry(personEvent.address, Person.class);
			txtPersonProb.setText(Double.toString(person.existProb)); 
		} catch (CASTException e1) {
			logger.error(e1);
		} catch (InterruptedException e2) {
			logger.error(e2);
		}
	}

	/**
	 * main function to show the GUI without any functionality
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		new PersonDetectorGUI();
	}

}
