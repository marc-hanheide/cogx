package manipulation.muster.visualisation;

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

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.itemMemory.ItemMemory;
import manipulation.muster.strategies.Strategy.Name;

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

	private JButton btnExecuteMobileStrategy;

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

		btnExecuteMobileStrategy = new JButton("Execute mobile Manipulation");
		btnExecuteMobileStrategy.setActionCommand("execMobMan");
		btnExecuteMobileStrategy.addActionListener(this);

		// cont, gbl, comp, x, y, width, height, weightx, weighty
		addComponent(pane, gbl, btnExecuteMobileStrategy, 0, 9, 5, 1, 0, 0);

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
		if (e.getActionCommand().equals("execMobMan")) {
			logger.debug("pressed btnExecuteMobileStrategy");
			// TODO nicht so toll hier nen thread zu starten
			Thread t = new Thread(new Runnable() {
				@Override
				public void run() {
					manipulator.getRunner().startStrategy(
							Name.MOBILE_MANIPULATION);
				}
			});
			t.start();
			mapPanel = new MapPanel(manipulator);
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
