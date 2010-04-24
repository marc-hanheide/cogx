package execution.util;

import java.awt.FlowLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Map;

import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.table.DefaultTableModel;

import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.ProbDistribution;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import execution.components.GraphicalExecutionManager;
import execution.slice.Action;

public class ActionInterfaceFrame extends JFrame {

	private static final int PLACE_ID_COLUMN = 0;
	private static final int BELIEF_ID_COLUMN = 0;
	@SuppressWarnings("unused")
	private static final int BELIEF_TYPE_COLUMN = 1;
	private static final long serialVersionUID = 1L;
	private JPanel jContentPane = null;

	private JPanel m_buttonPanel = null;
	private JButton m_goButton = null;
	private JButton m_stopButton = null;
	private JTable m_placeTable = null;
	private DefaultTableModel m_placeTableModel;
	private JPanel m_actionPanel;
	private JRadioButton m_avsAction;
	private JRadioButton m_goAction;
	private JRadioButton m_detectObjectsAction;
	private JRadioButton m_detectPeopleAction;
	private JRadioButton m_lookForObjectsAction;
	private JRadioButton m_lookForPeopleAction;
	private JRadioButton m_askForFeatureAction;

	private GraphicalExecutionManager m_exeMan;
	private JTable m_beliefTable;
	private DefaultTableModel m_beliefTableModel;

	/**
	 * This is the default constructor
	 * 
	 * @param _graphicalExecutionManager
	 */
	public ActionInterfaceFrame(
			GraphicalExecutionManager _graphicalExecutionManager) {
		super();
		m_exeMan = _graphicalExecutionManager;
		initialize();
	}

	/**
	 * This method initializes this
	 * 
	 * @return void
	 */
	private void initialize() {
		this.setSize(300, 200);
		this.setContentPane(getJContentPane());
		this.setTitle("Robot Actions (Don't you just hate writing GUIs?)");
	}

	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJContentPane() {
		if (jContentPane == null) {
			jContentPane = new JPanel();
			jContentPane.setLayout(new FlowLayout());
			jContentPane.add(getM_placeTable(), null);
			JPanel middlePanel = new JPanel();
			middlePanel.setLayout(new BoxLayout(middlePanel, BoxLayout.Y_AXIS));
			middlePanel.add(getActionPanel(), null);
			middlePanel.add(getM_buttonPanel(), null);
			jContentPane.add(middlePanel, null);
			jContentPane.add(getBeliefTable(), null);
		}
		return jContentPane;
	}

	public void addPlace(long _id) {
		// 1Model.addRow(new Object[] { _id, true });
		m_placeTableModel.addRow(new Object[] { _id });
		pack();
	}

	public void removePlace(long _placeID) {
		// m_placeTableModel.removeRow(row)
	}

	/**
	 * This method initializes m_buttonPanel
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getM_buttonPanel() {
		if (m_buttonPanel == null) {
			m_buttonPanel = new JPanel();
			m_buttonPanel.setLayout(new GridBagLayout());
			m_buttonPanel.add(getM_goButton(), new GridBagConstraints());
			m_buttonPanel.add(getM_stopButton(), new GridBagConstraints());
		}
		return m_buttonPanel;
	}

	/**
	 * This method initializes m_buttonPanel
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getActionPanel() {
		if (m_actionPanel == null) {
			m_actionPanel = new JPanel();
			m_actionPanel.setLayout(new GridBagLayout());
			m_goAction = new JRadioButton("go to place");
			m_avsAction = new JRadioButton("visual search in");
			m_detectObjectsAction = new JRadioButton("detect objects");
			m_detectPeopleAction = new JRadioButton("detect people");
			m_lookForObjectsAction = new JRadioButton("look for objects");
			m_lookForPeopleAction = new JRadioButton("look for people");

			m_askForFeatureAction = new JRadioButton("ask for feature");

			m_goAction.setSelected(true);

			ButtonGroup actionGroup = new ButtonGroup();
			actionGroup.add(m_goAction);
			actionGroup.add(m_avsAction);
			actionGroup.add(m_detectObjectsAction);
			actionGroup.add(m_detectPeopleAction);
			actionGroup.add(m_lookForObjectsAction);
			actionGroup.add(m_lookForPeopleAction);
			actionGroup.add(m_askForFeatureAction);

			m_actionPanel.add(m_goAction, new GridBagConstraints());
			m_actionPanel.add(m_avsAction, new GridBagConstraints());
			m_actionPanel.add(m_detectObjectsAction, new GridBagConstraints());
			m_actionPanel.add(m_detectPeopleAction, new GridBagConstraints());
			m_actionPanel.add(m_lookForObjectsAction, new GridBagConstraints());
			m_actionPanel.add(m_lookForPeopleAction, new GridBagConstraints());
			m_actionPanel.add(m_askForFeatureAction, new GridBagConstraints());
		}
		return m_actionPanel;
	}

	/**
	 * This method initializes m_goButton
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getM_goButton() {
		if (m_goButton == null) {
			m_goButton = new JButton("Go!");
			m_goButton.addActionListener(new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent _e) {
					try {
						go();
					} catch (CASTException e) {
						e.printStackTrace();
					}
				}
			});
		}
		return m_goButton;
	}

	private class MonitorPanel implements ActionMonitor {

		@Override
		public void actionComplete(Action _action) {
			m_exeMan.println("Action complete");
		}

	}

	private void go() throws CASTException {
		// get action
		if (m_goAction.isSelected()) {
			goToPlace();
		} else if (m_avsAction.isSelected()) {
			runAVS();
		} else if (m_detectObjectsAction.isSelected()) {
			detectObjects();

		} else if (m_detectPeopleAction.isSelected()) {
			detectPeople();
		} else if (m_lookForObjectsAction.isSelected()) {
			lookForObjects();
		} else if (m_lookForPeopleAction.isSelected()) {
			lookForPeople();
		} else if (m_askForFeatureAction.isSelected()) {
			askForFeature();
		}

	}

	/**
	 * Popup
	 */
	private void askForFeature() {
		int selectedRow = m_beliefTable.getSelectedRow();
		if (selectedRow != -1) {
			Object beliefIDVal = m_beliefTableModel.getValueAt(selectedRow,
					BELIEF_ID_COLUMN);
			assert (beliefIDVal != null);
			final String beliefID = (String) beliefIDVal;

			final JDialog dialog = new JDialog(this);
			dialog.setLayout(new FlowLayout());
			dialog.add(new JLabel("What feature do you want to ask about?"));

			final JTextField textfield = new JTextField(30);
			dialog.add(textfield);

			ActionListener submit = new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent _e) {
					submit(beliefID, dialog, textfield);
				}
			};

			textfield.addActionListener(submit);

			JButton goButton = new JButton("Go!");
			goButton.addActionListener(submit);

			dialog.add(goButton);
			dialog.pack();
			dialog.setVisible(true);

		}

	}

	/**
	 * @param beliefID
	 * @param dialog
	 * @param textfield
	 */
	private void submit(final String beliefID, final JDialog dialog,
			final JTextField textfield) {
		dialog.setVisible(false);
		String featureType = textfield.getText();
		if (featureType.length() > 0) {
			try {
				m_exeMan.triggerAskForFeatureAction(beliefID, featureType,
						new MonitorPanel());
			} catch (CASTException e) {
				m_exeMan.logException(e);
			}
		}
	}

	/**
	 * @throws CASTException
	 */
	private void runAVS() throws CASTException {
		int[] selectedRows = m_placeTable.getSelectedRows();
		if (selectedRows.length > 0) {
			long[] placeIDs = new long[selectedRows.length];
			for (int i = 0; i < selectedRows.length; ++i) {
				placeIDs[i] = (Long) m_placeTableModel.getValueAt(
						selectedRows[i], PLACE_ID_COLUMN);
			}
			m_exeMan.triggerAVSAction(placeIDs, new MonitorPanel());

		}
	}

	/**
	 * @throws CASTException
	 */
	private void goToPlace() throws CASTException {
		int selectedRow = m_placeTable.getSelectedRow();
		if (selectedRow != -1) {
			Object placeIDVal = m_placeTableModel.getValueAt(selectedRow,
					PLACE_ID_COLUMN);
			assert (placeIDVal != null);
			long placeID = (Long) placeIDVal;
			m_exeMan.triggerGoToAction(placeID, new MonitorPanel());
		}
	}

	/**
	 * @throws CASTException
	 */
	private void detectObjects() throws CASTException {
		m_exeMan.triggerDetectObjects(new MonitorPanel());
	}

	/**
	 * @throws CASTException
	 */
	private void detectPeople() throws CASTException {
		m_exeMan.triggerDetectPeople(new MonitorPanel());
	}

	private void lookForPeople() throws CASTException {
		m_exeMan.triggerLookForPeople(new MonitorPanel());
	}
	
	private void lookForObjects() throws CASTException {
		m_exeMan.triggerLookForObjects(new MonitorPanel());
	}

	/**
	 * This method initializes m_stopButton
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getM_stopButton() {
		if (m_stopButton == null) {
			m_stopButton = new JButton("Stop!");
			m_stopButton.addActionListener(new ActionListener() {
				@Override
				public void actionPerformed(ActionEvent _e) {
					try {
						m_exeMan.stopCurrentAction();
					} catch (CASTException e) {
						m_exeMan.logException(e);
					}
				}
			});
		}
		return m_stopButton;
	}

	/**
	 * This method initializes m_placeTable
	 * 
	 * @return javax.swing.JTable
	 */
	private JTable getM_placeTable() {
		if (m_placeTable == null) {
			// m_placeTable = new JTable(1, 2);
			// m_placeTableModel = new DefaultTableModel(new String[] { "id",
			// "explored" }, 0);
			m_placeTable = new JTable(1, 1);
			m_placeTableModel = new DefaultTableModel(new String[] { "id" }, 0);
			m_placeTable.setModel(m_placeTableModel);
		}
		return m_placeTable;
	}

	/**
	 * This method initializes m_beliefTable
	 * 
	 * @return javax.swing.JTable
	 */
	private JTable getBeliefTable() {
		if (m_beliefTable == null) {
			// m_placeTable = new JTable(1, 2);
			// m_placeTableModel = new DefaultTableModel(new String[] { "id",
			// "explored" }, 0);
			m_beliefTable = new JTable(1, 2);
			m_beliefTableModel = new DefaultTableModel(new String[] { "id",
					"type" }, 0);
			m_beliefTable.setModel(m_beliefTableModel);
		}
		return m_beliefTable;
	}

	public void addBelief(WorkingMemoryAddress _address, StableBelief _belief) {
		println(_belief.type);
		assert (_belief.content instanceof CondIndependentDistribs);
		CondIndependentDistribs cid = (CondIndependentDistribs) _belief.content;
		Map<String, ProbDistribution> featureDistributions = cid.distribs;
		for (String featureType : featureDistributions.keySet()) {
			println(featureType);
		}
		m_beliefTableModel.addRow(new Object[] { _address.id, _belief.type });
		pack();

	}

	private void println(Object _o) {
		m_exeMan.println(_o);
	}

	public void removeBelief(WorkingMemoryAddress _address) {
		// TODO Auto-generated method stub

	}

}
