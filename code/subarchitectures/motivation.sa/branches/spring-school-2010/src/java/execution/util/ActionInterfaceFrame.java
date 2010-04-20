package execution.util;

import java.awt.FlowLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JTable;
import javax.swing.table.DefaultTableModel;

import cast.CASTException;
import execution.components.GraphicalExecutionManager;
import execution.slice.Action;

public class ActionInterfaceFrame extends JFrame {

	private static final int PLACE_ID_COLUMN = 0;
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
	private GraphicalExecutionManager m_exeMan;
	private JRadioButton m_detectPeopleAction;

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
		this.setTitle("Spatial SA Actions");
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

			m_goAction.setSelected(true);

			ButtonGroup actionGroup = new ButtonGroup();
			actionGroup.add(m_goAction);
			actionGroup.add(m_avsAction);
			actionGroup.add(m_detectObjectsAction);
			actionGroup.add(m_detectPeopleAction);

			m_actionPanel.add(m_goAction, new GridBagConstraints());
			m_actionPanel.add(m_avsAction, new GridBagConstraints());
			m_actionPanel.add(m_detectObjectsAction, new GridBagConstraints());
			m_actionPanel.add(m_detectPeopleAction, new GridBagConstraints());

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

}
