package fakecoma.gui;

import java.awt.FlowLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Vector;

import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.table.DefaultTableModel;

import cast.CASTException;
import fakecoma.components.GraphicalComa;

public class ComaJFrame extends JFrame {

	private static final int PLACE_ID_COLUMN = 0;
	private static final long serialVersionUID = 1L;
	private JPanel jContentPane = null;

	private JPanel m_actionPanel;
	private JRadioButton m_avsAction;
	private JPanel m_buttonPanel = null;
	private JRadioButton m_detectObjectsAction;
	private JRadioButton m_detectPeopleAction;
	private JRadioButton m_goAction;
	private JButton m_goButton = null;
	private JTable m_placeTable = null;
	private DefaultTableModel m_placeTableModel;
	private JButton m_stopButton = null;
	private JPanel m_roomPanel;
	private JTextField m_roomLabel;
	private ActionListener goAction;

	/**
	 * This is the default constructor
	 * 
	 * @param _graphicalExecutionManager
	 */
	public ComaJFrame(ActionListener goAction) {
		super();
		this.goAction = goAction;
		initialize();
	}

	public void addPlace(long _id) {
		// 1Model.addRow(new Object[] { _id, true });
		m_placeTableModel.addRow(new Object[] { _id });
		pack();
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
	private JTextField getRoomLabel() {
		if (m_roomLabel == null) {
			m_roomLabel = new JTextField();
			m_roomLabel.setColumns(30);
		}
		return m_roomLabel;
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
			middlePanel.add(getRoomLabelPanel(), null);

			//middlePanel.add(getActionPanel(), null);
			middlePanel.add(getM_buttonPanel(), null);
			jContentPane.add(middlePanel, null);
		}
		return jContentPane;
	}

	private JPanel getRoomLabelPanel() {
		if (m_roomPanel == null) {
			m_roomPanel = new JPanel();
			m_roomPanel.setLayout(new GridBagLayout());
			m_roomPanel.add(getRoomLabel(), new GridBagConstraints());
		}
		return m_roomPanel;
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
			//m_buttonPanel.add(getM_stopButton(), new GridBagConstraints());
		}
		return m_buttonPanel;
	}

	/**
	 * This method initializes m_goButton
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getM_goButton() {
		if (m_goButton == null) {
			m_goButton = new JButton("Go!");
			m_goButton.addActionListener(goAction);
		}
		return m_goButton;
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
	 * This method initializes this
	 * 
	 * @return void
	 */
	private void initialize() {
		this.setSize(300, 200);
		this.setContentPane(getJContentPane());
		this.setTitle(GraphicalComa.class.getSimpleName());
	}

	public void removePlace(long _placeID) {
		// m_placeTableModel.removeRow(row)
	}

	public Vector<Long> getPlaces() {
		int[] rows=m_placeTable.getSelectedRows();
		Vector<Long> result = new Vector<Long>();
		for (int r:rows) {
			Object val = m_placeTableModel.getValueAt(r, 0);
			result.add((Long) val);
		}
		return result;
	}

}
