package execution.util;

import java.awt.FlowLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Map;


import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JComboBox;
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
import beliefmodels.autogen.featurecontent.BooleanValue;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.FloatValue;
import beliefmodels.autogen.featurecontent.IntegerValue;
import beliefmodels.autogen.featurecontent.StringValue;
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

	private double m_tolerance = 0.2;

	private JPanel m_buttonPanel = null;
	private JButton m_goButton = null;
	private JButton m_stopButton = null;
	private JTable m_placeTable = null;
	private DefaultTableModel m_placeTableModel;
	private JPanel m_actionPanel;
	private JRadioButton m_avsAction;
	private JRadioButton m_goAction;
	private JRadioButton m_goActionRough;
	private JRadioButton m_detectObjectsAction;
	private JRadioButton m_detectPeopleAction;
	private JRadioButton m_lookForObjectsAction;
	private JRadioButton m_lookForObjectsAndPeopleAction;
	private JRadioButton m_spinAroundAction;
	private JRadioButton m_deliverMessageAction;
	private JRadioButton m_ptulookForObjectsAction;
	private JRadioButton m_lookForPeopleAction;
	private JRadioButton m_askForFeatureAction;
	private JRadioButton m_fartAction;
	private JRadioButton m_testFeatureValueAction;

	private GraphicalExecutionManager m_exeMan;
	private JTable m_beliefTable;
	private DefaultTableModel m_beliefTableModel;
	private static final Object[] FEATURE_VALUE_TYPES = { StringValue.class,
			IntegerValue.class, FloatValue.class, BooleanValue.class };

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

	public void setTolerance(double tolerance) {
		m_tolerance = tolerance;
	}

	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJContentPane() {
		if (jContentPane == null) {
			jContentPane = new JPanel();
			jContentPane.setLayout(new BoxLayout(jContentPane, BoxLayout.X_AXIS));
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
			m_actionPanel.setLayout(new BoxLayout(m_actionPanel, BoxLayout.Y_AXIS));
			m_goAction = new JRadioButton("go to place");
			m_goActionRough = new JRadioButton("go to place roughly");
			m_avsAction = new JRadioButton("visual search in");
			m_detectObjectsAction = new JRadioButton("detect objects");
			m_detectPeopleAction = new JRadioButton("detect people");
			m_lookForObjectsAction = new JRadioButton("look for objects");
			m_lookForObjectsAndPeopleAction = new JRadioButton("look for objects and people");
			m_spinAroundAction = new JRadioButton("spin around");
			m_deliverMessageAction = new JRadioButton("deliver message");
			m_fartAction = new JRadioButton("fart");
			m_ptulookForObjectsAction = new JRadioButton("ptu look for objects");
			m_lookForPeopleAction = new JRadioButton("look for people");

			m_askForFeatureAction = new JRadioButton("ask for feature");
			m_testFeatureValueAction = new JRadioButton("test a feature value");

			m_goAction.setSelected(true);

			ButtonGroup actionGroup = new ButtonGroup();
			actionGroup.add(m_goAction);
			actionGroup.add(m_goActionRough);
			actionGroup.add(m_avsAction);
			actionGroup.add(m_detectObjectsAction);
			actionGroup.add(m_detectPeopleAction);
			actionGroup.add(m_lookForObjectsAction);
			actionGroup.add(m_lookForObjectsAndPeopleAction);
			actionGroup.add(m_spinAroundAction);
			actionGroup.add(m_deliverMessageAction);
			actionGroup.add(m_fartAction);
			actionGroup.add(m_ptulookForObjectsAction);
			actionGroup.add(m_lookForPeopleAction);
			actionGroup.add(m_askForFeatureAction);
			actionGroup.add(m_testFeatureValueAction);

			m_actionPanel.add(m_goAction, new GridBagConstraints());
			m_actionPanel.add(m_goActionRough, new GridBagConstraints());
			m_actionPanel.add(m_avsAction, new GridBagConstraints());
			m_actionPanel.add(m_detectObjectsAction, new GridBagConstraints());
			m_actionPanel.add(m_detectPeopleAction, new GridBagConstraints());
			m_actionPanel.add(m_lookForObjectsAction, new GridBagConstraints());
			m_actionPanel.add(m_lookForObjectsAndPeopleAction, new GridBagConstraints());
			m_actionPanel.add(m_spinAroundAction, new GridBagConstraints());
			m_actionPanel.add(m_deliverMessageAction, new GridBagConstraints());
			m_actionPanel.add(m_fartAction, new GridBagConstraints());
			m_actionPanel.add(m_ptulookForObjectsAction, new GridBagConstraints());
			m_actionPanel.add(m_lookForPeopleAction, new GridBagConstraints());
			m_actionPanel.add(m_askForFeatureAction, new GridBagConstraints());
			m_actionPanel.add(m_testFeatureValueAction,
					new GridBagConstraints());
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
		} else if (m_goActionRough.isSelected()) {
			goToPlaceRough();
		} else if (m_avsAction.isSelected()) {
			runAVS();
		} else if (m_detectObjectsAction.isSelected()) {
			detectObjects();

		} else if (m_detectPeopleAction.isSelected()) {
			detectPeople();
		} else if (m_lookForObjectsAction.isSelected()) {
			lookForObjects();
		} else if (m_lookForObjectsAndPeopleAction.isSelected()) {
			lookForObjectsAndPeople();
		} else if (m_spinAroundAction.isSelected()) {
			spinAround();
		} else if (m_deliverMessageAction.isSelected()) {
			deliverMessage();
		} else if (m_fartAction.isSelected()) {
			fart();
		} else if (m_ptulookForObjectsAction.isSelected()) {
			ptulookForObjects();
		} else if (m_lookForPeopleAction.isSelected()) {
			lookForPeople();
		} else if (m_askForFeatureAction.isSelected()) {
			askForFeature();
		} else if (m_testFeatureValueAction.isSelected()) {
			testFeatureValue();
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

			final JTextField textfield = new JTextField(10);
			dialog.add(textfield);

			ActionListener submit = new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent _e) {
					submitFeatureQuery(beliefID, dialog, textfield);
					dialog.setVisible(false);
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
	 * Popup
	 */
	private void testFeatureValue() {
		int selectedRow = m_beliefTable.getSelectedRow();
		if (selectedRow != -1) {
			Object beliefIDVal = m_beliefTableModel.getValueAt(selectedRow,
					BELIEF_ID_COLUMN);
			assert (beliefIDVal != null);
			final String beliefID = (String) beliefIDVal;

			final JDialog dialog = new JDialog(this);
			dialog.setLayout(new FlowLayout());
			dialog
					.add(new JLabel(
							"What feature TYPE do you want to ask about?"));
			final JTextField textfield = new JTextField(10);
			dialog.add(textfield);

			dialog.add(new JLabel(
					"VALUE TYPE: "));

			final JComboBox featureValueTypesCombo = new JComboBox(
					FEATURE_VALUE_TYPES);

			dialog.add(featureValueTypesCombo);
			dialog.add(new JLabel("VALUE: "));

			final JTextField valuefield = new JTextField(10);
			dialog.add(valuefield);

			ActionListener submit = new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent _e) {
					submitFeatureValueTest(beliefID, textfield.getText(),
							FEATURE_VALUE_TYPES[featureValueTypesCombo
									.getSelectedIndex()], valuefield.getText());
					dialog.setVisible(false);
				}
			};

			valuefield.addActionListener(submit);

			JButton goButton = new JButton("Go!");
			goButton.addActionListener(submit);

			dialog.add(goButton);
			dialog.pack();
			dialog.setVisible(true);

		}

	}

	private void submitFeatureValueTest(String _beliefID, String _featureLabel,
			Object _valueType, String _value) {
		if (_featureLabel.isEmpty() || _value.isEmpty()) {
			m_exeMan
					.println("Missing values for feature test. Please fill in all the fields");
			return;
		}

		FeatureValue fv = null;

		if (_valueType == StringValue.class) {
			fv = new StringValue(_value);
		} else if (_valueType == IntegerValue.class) {
			fv = new IntegerValue(Integer.parseInt(_value));
		} else if (_valueType == FloatValue.class) {
			fv = new FloatValue(Float.parseFloat(_value));
		} else if (_valueType == BooleanValue.class) {
			fv = new BooleanValue(Boolean.parseBoolean(_value));
		} else {
			assert (false);
		}

		//final sanity check in case assertions are disable
		if(fv != null) {
			try {
				m_exeMan.triggerFeatureValueTest(_beliefID, _featureLabel, fv,
						new MonitorPanel());
			} catch (CASTException e) {
				m_exeMan.logException(e);
			}
		}
		
	}

	/**
	 * @param beliefID
	 * @param dialog
	 * @param textfield
	 */
	private void submitFeatureQuery(final String beliefID,
			final JDialog dialog, final JTextField textfield) {
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
	private void goToPlaceRough() throws CASTException {
		int selectedRow = m_placeTable.getSelectedRow();
		if (selectedRow != -1) {
			Object placeIDVal = m_placeTableModel.getValueAt(selectedRow,
					PLACE_ID_COLUMN);
			assert (placeIDVal != null);
			long placeID = (Long) placeIDVal;
			double tolerance = m_tolerance;
			m_exeMan.triggerGoToRoughAction(placeID, tolerance, new MonitorPanel());
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

	private void lookForObjectsAndPeople() throws CASTException {
		m_exeMan.triggerLookForObjectsAndPeople(new MonitorPanel());
	}

	private void spinAround() throws CASTException {
		m_exeMan.triggerSpinAround(new MonitorPanel());
	}

        private void deliverMessage() throws CASTException {
            try {
                
            String command = "./delivermessage.sh";
            Runtime.getRuntime().exec(command);
            } catch (Exception e) {
                System.out.println("Exception " + e.getMessage());
            }            
	}

    private void fart() throws CASTException {
        System.out.println("Farting now....");
    }

	private void ptulookForObjects() throws CASTException {
		m_exeMan.triggerPTULookForObjects(new MonitorPanel());
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
