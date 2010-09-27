package dora.execution.util;

import java.awt.FlowLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.HashMap;
import java.util.Map.Entry;

import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.JTabbedPane;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.table.DefaultTableModel;

import SpatialData.PlaceStatus;
import SpatialData.ViewPoint;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import dora.execution.components.GraphicalExecutionManager;
import execution.slice.Action;
import execution.util.ActionMonitor;

public class ActionInterfaceFrame extends JFrame {

	private static final int PLACE_ID_COLUMN = 0;
	private static final int BELIEF_ID_COLUMN = 0;
	private static final int OBJECT_MODEL_COLUMN = 0;

	@SuppressWarnings("unused")
	private static final int BELIEF_TYPE_COLUMN = 1;
	private static final long serialVersionUID = 1L;
	private JPanel jContentPane = null;

	private JPanel m_buttonPanel = null;
	private JButton m_goButton = null;
	private JButton m_stopButton = null;

	private JTable m_placeTable = null;
	private DefaultTableModel m_placeTableModel;
	private JTable m_coneTable = null;
	private DefaultTableModel m_coneTableModel;

	private JPanel m_placesActionPanel;
	private JRadioButton m_generateConesAction;
	private JRadioButton m_processConeAction;
	private JRadioButton m_goAction;
	private JRadioButton m_detectObjectsAction;
	private JRadioButton m_detectPeopleAction;
	private JRadioButton m_lookForObjectsAction;
	// private JRadioButton m_lookForPeopleAction;
	private JRadioButton m_askForFeatureAction;
	private JRadioButton m_testFeatureValueAction;

	private JRadioButton m_foregroundModelsAction;
	private JRadioButton m_backgroundModelsAction;
	private JRadioButton m_recogniseForegroundedModelsAction;

	private GraphicalExecutionManager m_exeMan;
	private JTable m_beliefTable;
	private DefaultTableModel m_beliefTableModel;
	private JPanel m_beliefsActionPanel;
	private JPanel m_objectsActionPanel;
	private JPanel m_placesPanel;
	private JPanel m_beliefsPanel;
	private JPanel m_objectsPanel;
	private JTabbedPane m_tabbedPane;
	private JTable m_objectTable;
	private DefaultTableModel m_objectTableModel;
	private final HashMap<String, WorkingMemoryAddress> m_cones;
	private static final Class<?>[] FEATURE_VALUE_TYPES = {
			ElementaryFormula.class, IntegerFormula.class, FloatFormula.class,
			BooleanFormula.class };
	private static final int CONE_ID_COLUMN = 0;

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
		m_cones = new HashMap<String, WorkingMemoryAddress>();
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

	private JPanel getPlacesPanel() {
		if (m_placesPanel == null) {
			m_placesPanel = new JPanel();
			m_placesPanel.add(new JScrollPane(getPlaceTable()));
			m_placesPanel.add(new JScrollPane(getConeTable()));
			m_placesPanel.add(getPlacesActionPanel());
		}
		return m_placesPanel;
	}

	private JPanel getBeliefsPanel() {
		if (m_beliefsPanel == null) {
			m_beliefsPanel = new JPanel();
			m_beliefsPanel.add(new JScrollPane(getBeliefTable()));
			m_beliefsPanel.add(getBeliefsActionPanel());
		}
		return m_beliefsPanel;
	}

	private JPanel getObjectsPanel() {
		if (m_objectsPanel == null) {
			m_objectsPanel = new JPanel();
			m_objectsPanel.add(new JScrollPane(getObjectTable()));
			m_objectsPanel.add(getObjectsActionPanel());
		}
		return m_objectsPanel;
	}

	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJContentPane() {
		if (jContentPane == null) {
			jContentPane = new JPanel();
			jContentPane.setLayout(new GridLayout(2, 1));
			JPanel middlePanel = new JPanel();
			middlePanel.setLayout(new BoxLayout(middlePanel, BoxLayout.Y_AXIS));
			middlePanel.add(getTabbedPane(), null);
			middlePanel.add(getButtonPanel(), null);
			jContentPane.add(middlePanel, null);

		}
		return jContentPane;
	}

	/**
	 * @return
	 */
	private JTabbedPane getTabbedPane() {

		if (m_tabbedPane == null) {

			m_tabbedPane = new JTabbedPane();
			m_tabbedPane.addTab("Places", getPlacesPanel());
			m_tabbedPane.addTab("Beliefs", getBeliefsPanel());
			m_tabbedPane.addTab("Objects", getObjectsPanel());

		}
		return m_tabbedPane;
	}

	public void addPlace(long _id, PlaceStatus _status) {
		// 1Model.addRow(new Object[] { _id, true });
		m_placeTableModel.addRow(new Object[] { _id, _status });
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
	private JPanel getButtonPanel() {
		if (m_buttonPanel == null) {
			m_buttonPanel = new JPanel();
			m_buttonPanel.setLayout(new GridBagLayout());
			m_buttonPanel.add(getGoButton(), new GridBagConstraints());
			m_buttonPanel.add(getM_stopButton(), new GridBagConstraints());
		}
		return m_buttonPanel;
	}

	/**
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getPlacesActionPanel() {
		if (m_placesActionPanel == null) {
			m_placesActionPanel = new JPanel();
			m_placesActionPanel.setLayout(new GridBagLayout());

			m_goAction = new JRadioButton("go to place");
			m_generateConesAction = new JRadioButton(
					"generate cones in selected places");

			m_processConeAction = new JRadioButton("process selected cone");

			m_goAction.setSelected(true);

			ButtonGroup actionGroup = new ButtonGroup();
			actionGroup.add(m_goAction);
			actionGroup.add(m_generateConesAction);

			actionGroup.add(m_processConeAction);

			m_placesActionPanel.add(m_goAction, new GridBagConstraints());
			m_placesActionPanel.add(m_generateConesAction,
					new GridBagConstraints());
			m_placesActionPanel.add(m_processConeAction,
					new GridBagConstraints());
		}
		return m_placesActionPanel;
	}

	/**
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getBeliefsActionPanel() {
		if (m_beliefsActionPanel == null) {
			m_beliefsActionPanel = new JPanel();
			m_beliefsActionPanel.setLayout(new GridBagLayout());

			m_askForFeatureAction = new JRadioButton("ask for feature");
			m_testFeatureValueAction = new JRadioButton("test a feature value");

			m_askForFeatureAction.setSelected(true);

			ButtonGroup actionGroup = new ButtonGroup();
			actionGroup.add(m_askForFeatureAction);
			actionGroup.add(m_testFeatureValueAction);

			m_beliefsActionPanel.add(m_askForFeatureAction,
					new GridBagConstraints());
			m_beliefsActionPanel.add(m_testFeatureValueAction,
					new GridBagConstraints());
		}
		return m_beliefsActionPanel;
	}

	/**
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getObjectsActionPanel() {
		if (m_objectsActionPanel == null) {
			m_objectsActionPanel = new JPanel();

			m_objectsActionPanel.setLayout(new GridBagLayout());
			m_detectObjectsAction = new JRadioButton("detect objects");
			m_detectPeopleAction = new JRadioButton("detect people");
			m_lookForObjectsAction = new JRadioButton("look for objects");
			// m_lookForPeopleAction = new JRadioButton("look for people");
			m_foregroundModelsAction = new JRadioButton("foreground models");
			m_backgroundModelsAction = new JRadioButton("background models");
			m_recogniseForegroundedModelsAction = new JRadioButton(
					"recognise foregrounded models");

			// m_avsAction.setSelected(true);
			m_detectObjectsAction.setSelected(true);

			ButtonGroup actionGroup = new ButtonGroup();
			actionGroup.add(m_detectObjectsAction);
			actionGroup.add(m_detectPeopleAction);
			actionGroup.add(m_lookForObjectsAction);
			// actionGroup.add(m_lookForPeopleAction);
			actionGroup.add(m_foregroundModelsAction);
			actionGroup.add(m_backgroundModelsAction);
			actionGroup.add(m_recogniseForegroundedModelsAction);

			m_objectsActionPanel.add(m_detectObjectsAction,
					new GridBagConstraints());
			m_objectsActionPanel.add(m_detectPeopleAction,
					new GridBagConstraints());
			m_objectsActionPanel.add(m_lookForObjectsAction,
					new GridBagConstraints());
			// m_objectsActionPanel.add(m_lookForPeopleAction,
			// new GridBagConstraints());
			m_objectsActionPanel.add(m_foregroundModelsAction,
					new GridBagConstraints());
			m_objectsActionPanel.add(m_backgroundModelsAction,
					new GridBagConstraints());
			m_objectsActionPanel.add(m_recogniseForegroundedModelsAction,
					new GridBagConstraints());
		}
		return m_objectsActionPanel;
	}

	/**
	 * This method initializes m_goButton
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getGoButton() {
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

		// TODO make more robust to code changes
		int tabIndex = getTabbedPane().getSelectedIndex();

		m_exeMan.println("go() called: " + tabIndex);

		if (tabIndex == 0) {
			if (m_generateConesAction.isSelected()) {
				generateCones();
			} else if (m_processConeAction.isSelected()) {
				processCone();
			} else if (m_goAction.isSelected()) {
				goToPlace();
			}
		} else if (tabIndex == 2) {

			if (m_detectObjectsAction.isSelected()) {
				detectObjects();

			} else if (m_detectPeopleAction.isSelected()) {
				detectPeople();
			}

			else if (m_lookForObjectsAction.isSelected()) {
				lookForObjects();
			}
			// else if (m_lookForPeopleAction.isSelected()) {
			// lookForPeople();
			// }
			else if (m_foregroundModelsAction.isSelected()) {
				foregroundModels();
			} else if (m_backgroundModelsAction.isSelected()) {
				backgroundModels();
			} else if (m_recogniseForegroundedModelsAction.isSelected()) {
				recogniseForegroundedModels();
			}

		} else if (tabIndex == 1) {
			if (m_askForFeatureAction.isSelected()) {
				askForFeature();
			} else if (m_testFeatureValueAction.isSelected()) {
				testFeatureValue();
			}
		} else {
			throw new RuntimeException("No tab selected apparently... "
					+ m_tabbedPane.getSelectedIndex());
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
			dialog.add(new JLabel("What feature TYPE do you want to ask about?"));
			final JTextField textfield = new JTextField(10);
			dialog.add(textfield);

			dialog.add(new JLabel("VALUE TYPE: "));

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
			m_exeMan.println("Missing values for feature test. Please fill in all the fields");
			return;
		}

		// FeatureValue fv = null;
		//
		// if (_valueType == StringValue.class) {
		// fv = new StringValue(_value);
		// } else if (_valueType == IntegerValue.class) {
		// fv = new IntegerValue(Integer.parseInt(_value));
		// } else if (_valueType == FloatValue.class) {
		// fv = new FloatValue(Float.parseFloat(_value));
		// } else if (_valueType == BooleanValue.class) {
		// fv = new BooleanValue(Boolean.parseBoolean(_value));
		// } else {
		assert (false);
		// }

		// final sanity check in case assertions are disable
		// if(fv != null) {
		// try {
		// m_exeMan.triggerFeatureValueTest(_beliefID, _featureLabel, fv,
		// new MonitorPanel());
		// } catch (CASTException e) {
		// m_exeMan.logException(e);
		// }
		// }

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
	private void generateCones() throws CASTException {
		int[] selectedRows = m_placeTable.getSelectedRows();
		if (selectedRows.length > 0) {
			long[] placeIDs = new long[selectedRows.length];
			for (int i = 0; i < selectedRows.length; ++i) {

				Object placeIDVal = m_placeTableModel.getValueAt(
						selectedRows[i], PLACE_ID_COLUMN);
				assert (placeIDVal != null);
				placeIDs[i] = (Long) placeIDVal;
			}
			m_exeMan.triggerConeGeneration((String) JOptionPane
					.showInputDialog(this,
							"What object should the cones be generated for?"),
					placeIDs, new MonitorPanel());

		} else {
			m_exeMan.println("no places selected, doing nothing");
		}
	}

	/**
	 * @throws CASTException
	 */
	private void processCone() throws CASTException {
		int selectedRow = m_coneTable.getSelectedRow();
		if (selectedRow != -1) {
			m_exeMan.log("processCone()");
			Object coneIDVal = m_coneTableModel.getValueAt(selectedRow,
					CONE_ID_COLUMN);
			assert (coneIDVal != null);
			String coneID = (String) coneIDVal;
			WorkingMemoryAddress coneAddr = m_cones.get(coneID);
			assert(coneAddr != null);
			m_exeMan.triggerProccesCone(coneAddr, new MonitorPanel());
		} else {
			m_exeMan.println("no cone selected, doing nothing");
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
		m_exeMan.log("detectObjects");
		m_exeMan.triggerDetectObjects(getSelectedObjectModels(),
				new MonitorPanel());
	}

	/**
	 * @throws CASTException
	 */
	private void foregroundModels() throws CASTException {
		m_exeMan.foregroundModels(getSelectedObjectModels(), new MonitorPanel());
	}

	/**
	 * @throws CASTException
	 */
	private void backgroundModels() throws CASTException {
		m_exeMan.backgroundModels(getSelectedObjectModels(), new MonitorPanel());
	}

	/**
	 * @throws CASTException
	 */
	private void recogniseForegroundedModels() throws CASTException {
		m_exeMan.recogniseForegroundedModels(new MonitorPanel());
	}

	/**
	 * @throws CASTException
	 */
	private void detectPeople() throws CASTException {
		m_exeMan.triggerDetectPeople(new MonitorPanel());
	}

	// private void lookForPeople() throws CASTException {
	// m_exeMan.triggerLookForPeople(new MonitorPanel());
	// }
	//
	private void lookForObjects() throws CASTException {
		m_exeMan.log("lookForObjects");
		m_exeMan.triggerLookForObjects(getSelectedObjectModels(),
				new MonitorPanel());
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
	private JTable getPlaceTable() {
		if (m_placeTable == null) {

			m_placeTable = new JTable(1, 2);
			m_placeTableModel = new DefaultTableModel(new String[] { "id",
					"status" }, 0);
			m_placeTable.setModel(m_placeTableModel);
		}
		return m_placeTable;
	}

	/**
	 * This method initializes m_coneTable
	 * 
	 * @return javax.swing.JTable
	 */
	private JTable getConeTable() {
		if (m_coneTable == null) {
			// m_coneTable = new JTable(1, 2);
			// m_coneTableModel = new DefaultTableModel(new String[] { "id",
			// "explored" }, 0);
			m_coneTable = new JTable(1, 1);
			m_coneTableModel = new DefaultTableModel(new String[] { "id" }, 0);
			m_coneTable.setModel(m_coneTableModel);
		}
		return m_coneTable;
	}

	/**
	 * This method initializes m_objectTable
	 * 
	 * @return javax.swing.JTable
	 */
	private JTable getObjectTable() {
		if (m_objectTable == null) {
			m_objectTable = new JTable(1, 2);
			m_objectTableModel = new DefaultTableModel(
					new String[] { "model" }, 0);
			m_objectTable.setModel(m_objectTableModel);
		}
		return m_objectTable;
	}

	public void setObjectModels(String[] _models) {
		for (String model : _models) {
			m_objectTableModel.addRow(new Object[] { model });
		}
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

	public void addBelief(WorkingMemoryAddress _address, dBelief _belief) {
		println(_belief.type);
		IndependentFormulaDistributionsBelief<dBelief> b = IndependentFormulaDistributionsBelief
				.create(dBelief.class, _belief);

		IndependentFormulaDistributions cid = b.getContent();

		for (Entry<String, FormulaDistribution> featureType : cid.entrySet()) {
			println(featureType.getValue().get());
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

	public String[] getSelectedObjectModels() {
		int[] selectedRows = m_objectTable.getSelectedRows();
		String[] models = new String[selectedRows.length];
		int modelCount = 0;
		for (int row : selectedRows) {
			models[modelCount++] = (String) m_objectTableModel.getValueAt(row,
					OBJECT_MODEL_COLUMN);
		}
		return models;
	}

	public void addCone(WorkingMemoryAddress _address, ViewPoint _cone) {
		m_coneTableModel.addRow(new Object[] { _address.id });
		m_cones.put(_address.id, _address);
		pack();
	}

	public void removeCone(WorkingMemoryAddress _address) {
		// TODO Auto-generated method stub
	}

}
