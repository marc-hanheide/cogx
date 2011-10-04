package george.execution.util;

import george.execution.components.GraphicalExecutionManager;

import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
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
import execution.slice.Action;
import execution.slice.actions.PointToObject;
import execution.slice.actions.george.yr3.AnalyzeProtoObject;
import execution.slice.actions.george.yr3.MoveToViewCone;
import execution.util.ActionMonitor;

/**
 * TODO - replace all JDialog crap with JOptionPane things
 * 
 * 
 * @author nah
 * 
 */
public class ActionInterfaceFrame extends JFrame {

	private static final int BELIEF_ID_COLUMN = 0;
	private static final int OBJECT_MODEL_COLUMN = 0;

	@SuppressWarnings("unused")
	private static final int BELIEF_TYPE_COLUMN = 1;
	private static final long serialVersionUID = 1L;
	private JPanel jContentPane = null;

	private JPanel m_buttonPanel = null;
	private JButton m_goButton = null;
	private JButton m_stopButton = null;

	// private JRadioButton m_detectObjectsAction;
	// private JRadioButton m_detectPeopleAction;
	//
	// private JRadioButton m_learnColourAction;
	// private JRadioButton m_learnShapeAction;
	// private JRadioButton m_learnIdentityAction;
	//
	// private JRadioButton m_unlearnColourAction;
	// private JRadioButton m_unlearnShapeAction;
	// private JRadioButton m_unlearnIdentityAction;
	//
	//
	// private JRadioButton m_foregroundModelsAction;
	// private JRadioButton m_backgroundModelsAction;
	// private JRadioButton m_recogniseForegroundedModelsAction;

	private GraphicalExecutionManager m_exeMan;
	private JTable m_beliefTable;
	private DefaultTableModel m_beliefTableModel;
	private JPanel m_beliefsActionPanel;
	// private JPanel m_objectsActionPanel;
	private JPanel m_beliefsPanel;
	// private JPanel m_objectsPanel;
	private JTabbedPane m_tabbedPane;
	// private JTable m_objectTable;
	// private DefaultTableModel m_objectTableModel;

	private JRadioButton m_focusViewConeAction;
	private JRadioButton m_analyseProtoObjectAction;
	private JRadioButton m_pointTpObjectAction;

	
	// private JRadioButton m_askForColourAction;
	// private JRadioButton m_askForShapeAction;
	// private JRadioButton m_askForIdentityAction;
	//
	// private JRadioButton m_askPolarColourAction;
	// private JRadioButton m_askPolarShapeAction;
	// private JRadioButton m_askPolarIdentityAction;
	//
	// private JRadioButton m_askForObjectAction;

	private static final Class<?>[] FEATURE_VALUE_TYPES = {
			ElementaryFormula.class, IntegerFormula.class, FloatFormula.class,
			BooleanFormula.class };

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

	private JPanel getBeliefsPanel() {
		if (m_beliefsPanel == null) {
			m_beliefsPanel = new JPanel();
			m_beliefsPanel.add(new JScrollPane(getBeliefTable()));
			m_beliefsPanel.add(getBeliefsActionPanel());

			getBeliefTable().setPreferredScrollableViewportSize(
					new Dimension(200, 300));
		}
		return m_beliefsPanel;
	}

	// private JPanel getObjectsPanel() {
	// if (m_objectsPanel == null) {
	// m_objectsPanel = new JPanel();
	// m_objectsPanel.add(new JScrollPane(getObjectTable()));
	// m_objectsPanel.add(getObjectsActionPanel());
	// getObjectTable().setPreferredScrollableViewportSize(
	// new Dimension(200, 300));
	// }
	// return m_objectsPanel;
	// }

	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJContentPane() {
		if (jContentPane == null) {
			jContentPane = new JPanel();
			jContentPane.setLayout(new GridLayout(1, 1));
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
			m_tabbedPane.addTab("Beliefs", getBeliefsPanel());
			// m_tabbedPane.addTab("Objects", getObjectsPanel());

		}
		return m_tabbedPane;
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
	private JPanel getBeliefsActionPanel() {
		if (m_beliefsActionPanel == null) {
			m_beliefsActionPanel = new JPanel();
			m_beliefsActionPanel.setLayout(new BoxLayout(m_beliefsActionPanel,
					BoxLayout.Y_AXIS));

			// m_learnColourAction = new JRadioButton("learn colour");
			// m_learnShapeAction = new JRadioButton("learn shape");
			// m_learnIdentityAction = new JRadioButton("learn identity");
			//
			// m_unlearnColourAction = new JRadioButton("unlearn colour");
			// m_unlearnShapeAction = new JRadioButton("unlearn shape");
			// m_unlearnIdentityAction = new JRadioButton("unlearn identity");
			//
			//

			m_focusViewConeAction = new JRadioButton("focus viewcone");
			m_analyseProtoObjectAction = new JRadioButton("analyse protoobject");
			m_pointTpObjectAction = new JRadioButton("point to visualobject");

			// m_askForColourAction = new JRadioButton("ask for colour");
			// m_askForShapeAction = new JRadioButton("ask for shape");
			// m_askForIdentityAction = new JRadioButton("ask for identity");
			//
			// m_askPolarColourAction = new JRadioButton("ask polar colour");
			// m_askPolarShapeAction = new JRadioButton("ask polar shape");
			// m_askPolarIdentityAction = new
			// JRadioButton("ask polar identity");
			//
			// m_askForObjectAction = new JRadioButton(
			// "hey mister, got a BLANK object?");

			ButtonGroup actionGroup = new ButtonGroup();
			actionGroup.add(m_focusViewConeAction);
			actionGroup.add(m_analyseProtoObjectAction);
			actionGroup.add(m_pointTpObjectAction);
			
			// actionGroup.add(m_learnColourAction);
			// actionGroup.add(m_learnShapeAction);
			// actionGroup.add(m_learnIdentityAction);
			// actionGroup.add(m_unlearnColourAction);
			// actionGroup.add(m_unlearnShapeAction);
			// actionGroup.add(m_unlearnIdentityAction);
			//
			// actionGroup.add(m_askForColourAction);
			// actionGroup.add(m_askForShapeAction);
			// actionGroup.add(m_askForIdentityAction);
			// actionGroup.add(m_askPolarColourAction);
			// actionGroup.add(m_askPolarShapeAction);
			// actionGroup.add(m_askPolarIdentityAction);
			// actionGroup.add(m_askForObjectAction);
			//
			m_focusViewConeAction.setSelected(true);

			m_beliefsActionPanel.add(m_focusViewConeAction, null);
			m_beliefsActionPanel.add(m_analyseProtoObjectAction, null);
			m_beliefsActionPanel.add(m_pointTpObjectAction, null);
			
			// m_beliefsActionPanel.add(m_learnColourAction, null);
			// m_beliefsActionPanel.add(m_learnShapeAction, null);
			// m_beliefsActionPanel.add(m_learnIdentityAction, null);
			//
			// m_beliefsActionPanel.add(m_unlearnColourAction, null);
			// m_beliefsActionPanel.add(m_unlearnShapeAction, null);
			// m_beliefsActionPanel.add(m_unlearnIdentityAction, null);
			//
			// m_beliefsActionPanel.add(m_askForColourAction, null);
			// m_beliefsActionPanel.add(m_askForShapeAction, null);
			// m_beliefsActionPanel.add(m_askForIdentityAction, null);
			//
			// m_beliefsActionPanel.add(m_askPolarColourAction, null);
			// m_beliefsActionPanel.add(m_askPolarShapeAction, null);
			// m_beliefsActionPanel.add(m_askPolarIdentityAction, null);
			//
			// m_beliefsActionPanel.add(m_askForObjectAction);
		}
		return m_beliefsActionPanel;
	}

	// /**
	// *
	// * @return javax.swing.JPanel
	// */
	// private JPanel getObjectsActionPanel() {
	// if (m_objectsActionPanel == null) {
	// m_objectsActionPanel = new JPanel();
	//
	// // m_objectsActionPanel.setLayout(new GridBagLayout());
	// m_objectsActionPanel.setLayout(new BoxLayout(m_objectsActionPanel,
	// BoxLayout.Y_AXIS));
	//
	// // m_avsAction = new JRadioButton("visual search in");
	// m_detectObjectsAction = new JRadioButton("detect objects");
	// m_detectPeopleAction = new JRadioButton("detect people");
	// // m_lookForObjectsAction = new JRadioButton("look for objects");
	// // m_lookForPeopleAction = new JRadioButton("look for people");
	// m_foregroundModelsAction = new JRadioButton("foreground models");
	// m_backgroundModelsAction = new JRadioButton("background models");
	// m_recogniseForegroundedModelsAction = new JRadioButton(
	// "recognise foregrounded models");
	//
	// // m_avsAction.setSelected(true);
	// m_detectObjectsAction.setSelected(true);
	//
	// ButtonGroup actionGroup = new ButtonGroup();
	// // actionGroup.add(m_avsAction);
	// actionGroup.add(m_detectObjectsAction);
	// actionGroup.add(m_detectPeopleAction);
	// // actionGroup.add(m_lookForObjectsAction);
	// // actionGroup.add(m_lookForPeopleAction);
	// actionGroup.add(m_foregroundModelsAction);
	// actionGroup.add(m_backgroundModelsAction);
	// actionGroup.add(m_recogniseForegroundedModelsAction);
	//
	// // m_objectsActionPanel.add(m_avsAction, new GridBagConstraints());
	// m_objectsActionPanel.add(m_detectObjectsAction,
	// // new GridBagConstraints());
	// null);
	// m_objectsActionPanel.add(m_detectPeopleAction,
	// // new GridBagConstraints());
	// null);
	// // // m_objectsActionPanel.add(m_lookForObjectsAction,
	// // new GridBagConstraints());
	// // m_objectsActionPanel.add(m_lookForPeopleAction,
	// // new GridBagConstraints());
	// m_objectsActionPanel.add(m_foregroundModelsAction,
	// // new GridBagConstraints());
	// null);
	// m_objectsActionPanel.add(m_backgroundModelsAction,
	// // new GridBagConstraints());
	// null);
	// m_objectsActionPanel.add(m_recogniseForegroundedModelsAction,
	// // new GridBagConstraints());
	// null);
	// }
	// return m_objectsActionPanel;
	// }

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

		println("go()");

		// TODO make more robust to code changes
		int tabIndex = getTabbedPane().getSelectedIndex();

		// if (tabIndex == 1) {
		// // if (m_avsAction.isSelected()) {
		// // runAVS();
		// // } else
		// if (m_detectObjectsAction.isSelected()) {
		// detectObjects();
		//
		// } else if (m_detectPeopleAction.isSelected()) {
		// detectPeople();
		// }
		//
		// // else if (m_lookForObjectsAction.isSelected()) {
		// // lookForObjects();
		// // } else if (m_lookForPeopleAction.isSelected()) {
		// // lookForPeople();
		// // }
		// else if (m_foregroundModelsAction.isSelected()) {
		// foregroundModels();
		// } else if (m_backgroundModelsAction.isSelected()) {
		// backgroundModels();
		// } else if (m_recogniseForegroundedModelsAction.isSelected()) {
		// recogniseForegroundedModels();
		// }
		//
		// } else
		if (tabIndex == 0) {

			if (m_focusViewConeAction.isSelected()) {
				focusViewCone();
			}
			else if (m_analyseProtoObjectAction.isSelected()) {
				analyseProtoObject();
			}
			else if (m_pointTpObjectAction.isSelected()) {
				pointToObject();
			}
			// if (m_learnColourAction.isSelected()) {
			// learnColour();
			// } else if (m_learnShapeAction.isSelected()) {
			// learnShape();
			// } else if (m_learnIdentityAction.isSelected()) {
			// learnIdentity();
			// }
			// if (m_unlearnColourAction.isSelected()) {
			// unlearnColour();
			// } else if (m_unlearnShapeAction.isSelected()) {
			// unlearnShape();
			// } else if (m_unlearnIdentityAction.isSelected()) {
			// unlearnIdentity();
			// } else if (m_askForColourAction.isSelected()) {
			// askForColour();
			// } else if (m_askForShapeAction.isSelected()) {
			// askForShape();
			// } else if (m_askForIdentityAction.isSelected()) {
			// askForIdentity();
			// } else if (m_askPolarColourAction.isSelected()) {
			// askPolarColour();
			// } else if (m_askPolarShapeAction.isSelected()) {
			// askPolarShape();
			// } else if (m_askPolarIdentityAction.isSelected()) {
			// askPolarIdentity();
			//
			// } else if (m_askForObjectAction.isSelected()) {
			// askForObject();
			// }

		} else {
			throw new RuntimeException("No tab selected apparently... "
					+ m_tabbedPane.getSelectedIndex());
		}
	}

	private String getSelectedBeliefID() {
		String beliefID = null;
		int selectedRow = m_beliefTable.getSelectedRow();
		if (selectedRow != -1) {
			Object beliefIDVal = m_beliefTableModel.getValueAt(selectedRow,
					BELIEF_ID_COLUMN);
			assert (beliefIDVal != null);
			beliefID = (String) beliefIDVal;
		}
		return beliefID;
	}

	private void learnColour() throws CASTException {
		String beliefID = getSelectedBeliefID();
		if (beliefID != null) {
			String colour = getFeatureValue(beliefID, "colour", "learnt");
			if (colour != null) {
				m_exeMan.learnColour(new WorkingMemoryAddress(beliefID,
						"binder"), colour, new MonitorPanel());
			}
		}
	}

	private void learnShape() throws CASTException {
		String beliefID = getSelectedBeliefID();
		if (beliefID != null) {
			String colour = getFeatureValue(beliefID, "shape", "learnt");
			if (colour != null) {
				m_exeMan.learnShape(
						new WorkingMemoryAddress(beliefID, "binder"), colour,
						new MonitorPanel());
			}
		}
	}

	private void learnIdentity() throws CASTException {
		String beliefID = getSelectedBeliefID();
		if (beliefID != null) {
			String colour = getFeatureValue(beliefID, "identity", "learnt");
			if (colour != null) {
				m_exeMan.learnIdentity(new WorkingMemoryAddress(beliefID,
						"binder"), colour, new MonitorPanel());
			}
		}
	}

	private void unlearnColour() throws CASTException {
		String beliefID = getSelectedBeliefID();
		if (beliefID != null) {
			String colour = getFeatureValue(beliefID, "colour", "unlearnt");
			if (colour != null) {
				m_exeMan.unlearnColour(new WorkingMemoryAddress(beliefID,
						"binder"), colour, new MonitorPanel());
			}
		}
	}

	private void unlearnShape() throws CASTException {
		String beliefID = getSelectedBeliefID();
		if (beliefID != null) {
			String colour = getFeatureValue(beliefID, "shape", "unlearnt");
			if (colour != null) {
				m_exeMan.unlearnShape(new WorkingMemoryAddress(beliefID,
						"binder"), colour, new MonitorPanel());
			}
		}
	}

	private void unlearnIdentity() throws CASTException {
		String beliefID = getSelectedBeliefID();
		if (beliefID != null) {
			String colour = getFeatureValue(beliefID, "identity", "unlearnt");
			if (colour != null) {
				m_exeMan.unlearnIdentity(new WorkingMemoryAddress(beliefID,
						"binder"), colour, new MonitorPanel());
			}
		}
	}

	private void askForColour() throws CASTException {
		String beliefID = getSelectedBeliefID();
		if (beliefID != null) {
			m_exeMan.askForColour(new WorkingMemoryAddress(beliefID, "binder"),
					new MonitorPanel());
		}
	}

	private void focusViewCone() throws CASTException {

		// TODO assume for now that this is a viewcone belief
		String beliefID = getSelectedBeliefID();

		if (beliefID != null) {
			m_exeMan.executeSingleBeliefAction(new WorkingMemoryAddress(
					beliefID, "binder"), new MonitorPanel(),
					MoveToViewCone.class);
		}
	}

	private void analyseProtoObject() throws CASTException {

		// TODO assume for now that this is a protoobject belief
		String beliefID = getSelectedBeliefID();

		if (beliefID != null) {
			println("go proto");
			m_exeMan.executeSingleBeliefAction(new WorkingMemoryAddress(
					beliefID, "binder"), new MonitorPanel(),
					AnalyzeProtoObject.class);
		}
	}


	private void pointToObject() throws CASTException {

		// TODO assume for now that this is a visualobject belief
		String beliefID = getSelectedBeliefID();

		if (beliefID != null) {
			println("go pointy");
			m_exeMan.executeSingleBeliefAction(new WorkingMemoryAddress(
					beliefID, "binder"), new MonitorPanel(),
					PointToObject.class);
		}
	}

	
	private void askForShape() throws CASTException {
		String beliefID = getSelectedBeliefID();
		if (beliefID != null) {
			m_exeMan.askForShape(new WorkingMemoryAddress(beliefID, "binder"),
					new MonitorPanel());
		} else {
			println("BELIEF IF IS NULL");
		}
	}

	private void askForIdentity() throws CASTException {
		String beliefID = getSelectedBeliefID();
		if (beliefID != null) {
			m_exeMan.askForShape(new WorkingMemoryAddress(beliefID, "binder"),
					new MonitorPanel());
		}
	}

	private void askPolarColour() throws CASTException {
		String beliefID = getSelectedBeliefID();
		if (beliefID != null) {
			m_exeMan.askPolarColour(
					new WorkingMemoryAddress(beliefID, "binder"),
					getFeatureValue(beliefID, "colour", "asked for"),
					new MonitorPanel());
		}
	}

	private void askPolarShape() throws CASTException {
		String beliefID = getSelectedBeliefID();
		if (beliefID != null) {
			m_exeMan.askPolarShape(
					new WorkingMemoryAddress(beliefID, "binder"),
					getFeatureValue(beliefID, "shape", "asked for"),
					new MonitorPanel());
		}
	}

	private void askPolarIdentity() throws CASTException {
		String beliefID = getSelectedBeliefID();
		if (beliefID != null) {
			m_exeMan.askPolarIdentity(new WorkingMemoryAddress(beliefID,
					"binder"),
					getFeatureValue(beliefID, "identity", "asked for"),
					new MonitorPanel());
		}
	}

	private void askForObject() throws CASTException {

		String feature = (String) JOptionPane.showInputDialog(this,
				"What feature do you want? (color, shape, identity)");

		if (feature != null) {
			String value = (String) JOptionPane.showInputDialog(this,
					"What value do you want for this?");
			if (value != null) {
				m_exeMan.askForObject(feature, value, new MonitorPanel());
			}
		}
	}

	private String getFeatureValue(String _beliefID, String _concept, String _op) {
		String message = "What " + _concept + " should be " + _op
				+ " for belief " + _beliefID + "?";

		return (String) JOptionPane.showInputDialog(this, message);
	}

	// /**
	// * Popup
	// */
	// private void askForFeature() {
	// int selectedRow = m_beliefTable.getSelectedRow();
	// if (selectedRow != -1) {
	// Object beliefIDVal = m_beliefTableModel.getValueAt(selectedRow,
	// BELIEF_ID_COLUMN);
	// assert (beliefIDVal != null);
	// final String beliefID = (String) beliefIDVal;
	//
	// final JDialog dialog = new JDialog(this);
	// dialog.setLayout(new FlowLayout());
	// dialog.add(new JLabel("What feature do you want to ask about?"));
	//
	// final JTextField textfield = new JTextField(10);
	// dialog.add(textfield);
	//
	// ActionListener submit = new ActionListener() {
	//
	// @Override
	// public void actionPerformed(ActionEvent _e) {
	// submitFeatureQuery(beliefID, dialog, textfield);
	// dialog.setVisible(false);
	// }
	// };
	//
	// textfield.addActionListener(submit);
	//
	// JButton goButton = new JButton("Go!");
	// goButton.addActionListener(submit);
	//
	// dialog.add(goButton);
	// dialog.pack();
	// dialog.setVisible(true);
	// }
	// }

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

	// /**
	// * @param beliefID
	// * @param dialog
	// * @param textfield
	// */
	// private void submitFeatureQuery(final String beliefID,
	// final JDialog dialog, final JTextField textfield) {
	// dialog.setVisible(false);
	// String featureType = textfield.getText();
	// if (featureType.length() > 0) {
	// try {
	// m_exeMan.triggerAskForFeatureAction(new
	// WorkingMemoryAddress(beliefID,"binder"), featureType,
	// new MonitorPanel());
	// } catch (CASTException e) {
	// m_exeMan.logException(e);
	// }
	// }
	// }

	// /**
	// * @throws CASTException
	// */
	// private void detectObjects() throws CASTException {
	// m_exeMan.triggerDetectObjects(getSelectedObjectModels(),
	// new MonitorPanel());
	// }
	//
	// /**
	// * @throws CASTException
	// */
	// private void foregroundModels() throws CASTException {
	// m_exeMan.foregroundModels(getSelectedObjectModels(), new MonitorPanel());
	// }
	//
	// /**
	// * @throws CASTException
	// */
	// private void backgroundModels() throws CASTException {
	// m_exeMan.backgroundModels(getSelectedObjectModels(), new MonitorPanel());
	// }

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

	// /**
	// * This method initializes m_objectTable
	// *
	// * @return javax.swing.JTable
	// */
	// private JTable getObjectTable() {
	// if (m_objectTable == null) {
	// m_objectTable = new JTable(1, 2);
	// m_objectTableModel = new DefaultTableModel(
	// new String[] { "model" }, 0);
	// m_objectTable.setModel(m_objectTableModel);
	// }
	// return m_objectTable;
	// }
	//
	// public void setObjectModels(String[] _models) {
	// for (String model : _models) {
	// m_objectTableModel.addRow(new Object[] { model });
	// }
	// }

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
		try {
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
		catch(ClassCastException e) {
			m_exeMan.logException("can't collect this type of belief",e);
		}

	}

	private void println(Object _o) {
		m_exeMan.println(_o);
	}

	public void removeBelief(WorkingMemoryAddress _address) {
		// TODO Auto-generated method stub

	}

	// public String[] getSelectedObjectModels() {
	// int[] selectedRows = m_objectTable.getSelectedRows();
	// String[] models = new String[selectedRows.length];
	// int modelCount = 0;
	// for (int row : selectedRows) {
	// models[modelCount++] = (String) m_objectTableModel.getValueAt(row,
	// OBJECT_MODEL_COLUMN);
	// }
	// return models;
	// }

}
