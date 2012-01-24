package dora.execution.util;

import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.GridBagConstraints;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.JTabbedPane;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.table.DefaultTableModel;

import SpatialData.Place;
import SpatialData.PlaceStatus;
import VisionData.Person;
import VisionData.VisualObject;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;

import comadata.ComaRoom;

import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import dora.execution.components.GraphicalExecutionManager;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.utils.BeliefUtils;
import eu.cogx.perceptmediator.dora.PersonTransferFunction;
import eu.cogx.perceptmediator.dora.VisualObjectTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.slice.actions.AskForIdentity;
import execution.slice.actions.AskPolarIdentity;
import execution.slice.actions.TurnToHuman;
import execution.util.ActionMonitor;

/**
* This code was edited or generated using CloudGarden's Jigloo
* SWT/Swing GUI Builder, which is free for non-commercial
* use. If Jigloo is being used commercially (ie, by a corporation,
* company or business for any purpose whatever) then you
* should purchase a license for each developer using Jigloo.
* Please visit www.cloudgarden.com for details.
* Use of Jigloo implies acceptance of these licensing terms.
* A COMMERCIAL LICENSE HAS NOT BEEN PURCHASED FOR
* THIS MACHINE, SO JIGLOO OR THIS CODE CANNOT BE USED
* LEGALLY FOR ANY CORPORATE OR COMMERCIAL PURPOSE.
*/
/**
 * Every time I look at this code I die inside a little.
 * 
 * @author nah
 * 
 */
public class ActionInterfaceFrame extends JFrame {

	{
		//Set Look & Feel
		try {
			javax.swing.UIManager.setLookAndFeel("com.sun.java.swing.plaf.gtk.GTKLookAndFeel");
		} catch(Exception e) {
			e.printStackTrace();
		}
	}


	private static final String PERSONTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(Person.class);
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
	private JRadioButton m_generateConesForRoomAction;
	private JRadioButton m_processConeAction;
	// private JRadioButton m_processConesAtPlaceAction;

	private JRadioButton m_goAction;
	private JRadioButton m_detectObjectsAction;
	private JRadioButton m_detectPeopleAction;
	private JRadioButton m_lookForObjectsAction;
	private JRadioButton m_lookForPeopleAction;

	private JRadioButton m_generateConesOnObject;
	private JRadioButton m_foregroundModelsAction;
	private JRadioButton m_backgroundModelsAction;
	private JRadioButton m_recogniseForegroundedModelsAction;
	
	private JRadioButton jStatusExecuting;
	private JRadioButton jStatusFailed;
	private JRadioButton jStatusSuccess;
	private JPanel jPanelStatus;
	private JTextField jStatusText;

	private JRadioButton m_reportPosition;

	// private JRadioButton m_learnIdentityAction;
	private JRadioButton m_askForIdentityAction;
	private JRadioButton m_askPolarIdentityAction;
	private JRadioButton m_engageWithHuman;

	private GraphicalExecutionManager m_exeMan;
	private JPanel m_objectsActionPanel;
	private JPanel m_placesPanel;
	private JPanel m_objectsPanel;
	private JTabbedPane m_tabbedPane;
	private JTable m_objectTable;
	private DefaultTableModel m_objectTableModel;
	private JTable m_roomTable;
	private DefaultTableModel m_roomTableModel;
	private JTable m_objectBeliefsTable;
	private DefaultTableModel m_objectBeliefsTableModel;

	private static final int CONE_ADDR_COLUMN = 3;
	private static final int PLACE_ADDR_COLUMN = 2;
	private static final int ROOM_ADDR_COLUMN = 1;
	private static final int OBJ_ADDR_COLUMN = 1;

	private static final String PLACETYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(Place.class));

	private static final String COMAROOMTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(ComaRoom.class));

	// private static final String VIEWPOINTTYPE = SimpleDiscreteTransferFunction
	// 		.getBeliefTypeFromCastType(CASTUtils.typeName(ViewPoint.class));
	private static final String VIEWPOINTTYPE = "conegroup";

	private static final String VISUALOBJECTTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(VisualObject.class));

	private static final String SEPARATOR = " ";

	private static String toAddressString(WorkingMemoryAddress _wma) {
		return _wma.id + SEPARATOR + _wma.subarchitecture;
	}

	private static WorkingMemoryAddress addressFromString(String _address) {
		String[] split = _address.split(SEPARATOR);
		assert (split.length == 2);
		return new WorkingMemoryAddress(split[0], split[1]);
	}

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
		this.setContentPane(getJContentPane());
		this.setTitle("Robot Actions (Don't you just hate writing GUIs?)");
	}

	private JPanel getPlacesPanel() {
		if (m_placesPanel == null) {
			m_placesPanel = new JPanel(new GridLayout(1, 0));
			// m_placesPanel.setLayout(new GridLayout(0, 1));
			m_placesPanel.add(new JScrollPane(getPlaceTable()));
			m_placesPanel.add(new JScrollPane(getConeTable()));
			m_placesPanel.add(new JScrollPane(getRoomTable()));

			getPlaceTable().setPreferredScrollableViewportSize(
					new Dimension(200, 300));

			getConeTable().setPreferredScrollableViewportSize(
					new Dimension(200, 300));

			getRoomTable().setPreferredScrollableViewportSize(
					new Dimension(200, 300));

			m_placesPanel.add(getPlacesActionPanel());
		}
		return m_placesPanel;
	}

	private JPanel getObjectsPanel() {
		if (m_objectsPanel == null) {
			m_objectsPanel = new JPanel(new GridLayout(1, 0));
			m_objectsPanel.add(new JScrollPane(getObjectBeliefsTable()));
			m_objectsPanel.add(new JScrollPane(getObjectTable()));
			m_objectsPanel.add(getObjectsActionPanel());
			getObjectTable().setPreferredScrollableViewportSize(
					new Dimension(200, 300));
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
			m_tabbedPane.addTab("Spatial", getPlacesPanel());
			m_tabbedPane.addTab("Vision", getObjectsPanel());
			// m_tabbedPane.addTab("Beliefs", getBeliefsPanel());

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
			FlowLayout m_buttonPanelLayout = new FlowLayout();
			m_buttonPanel.setLayout(m_buttonPanelLayout);
			m_buttonPanel.add(getJPanelStatus());
			m_buttonPanel.add(getGoButton());
			m_buttonPanel.add(getM_stopButton());
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
			m_placesActionPanel.setLayout(new GridLayout(0, 1));

			m_goAction = new JRadioButton("go to place");
			// m_generateConesForPlacesAction = new JRadioButton(
			// 		"generate cones in selected places");

			m_generateConesForRoomAction = new JRadioButton(
					"generate cones in selected room");

			m_processConeAction = new JRadioButton("process selected cone");
			// m_processConesAtPlaceAction = new JRadioButton(
			// 		"process all cones at place");

			m_askForIdentityAction = new JRadioButton("ask for identity");
			m_askPolarIdentityAction = new JRadioButton("ask polar identity");

			m_goAction.setSelected(true);

			ButtonGroup actionGroup = new ButtonGroup();
			actionGroup.add(m_goAction);
			// actionGroup.add(m_generateConesForPlacesAction);
			actionGroup.add(m_generateConesForRoomAction);
			actionGroup.add(m_processConeAction);
			// actionGroup.add(m_processConesAtPlaceAction);
			actionGroup.add(m_askForIdentityAction);
			actionGroup.add(m_askPolarIdentityAction);

			m_placesActionPanel.add(m_goAction);
			// m_placesActionPanel.add(m_generateConesForPlacesAction);
			m_placesActionPanel.add(m_generateConesForRoomAction);
			m_placesActionPanel.add(m_processConeAction);
			// m_placesActionPanel.add(m_processConesAtPlaceAction);
			m_placesActionPanel.add(m_askForIdentityAction);
			m_placesActionPanel.add(m_askPolarIdentityAction);

		}
		return m_placesActionPanel;
	}

	/**
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getObjectsActionPanel() {
		if (m_objectsActionPanel == null) {
			m_objectsActionPanel = new JPanel(new GridLayout(0, 1));

			m_detectObjectsAction = new JRadioButton("detect objects");
			m_detectPeopleAction = new JRadioButton("detect people");
			m_lookForObjectsAction = new JRadioButton("look for objects");
			m_lookForPeopleAction = new JRadioButton("look for people");
            m_generateConesOnObject = new JRadioButton("generate cones on selected object");
			// m_foregroundModelsAction = new JRadioButton("foreground models");
			// m_backgroundModelsAction = new JRadioButton("background models");
			m_recogniseForegroundedModelsAction = new JRadioButton(
					"recognise foregrounded models");

			m_reportPosition = new JRadioButton("report position");
			m_engageWithHuman = new JRadioButton("engage with human");

			// m_avsAction.setSelected(true);
			m_detectObjectsAction.setSelected(true);

			ButtonGroup actionGroup = new ButtonGroup();
			actionGroup.add(m_detectObjectsAction);
			actionGroup.add(m_detectPeopleAction);
			actionGroup.add(m_lookForObjectsAction);
			actionGroup.add(m_lookForPeopleAction);
            actionGroup.add(m_generateConesOnObject);
			// actionGroup.add(m_foregroundModelsAction);
			// actionGroup.add(m_backgroundModelsAction);
			actionGroup.add(m_recogniseForegroundedModelsAction);
			actionGroup.add(m_reportPosition);
			actionGroup.add(m_engageWithHuman);

			m_objectsActionPanel.add(m_detectObjectsAction,
					new GridBagConstraints());
			m_objectsActionPanel.add(m_detectPeopleAction,
					new GridBagConstraints());
			m_objectsActionPanel.add(m_lookForObjectsAction,
					new GridBagConstraints());
			m_objectsActionPanel.add(m_lookForPeopleAction,
					new GridBagConstraints());
			m_objectsActionPanel.add(m_generateConesOnObject,
					new GridBagConstraints());
			// m_objectsActionPanel.add(m_foregroundModelsAction,
			// 		new GridBagConstraints());
			// m_objectsActionPanel.add(m_backgroundModelsAction,
			// 		new GridBagConstraints());

			m_objectsActionPanel.add(m_recogniseForegroundedModelsAction,
					new GridBagConstraints());
			m_objectsActionPanel
					.add(m_reportPosition, new GridBagConstraints());
			m_objectsActionPanel.add(m_engageWithHuman,
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
			jStatusText.setText("completed action "+_action.getClass().getSimpleName() + " with  " + _action.success.toString());
			setStatus(_action);
			m_exeMan.println("Action complete");
		}


	}
	public  void setStatus(Action _action) {
		jStatusText.setText("action " + _action.getClass().getSimpleName() + " status=" + _action.status.toString()+ ", success="+_action.success.toString());
		jStatusSuccess.setSelected(_action.success==TriBool.TRITRUE);
		jStatusFailed.setSelected(_action.success==TriBool.TRIFALSE);
		jStatusExecuting.setSelected(_action.success==TriBool.TRIINDETERMINATE);
	}

	private void go() throws CASTException {

		// TODO make more robust to code changes
		int tabIndex = getTabbedPane().getSelectedIndex();

		m_exeMan.println("go() called: " + tabIndex);

		if (tabIndex == 0) {
			// if (m_generateConesForPlacesAction.isSelected()) {
			// 	generateConesForPlaces();
			// } else 
            if (m_processConeAction.isSelected()) {
				processCone();
			} else if (m_goAction.isSelected()) {
				goToPlace();
			} else if (m_generateConesForRoomAction.isSelected()) {
				generateConesInRoom();
			// } else if (m_processConesAtPlaceAction.isSelected()) {
			// 	processConesAtPlace();
			} else if (m_askForIdentityAction.isSelected()) {
				askForIdentity();
			} else if (m_askPolarIdentityAction.isSelected()) {
				askPolarIdentity();
			}

		} else if (tabIndex == 1) {

			if (m_detectObjectsAction.isSelected()) {
				detectObjects();

			} else if (m_detectPeopleAction.isSelected()) {
				detectPeople();
			}

			else if (m_lookForObjectsAction.isSelected()) {
				lookForObjects();
			} else if (m_lookForPeopleAction.isSelected()) {
				lookForPeople();
			} else if (m_generateConesOnObject.isSelected()) {
				generateConesOnObject();
			// } else if (m_foregroundModelsAction.isSelected()) {
			// 	foregroundModels();
			// } else if (m_backgroundModelsAction.isSelected()) {
			// 	backgroundModels();
			} else if (m_recogniseForegroundedModelsAction.isSelected()) {
				recogniseForegroundedModels();
			} else if (m_reportPosition.isSelected()) {
				reportPosition();
			} else if (m_engageWithHuman.isSelected()) {
				engageWithHuman();
			}
			// } else if (tabIndex ==2) {
			// if (m_askForFeatureAction.isSelected()) {
			// askForFeature();
			// } else if (m_testFeatureValueAction.isSelected()) {
			// testFeatureValue();
			// }
		} else {
			throw new RuntimeException("No tab selected apparently... "
					+ m_tabbedPane.getSelectedIndex());
		}
		
		
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

	/**
	 * @throws CASTException
	 */
	private long[] getSelectedPlaces() {
		int[] selectedRows = m_placeTable.getSelectedRows();
		long[] placeIDs = new long[selectedRows.length];

		for (int i = 0; i < selectedRows.length; ++i) {

			Object placeIDVal = m_placeTableModel.getValueAt(selectedRows[i],
					PLACE_ID_COLUMN);
			assert (placeIDVal != null);
			placeIDs[i] = (Long) placeIDVal;
		}

		return placeIDs;
	}

	/**
	 * @throws CASTException
	 */
	private void generateConesForPlaces() throws CASTException {

		// long[] placeIDs = getSelectedPlaces();
		// if (placeIDs.length > 0) {

		// 	m_exeMan.triggerConeGeneration((String) JOptionPane
		// 			.showInputDialog(this,
		// 					"What object should the cones be generated for?"),
		// 			placeIDs, new MonitorPanel());

		// } else {
		// 	m_exeMan.println("no places selected, doing nothing");
		// }
	}

	/**
	 * @throws CASTException
	 */
	private void generateConesInRoom() throws CASTException {

		int selectedRow = m_roomTable.getSelectedRow();
		if (selectedRow >= 0) {

			WorkingMemoryAddress roomBeliefAddress = addressFromString((String) m_roomTableModel
					.getValueAt(selectedRow, ROOM_ADDR_COLUMN));

			IndependentFormulaDistributionsBelief<GroundedBelief> gb = IndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, m_exeMan.getMemoryEntry(
							roomBeliefAddress, GroundedBelief.class));

			ComaRoom room = BeliefUtils.getMostRecentPerceptAncestor(m_exeMan,
					gb, ComaRoom.class);

			m_exeMan.triggerConeGeneration((String) JOptionPane
					.showInputDialog(this,
							"What object should the cones be generated for?"),
					room.roomId, new MonitorPanel());

		} else {
			m_exeMan.println("no room selected, doing nothing");
		}
	}

	/**
	 * @throws CASTException
	 */
	private void generateConesOnObject() throws CASTException {

		int selectedRow = m_objectBeliefsTable.getSelectedRow();
		if (selectedRow >= 0) {

			WorkingMemoryAddress objectBeliefAddress = addressFromString((String) m_objectBeliefsTableModel
					.getValueAt(selectedRow, OBJ_ADDR_COLUMN));

			IndependentFormulaDistributionsBelief<GroundedBelief> ob = IndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, m_exeMan.getMemoryEntry(
							objectBeliefAddress, GroundedBelief.class));

            String support_label = ob.getContent().get(VisualObjectTransferFunction.LABEL_ID)
				.getDistribution().getMostLikely().getProposition();
            WMPointer roomPointer = WMPointer.create(ob.getContent()
                                                     .get("related-to").getDistribution()
                                                     .getMostLikely().get());
			IndependentFormulaDistributionsBelief<GroundedBelief> rb = IndependentFormulaDistributionsBelief
                .create(GroundedBelief.class, m_exeMan.getMemoryEntry(
                            roomPointer.getVal(), GroundedBelief.class));


			ComaRoom room = BeliefUtils.getMostRecentPerceptAncestor(m_exeMan,
					rb, ComaRoom.class);


			m_exeMan.triggerConeGenerationOnObject((String) JOptionPane
                                           .showInputDialog(this, "What object should the cones be generated for?"), 
                                           support_label, objectBeliefAddress, room.roomId, new MonitorPanel());

		} else {
			m_exeMan.println("no object selected, doing nothing");
		}
	}

	private WorkingMemoryAddress getSelectedRoomBeliefAddress() {
		int selectedRow = m_roomTable.getSelectedRow();
		if (selectedRow >= 0) {
			return addressFromString((String) m_roomTableModel.getValueAt(
					selectedRow, ROOM_ADDR_COLUMN));
		} else {
			return null;
		}

	}

	private WorkingMemoryAddress getSelectedObjectBeliefAddress() {
		int selectedRow = m_objectBeliefsTable.getSelectedRow();
		if (selectedRow >= 0) {
			return addressFromString((String) m_objectBeliefsTableModel
					.getValueAt(selectedRow, ROOM_ADDR_COLUMN));
		} else {
			return null;
		}

	}

	/**
	 * @throws CASTException
	 */
	private void processCone() throws CASTException {
		int selectedRow = m_coneTable.getSelectedRow();
		if (selectedRow != -1) {
			Object beliefAddrVal = m_coneTableModel.getValueAt(selectedRow,
					CONE_ADDR_COLUMN);
			assert (beliefAddrVal != null);
			WorkingMemoryAddress beliefAddress = addressFromString((String) beliefAddrVal);

			IndependentFormulaDistributionsBelief<GroundedBelief> gb = IndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, m_exeMan.getMemoryEntry(
							beliefAddress, GroundedBelief.class));

			// ViewPoint GBs come directly from data, no percept belief middle
			// ground

			// WorkingMemoryPointer perceptPointer = BeliefUtils
			// 		.getMostRecentAncestorPointer(gb);

			// Get the ID 
			Formula coneGroupIDProperty = gb.getContent().get("id").getDistribution().firstValue();

			m_exeMan.triggerProccesCone(beliefAddress, coneGroupIDProperty.getInteger(),
					new MonitorPanel());
		} else {
			m_exeMan.println("no cone selected, doing nothing");
		}
	}

	/**
	 * @throws CASTException
	 */
	private void reportPosition() throws CASTException {
		int selectedRow = m_objectBeliefsTable.getSelectedRow();
		if (selectedRow != -1) {
			Object objectBelief = m_objectBeliefsTableModel.getValueAt(
					selectedRow, OBJ_ADDR_COLUMN);
			assert (objectBelief != null);
			m_exeMan.triggerReportPositionAction(
					addressFromString((String) objectBelief),
					new MonitorPanel());
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
	private void processConesAtPlace() throws CASTException {
		int selectedRow = m_placeTable.getSelectedRow();
		if (selectedRow != -1) {
			Object placeIDVal = m_placeTableModel.getValueAt(selectedRow,
					PLACE_ID_COLUMN);
			assert (placeIDVal != null);
			long placeID = (Long) placeIDVal;
			m_exeMan.triggerProcessConesAtPlace(placeID, (String) JOptionPane
					.showInputDialog(this,
							"What object should the cones be processed for?"),
					new MonitorPanel());
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

	private void lookForPeople() throws CASTException {
		m_exeMan.triggerLookForPeople(new MonitorPanel());
	}

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
			m_placeTable = new JTable(1, 3);
			m_placeTableModel = new DefaultTableModel(new String[] {
					"place id", "status", "belief address" }, 0);
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
			m_coneTable = new JTable(1, 4);
			m_coneTableModel = new DefaultTableModel(new String[] { "object",
					"prob", "place id", "belief address" }, 0);
			m_coneTable.setModel(m_coneTableModel);
		}
		return m_coneTable;
	}

	/**
	 * This method initializes m_roomTable
	 * 
	 * @return javax.swing.JTable
	 */
	private JTable getRoomTable() {
		if (m_roomTable == null) {
			m_roomTable = new JTable(1, 2);
			m_roomTableModel = new DefaultTableModel(new String[] { "room id",
					"belief address" }, 0);
			m_roomTable.setModel(m_roomTableModel);

		}
		return m_roomTable;
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

	/**
	 * This method initializes m_objectTable
	 * 
	 * @return javax.swing.JTable
	 */
	private JTable getObjectBeliefsTable() {
		if (m_objectBeliefsTable == null) {
			m_objectBeliefsTable = new JTable(1, 2);
			m_objectBeliefsTableModel = new DefaultTableModel(new String[] {
					"ident", "belief addr" }, 0);
			m_objectBeliefsTable.setModel(m_objectBeliefsTableModel);
		}
		return m_objectBeliefsTable;
	}

	public void setObjectModels(String[] _models) {
		for (String model : _models) {
			m_objectTableModel.addRow(new Object[] { model });
		}
	}

	private void println(Object _o) {
		m_exeMan.println(_o);
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

	public void addConeBelief(WorkingMemoryAddress _address,
			IndependentFormulaDistributionsBelief<dBelief> _belief)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {

        try {
		String label = _belief.getContent().get("cg-label")
				.getDistribution().getMostLikely().getProposition();

		Double prob = _belief.getContent().get("p-visible")
				.getDistribution().getMostLikely().getDouble();

		String beliefAddr = toAddressString(_address);

		WMPointer pointer = WMPointer.create(_belief.getContent()
                                             .get("cg-related-to").getDistribution()
                                             .getMostLikely().get());

		// IndependentFormulaDistributionsBelief<GroundedBelief> relatedBelief = IndependentFormulaDistributionsBelief
		// 		.create(GroundedBelief.class, m_exeMan.getMemoryEntry(
		// 				pointer.get().pointer, GroundedBelief.class));
        
		// long placeID = placeBelief.getContent()
		// 		.get(PlaceTransferFunction.PLACE_ID_ID).getDistribution()
		// 		.getMostLikely().getInteger();
		String relatedAddress = toAddressString(pointer.get().pointer);

		m_coneTableModel
				.addRow(new Object[] { label, prob, relatedAddress, beliefAddr });
        } catch (Exception e) {
            e.printStackTrace();
        }

		// pack();

	}

	public void removeCone(WorkingMemoryAddress _address) {
		// for (int row = 0; row < m_coneTableModel.getRowCount(); row++) {
		// String coneID = (String) m_coneTableModel.getValueAt(row,
		// CONE_ID_COLUMN);
		// if (coneID.equals(_address.id)) {
		// m_coneTableModel.removeRow(row);
		// m_cones.remove(coneID);
		// return;
		// }
		// }

	}

	public void addBelief(WorkingMemoryAddress _address,
			IndependentFormulaDistributionsBelief<dBelief> _belief)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {

		if (_belief.getType().equals(PLACETYPE)) {
			addPlaceBelief(_address, _belief);
		} else if (_belief.getType().equals(VIEWPOINTTYPE)) {
			addConeBelief(_address, _belief);
		} else if (_belief.getType().equals(COMAROOMTYPE)) {
			addRoomBelief(_address, _belief);
		} else if (_belief.getType().equals(PERSONTYPE)) {
			addPersonBelief(_address, _belief);
		} else if (_belief.getType().equals(VISUALOBJECTTYPE)) {
			addObjectBelief(_address, _belief);
		}
		// pack();

	}

	private boolean removeConeBelief(WorkingMemoryAddress _address) {

		// println("input addr: " + CASTUtils.toString(_address));

		for (int row = 0; row < m_coneTableModel.getRowCount(); row++) {
			WorkingMemoryAddress coneAddr = addressFromString((String) m_coneTableModel
					.getValueAt(row, CONE_ADDR_COLUMN));

			// println("table addr: " + CASTUtils.toString(coneAddr));

			if (coneAddr.equals(_address)) {
				// println("removed it");
				m_coneTableModel.removeRow(row);
				// pack();
				return true;
			}
		}
		return false;
	}

	private boolean removePlaceBelief(WorkingMemoryAddress _address) {
		for (int row = 0; row < m_placeTableModel.getRowCount(); row++) {
			WorkingMemoryAddress placeAddr = addressFromString((String) m_placeTableModel
					.getValueAt(row, PLACE_ADDR_COLUMN));
			if (placeAddr.equals(_address)) {
				m_placeTableModel.removeRow(row);
				// pack();
				return true;
			}
		}
		return false;
	}

	private boolean removeObjectBelief(WorkingMemoryAddress _address) {
		for (int row = 0; row < m_objectBeliefsTableModel.getRowCount(); row++) {
			WorkingMemoryAddress objectAddr = addressFromString((String) m_objectBeliefsTableModel
					.getValueAt(row, OBJ_ADDR_COLUMN));
			if (objectAddr.equals(_address)) {
				m_objectBeliefsTableModel.removeRow(row);
				// pack();
				return true;
			}
		}
		return false;
	}

	private boolean removePersonBelief(WorkingMemoryAddress _address) {
		for (int row = 0; row < m_objectBeliefsTableModel.getRowCount(); row++) {
			
			WorkingMemoryAddress objectAddr = addressFromString((String) m_objectBeliefsTableModel
					.getValueAt(row, OBJ_ADDR_COLUMN));
			if (objectAddr.equals(_address)) {
				m_objectBeliefsTableModel.removeRow(row);
				// pack();
				return true;
			}
		}
		return false;
	}

	private boolean removeRoomBelief(WorkingMemoryAddress _address) {
		for (int row = 0; row < m_roomTableModel.getRowCount(); row++) {
			WorkingMemoryAddress roomAddr = addressFromString((String) m_roomTableModel
					.getValueAt(row, ROOM_ADDR_COLUMN));
			if (roomAddr.equals(_address)) {
				m_roomTableModel.removeRow(row);
				pack();
				return true;
			}
		}
		return false;
	}

	public void removeBelief(WorkingMemoryAddress _address) {
		if (removeConeBelief(_address)) {
			return;
		} else if (removePlaceBelief(_address)) {
			return;
		} else if (removeRoomBelief(_address)) {
			return;
		} else if (removeObjectBelief(_address)) {
			return;
		} else if (removePersonBelief(_address)) {
			return;
		}

	}

	public void addPlaceBelief(WorkingMemoryAddress _address,
			IndependentFormulaDistributionsBelief<dBelief> _belief) {

		long placeID = _belief.getContent()
				.get(PlaceTransferFunction.PLACE_ID_ID).getDistribution()
				.getMostLikely().getInteger();

		String status = _belief.getContent()
				.get(PlaceTransferFunction.PLACE_STATUS_ID).getDistribution()
				.getMostLikely().getProposition();

		String addr = toAddressString(_address);

		m_placeTableModel.addRow(new Object[] { placeID, status, addr });
		// pack();
	}

	public void addRoomBelief(WorkingMemoryAddress _address,
			IndependentFormulaDistributionsBelief<dBelief> _belief) {

		long roomID = _belief.getContent()
				.get(ComaRoomTransferFunction.ROOM_ID).getDistribution()
				.getMostLikely().getInteger();

		String addr = toAddressString(_address);
		m_roomTableModel.addRow(new Object[] { roomID, addr });
		pack();
	}

	public void addObjectBelief(WorkingMemoryAddress _address,
			IndependentFormulaDistributionsBelief<dBelief> _belief) {

		FormulaDistribution distribution = _belief.getContent().get(
				VisualObjectTransferFunction.LABEL_ID);

		if (distribution != null) {
			String ident = distribution.getDistribution().getMostLikely()
					.getProposition();

			String addr = toAddressString(_address);
			m_objectBeliefsTableModel.addRow(new Object[] { ident, addr });
			pack();
		}
	}

	public void addPersonBelief(WorkingMemoryAddress _address,
			IndependentFormulaDistributionsBelief<dBelief> _belief) {

		FormulaDistribution distribution = _belief.getContent().get(
				PersonTransferFunction.PERSON_ID);

		if (distribution != null) {
			String ident = "Person: "
					+ distribution.getDistribution().getMostLikely()
							.getProposition();

			String addr = toAddressString(_address);
			m_objectBeliefsTableModel.addRow(new Object[] { ident, addr });
			pack();
		}
	}

	public void updatePlace(WorkingMemoryAddress _address, long _id,
			PlaceStatus _status) {
		// println("trying to update place");
		// for (int row = 0; row < m_placeTableModel.getRowCount(); row++) {
		// long placeID = (Long) m_placeTableModel.getValueAt(row,
		// PLACE_ID_COLUMN);
		// if (placeID == _id) {
		// m_placeTableModel.removeRow(row);
		// m_placeTableModel.addRow(new Object[] { _id, _status });
		// println("done");
		// pack();
		// return;
		// }
		// }
	}

	public void removePlace(WorkingMemoryAddress _address) {
		// long id = m_places.get(_address.id);
		// println("trying to remove place");
		//
		// for (int row = 0; row < m_placeTableModel.getRowCount(); row++) {
		// long placeID = (Long) m_placeTableModel.getValueAt(row,
		// PLACE_ID_COLUMN);
		// if (placeID == id) {
		// m_placeTableModel.removeRow(row);
		// m_places.remove(_address.id);
		// pack();
		// println("done");
		//
		// return;
		// }
		// }
	}

	private void askForIdentity() throws CASTException {
		WorkingMemoryAddress beliefAddr = getSelectedRoomBeliefAddress();
		if (beliefAddr != null) {
			m_exeMan.executeSingleBeliefAction(beliefAddr, new MonitorPanel(),
					AskForIdentity.class);
		}
	}

	private void engageWithHuman() throws CASTException {
		WorkingMemoryAddress beliefAddr = getSelectedObjectBeliefAddress();
		if (beliefAddr != null) {
			m_exeMan.executeSingleBeliefAction(beliefAddr, new MonitorPanel(),
					TurnToHuman.class);
			
			// UNTESTED!
			// nah: to try the dialogue stuff, replace the above line with one
			// of these, or alternative add a sleep or some check on the result
			// of the previous action (the addr of which is returned, or use the
			// monitor thingy)
			// to open dialogue
			// m_exeMan.triggerEngagementAction(beliefAddr, true, new
			// MonitorPanel());
			// to close dialogue
			// m_exeMan.triggerEngagementAction(beliefAddr, false, new
			// MonitorPanel());
		}
	}

	private void askPolarIdentity() throws CASTException {
		WorkingMemoryAddress beliefAddr = getSelectedRoomBeliefAddress();
		if (beliefAddr != null) {
			m_exeMan.executeSingleBeliefPlusStringAction(
					beliefAddr,
					getFeatureValue(beliefAddr.id + ":"
							+ beliefAddr.subarchitecture, "identity",
							"asked for"), new MonitorPanel(),
					AskPolarIdentity.class);
		}
	}

	
	private String getFeatureValue(String _beliefID, String _concept, String _op) {
		String message = "What " + _concept + " should be " + _op
				+ " for belief " + _beliefID + "?";

		return (String) JOptionPane.showInputDialog(this, message);
	}

	private JPanel getJPanelStatus() {
		if(jPanelStatus == null) {
			jPanelStatus = new JPanel();
			jPanelStatus.add(getJStatusFailed());
			jPanelStatus.add(getJRadioButton1());
			jPanelStatus.add(getJStatusSuccess());
			jPanelStatus.add(getJStatusText());
		}
		return jPanelStatus;
	}
	
	private JRadioButton getJStatusSuccess() {
		if(jStatusSuccess == null) {
			jStatusSuccess = new JRadioButton();
			jStatusSuccess.setForeground(new java.awt.Color(0,255,0));
			jStatusSuccess.setText("SUCCEEDED");
		}
		return jStatusSuccess;
	}
	
	private JRadioButton getJStatusFailed() {
		if(jStatusFailed == null) {
			jStatusFailed = new JRadioButton();
			jStatusFailed.setForeground(new java.awt.Color(255,0,0));
			jStatusFailed.setText("FAILED");
		}
		return jStatusFailed;
	}
	
	private JRadioButton getJRadioButton1() {
		if(jStatusExecuting == null) {
			jStatusExecuting = new JRadioButton();
			jStatusExecuting.setEnabled(true);
			jStatusExecuting.setForeground(new java.awt.Color(128,128,0));
			jStatusExecuting.setText("ONGOING");
		}
		return jStatusExecuting;
	}
	
	public JTextField getJStatusText() {
		if(jStatusText == null) {
			jStatusText = new JTextField();
			jStatusText.setText("status");
			jStatusText.setPreferredSize(new java.awt.Dimension(600, 22));
			jStatusText.setEditable(false);
		}
		return jStatusText;
	}

}
