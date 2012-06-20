package castutils.castextensions.wmeditor;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.StringTokenizer;

import javax.swing.DefaultComboBoxModel;

import org.yaml.snakeyaml.error.YAMLException;

import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import castutils.castextensions.wmeditor.WMEditorFrame.EditorActionListener;
import castutils.castextensions.wmeditor.serializer.Serializer;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefs.slice.GroundedBelief;

public abstract class WMEditorComponent extends ManagedComponent implements
		EditorActionListener, WorkingMemoryChangeReceiver {

	
	private WMEditorFrame gui;
	Map<String, String> templates = new HashMap<String, String>();

	Map<WorkingMemoryAddress, WorkingMemoryChange> wmElemMap = Collections
			.synchronizedMap(new HashMap<WorkingMemoryAddress, WorkingMemoryChange>());

	Set<String> saSet = Collections.synchronizedSet(new HashSet<String>());
	private Set<String> subscriptions = new HashSet<String>();
	private Map<String, WorkingMemoryAddress> list2WmaMap = Collections
			.synchronizedMap(new HashMap<String, WorkingMemoryAddress>());
	private String templateListFilter = "";
	private String wmEntriesListFilter = "";

	protected abstract Serializer getSerializer();
	protected abstract String getTemplateFile();

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */

	@Override
	protected void configure(Map<String, String> config) {
		String subscrStr = config.get("--subscribe");
		if (subscrStr != null) {
			StringTokenizer st = new StringTokenizer(subscrStr, ",");
			while (st.hasMoreTokens()) {
				String className = st.nextToken();
				className = className.trim();
				subscriptions.add(className);
			}
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected void start() {
		for (String className : subscriptions) {
			try {
				println("add type '" + className + "'");
				ClassLoader.getSystemClassLoader().loadClass(className);
				addChangeFilter(ChangeFilterFactory
						.createGlobalTypeFilter(
								(Class<? extends Ice.ObjectImpl>) Class
										.forName(className),
								WorkingMemoryOperation.ADD), this);
				addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
						(Class<? extends Ice.ObjectImpl>) Class
								.forName(className),
						WorkingMemoryOperation.DELETE), this);
			} catch (ClassNotFoundException e) {
				println("trying to register for a class that doesn't exist: "
						+ className);
			}
		}

		gui = new WMEditorFrame(this);
		gui.setVisible(true);
		gui.getjEditorPane().setText(
				getSerializer().dump(CASTIndependentFormulaDistributionsBelief.create(
						GroundedBelief.class).get()));
		gui.getjSAComboBox().setModel(
				new DefaultComboBoxModel(
						new String[] { getSubarchitectureID() }));
		try {
			FileReader fr = new FileReader(getTemplateFile());
			templates = (Map<String, String>) getSerializer().load(fr);
			fr.close();
			updateTemplateList();
		} catch (IOException e) {
			getLogger().warn("no history file exists yet", e);
		}
	}

	@Override
	public void getFromWM() throws CASTException, YAMLException,
			ClassCastException {
		log("getFromWM called");
		String id = gui.getjIDTextField().getText();
		String sa = (String) gui.getjSAComboBox().getModel().getSelectedItem();
		Ice.Object obj;
		obj = getMemoryEntry(id, sa, Ice.Object.class);
		
		String str=getSerializer().dump(obj);
		gui.getjEditorPane().setText(str);
	}

	@Override
	public void selectedTemplate() {
		log("selectedTemplate()");
		String key = (String) gui.getjTemplateList().getSelectedValue();
		String newText = templates.get(key);
		println("key=" + key + " text=" + newText);
		if (newText != null)
			gui.getjEditorPane().setText(newText);
		// when we selected something switch to OVERWRITE
		((DefaultComboBoxModel) gui.getjOPComboBox().getModel())
				.setSelectedItem(WorkingMemoryOperation.ADD);
		// get a new ID as well
		getNewID();

	}

	@Override
	public void writeToWM() throws CASTException, YAMLException,
			ClassCastException {
		log("writeToWM()");
		String id = gui.getjIDTextField().getText();
		String sa = (String) gui.getjSAComboBox().getModel().getSelectedItem();
		Ice.Object obj = null;
		switch ((WorkingMemoryOperation) gui.getjOPComboBox().getSelectedItem()) {
		case ADD:
			obj = (Ice.Object) getSerializer().load(gui.getjEditorPane().getText());
			addToWorkingMemory(new WorkingMemoryAddress(id, sa), obj);
			break;
		case OVERWRITE:
			obj = (Ice.Object) getSerializer().load(gui.getjEditorPane().getText());
			try {
				lockEntry(id, sa, WorkingMemoryPermissions.LOCKEDOD);
				getMemoryEntry(id, sa, Ice.Object.class);
				overwriteWorkingMemory(new WorkingMemoryAddress(id, sa), obj);
			} finally {
				unlockEntry(id, sa);
			}
			break;
		case DELETE:
			try {
				lockEntry(id, sa, WorkingMemoryPermissions.LOCKEDOD);
				deleteFromWorkingMemory(id, sa);
				gui.getjEditorPane().setText("");
			} finally {
			}
		}
	}

	@Override
	public String getNewID() {
		return newDataID();
	}

	@Override
	public void addTemplate() {
		Object obj = getSerializer().load(gui.getjEditorPane().getText());
		String key = obj.getClass().getSimpleName() + "(#"
				+ System.currentTimeMillis() + ")";

		if (obj instanceof dBelief)
			key = obj.getClass().getSimpleName() + " ["+((dBelief) obj).type + "] (#"
					+ System.currentTimeMillis() + ")";

		templates.put(key, getSerializer().dump(obj));

		updateTemplateList();
		writeTemplateFile();

	}

	/**
	 * 
	 */
	private void updateTemplateList() {
		List<String> listData = new ArrayList<String>();
		for (String e : templates.keySet()) {
			if (templateListFilter.length() == 0
					|| e.contains(templateListFilter)) {

				listData.add(e);
			}
		}
		Collections.sort(listData);
		gui.getjTemplateList().setListData(listData.toArray());

	}

	/**
	 * 
	 */
	private void writeTemplateFile() {
		try {
			FileWriter fw = new FileWriter(getTemplateFile());
			fw.write(getSerializer().dump(templates));

			fw.close();
		} catch (IOException e1) {
			logException(e1);
		}
	}

	/**
	 * 
	 */
	private void updateEntryList() {
		List<String> listData = new ArrayList<String>();
		for (WorkingMemoryChange e : wmElemMap.values()) {
			
			String[] parts=e.type.split("::");
			String text = parts[parts.length-1] + ": " + e.address.id + "@"
					+ e.address.subarchitecture;
			if (wmEntriesListFilter.length() == 0
					|| text.contains(wmEntriesListFilter)) {
				listData.add(text);
			}
			list2WmaMap.put(text, e.address);
		}
		Collections.sort(listData);
		gui.getjWMList().setListData(listData.toArray());

		Object selectedItem = gui.getjSAComboBox().getModel().getSelectedItem();
		DefaultComboBoxModel cdm = new DefaultComboBoxModel(saSet.toArray());
		if (selectedItem != null)
			cdm.setSelectedItem(selectedItem);
		gui.getjSAComboBox().setModel(cdm);
	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange wmc)
			throws CASTException {
		switch (wmc.operation) {
		case ADD:
			saSet.add(wmc.address.subarchitecture);
			wmElemMap.put(wmc.address, wmc);
			break;
		case DELETE:
			wmElemMap.remove(wmc.address);
			break;
		}

	}

	@Override
	public void selectedEntry() throws CASTException, YAMLException,
			ClassCastException {
		log("selectedEntry()");
		String key = (String) gui.getjWMList().getSelectedValue();
		WorkingMemoryChange wmc = wmElemMap.get(list2WmaMap.get(key));
		gui.getjIDTextField().setText(wmc.address.id);

		// when we selected something switch to OVERWRITE
		((DefaultComboBoxModel) gui.getjOPComboBox().getModel())
				.setSelectedItem(WorkingMemoryOperation.OVERWRITE);

		DefaultComboBoxModel cdm = new DefaultComboBoxModel(saSet.toArray());
		cdm.setSelectedItem(wmc.address.subarchitecture);
		gui.getjSAComboBox().setModel(cdm);
		getFromWM();

		//
		// if (newText != null)
		// gui.getjEditorPane().setText(newText);

	}

	@Override
	public void updateWMList() {
		updateEntryList();
	}

	@Override
	public void deleteTemplate() {
		if (gui.getjTemplateList().getSelectedValue() != null) {
			templates.remove(gui.getjTemplateList().getSelectedValue());
			updateTemplateList();
			writeTemplateFile();

		}
	}

	@Override
	public void renameTemplate(String newName) {
		if (gui.getjTemplateList().getSelectedValue() != null) {
			String textValue = templates.remove(gui.getjTemplateList()
					.getSelectedValue());
			templates.put(newName, textValue);
			updateTemplateList();
			writeTemplateFile();

		}

	}

	@Override
	public void updateTemplateFilter() {
		templateListFilter = gui.getjTemplateFilterTextField().getText();
		updateTemplateList();

	}

	@Override
	public void updateWMEntriesFilter() {
		wmEntriesListFilter = gui.getjWMFilterTextField().getText();
		updateEntryList();

	}
}
