/**
 * 
 */
package motivation.util.viewer;

import java.awt.BorderLayout;
import java.awt.EventQueue;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.event.TableModelEvent;
import javax.swing.event.TableModelListener;
import javax.swing.table.DefaultTableModel;
import javax.swing.table.TableModel;
import javax.swing.table.TableRowSorter;
import javax.swing.text.TabExpander;

import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.Proxy;

import Ice.ObjectImpl;
import NavData.FNode;
import SpatialData.Place;

import motivation.slice.ExploreMotive;
import motivation.slice.Motive;
import motivation.slice.PlanProxy;
import motivation.slice.TestMotive;
import motivation.util.CASTTimeUtil;
import motivation.util.WMEntrySet.ChangeHandler;
import motivation.util.viewer.plugins.Plugin;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author marc
 * 
 */
public class ViewerGUI extends JFrame implements ChangeHandler {

	private static final long serialVersionUID = 1L;
	private JPanel jContentPane = null;
	private JScrollPane jScrollPane = null;
	private JTable jTable = null;
	private DefaultTableModel tableModel;
	private int counter;

	// Map<String, Integer> addrRowMap;
	Map<Class<?>, Plugin> objectDispatcherMap;

	Map<WorkingMemoryAddress, Vector<Object>> tableContent;

	protected Vector<String> columnHeadings;

	/**
	 * This is the default constructor
	 */
	public ViewerGUI() {
		super();
		counter = 0;
		// addrRowMap = new HashMap<String, Integer>();
		objectDispatcherMap = new HashMap<Class<?>, Plugin>();
		tableContent = new HashMap<WorkingMemoryAddress, Vector<Object>>();
		columnHeadings = new Vector<String>();
		columnHeadings.add("#");
		columnHeadings.add("NEW");
		columnHeadings.add("address");
		columnHeadings.add("type");
		columnHeadings.add("info1");
		columnHeadings.add("info2");
		columnHeadings.add("info3");
		columnHeadings.add("info4");
		initialize();

	}

	private void mapToTableModel() {
		final Vector<Vector<Object>> v = new Vector<Vector<Object>>(
				tableContent.values());
		EventQueue.invokeLater(new Runnable() {
			@Override
			public void run() {

				// delete all rows
				while (tableModel.getRowCount() > 0)
					tableModel.removeRow(0);
				// insert all new rows
				for (final Vector<Object> r : v) {
					tableModel.addRow(r);
				}
			}
		});

	}

	/**
	 * This method initializes this
	 * 
	 * @return void
	 */
	private void initialize() {
		this.setSize(629, 243);
		this.setContentPane(getJContentPane());
		this.setTitle("WorkingMemory Monitor");
	}

	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJContentPane() {
		if (jContentPane == null) {
			BorderLayout borderLayout = new BorderLayout();
			borderLayout.setVgap(10);
			jContentPane = new JPanel();
			jContentPane.setLayout(borderLayout);
			jContentPane.add(getJScrollPane(), BorderLayout.CENTER);
		}
		return jContentPane;
	}

	/**
	 * This method initializes jScrollPane
	 * 
	 * @return javax.swing.JScrollPane
	 */
	private JScrollPane getJScrollPane() {
		if (jScrollPane == null) {
			jScrollPane = new JScrollPane();
			jScrollPane.setViewportView(getJTable());
		}
		return jScrollPane;
	}

	/**
	 * This method initializes jTable
	 * 
	 * @return javax.swing.JTable
	 */
	private JTable getJTable() {
		if (jTable == null) {
			tableModel = new DefaultTableModel(columnHeadings, 1);
			jTable = new JTable(tableModel);
			final TableRowSorter<TableModel> sorter;
			sorter = new TableRowSorter<TableModel>(tableModel);
			sorter.setSortsOnUpdates(true);
			jTable.setRowSorter(sorter);
			getContentPane().add(new JScrollPane(jTable));
		}
		return jTable;
	}

	/**
	 * dynamically looks for suitable plugins and stores the association in a
	 * map Plugins are expected to be in package "plugins" relative to this
	 * package, implement the Plugin interface and their should be the
	 * respective SimpleName of the class it work with suffixed by "Info" an
	 * example is: MotiveInfo (for Motives...)
	 */
	private Plugin findPlugin(Class<?> origType) {
		Class<?> oType = origType;
		if (objectDispatcherMap.containsKey(oType))
			return objectDispatcherMap.get(oType);
		// if not yet found, look for supertype plugins
		Plugin pluginToCall = null;
		while (pluginToCall == null) {
			String SimpleName = oType.getSimpleName();
			String fullName = this.getClass().getPackage().getName()
					+ ".plugins." + SimpleName + "Info";
			try {
				System.out.println("trying to load class " + fullName);
				ClassLoader.getSystemClassLoader().loadClass(fullName);
				pluginToCall = (Plugin) Class.forName(fullName).newInstance();
				System.out.println("succeeded... memorize plugin " + fullName
						+ "for type " + oType.getSimpleName());
				break;
			} catch (ClassNotFoundException e) {
				System.out.println("no class " + fullName + "exists.");
			} catch (InstantiationException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IllegalAccessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			oType = oType.getSuperclass();
			if (oType == null) // if no superclass exists, we have to give up
				break;
			if (oType == Ice.Object.class) // we don't need to look up
				// further
				break;

		}
		objectDispatcherMap.put(origType, pluginToCall);
		return null;

	}

	private String addrToString(WorkingMemoryAddress wma) {
		return wma.subarchitecture + "::" + wma.id;
	}

	@Override
	public void motiveChanged(Map<WorkingMemoryAddress, Ice.ObjectImpl> map,
			final WorkingMemoryChange wmc, final Ice.ObjectImpl motive) {

		switch (wmc.operation) {
		case ADD:
		case OVERWRITE:
			Plugin pluginToCall = findPlugin(motive.getClass());

			Vector<Object> row = new Vector<Object>();
			row.add(counter++);
			// mark additions
			if (wmc.operation == WorkingMemoryOperation.ADD)
				row.add("*");
			else
				row.add("");
			row.add(addrToString(wmc.address));
			row.add(motive.getClass().getSimpleName());
			if (pluginToCall != null) { // if we have a plugin for this object
				Vector<Object> extraInfo = pluginToCall.toVector(motive);
				row.addAll(extraInfo);
			}
			tableContent.put(wmc.address, row);
			break;
		case DELETE:
			tableContent.remove(wmc.address);
			break;
		}
		mapToTableModel();
	}

} // @jve:decl-index=0:visual-constraint="10,10"
