/**
 * 
 */
package castutils.viewer;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.EventQueue;
import java.awt.GridBagLayout;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.HashMap;
import java.util.Map;
import java.util.StringTokenizer;
import java.util.Vector;

import javax.swing.JCheckBox;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.table.DefaultTableModel;
import javax.swing.table.TableCellRenderer;
import javax.swing.table.TableModel;
import javax.swing.table.TableRowSorter;

import org.apache.log4j.Logger;

import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEntrySet.ChangeHandler;
import castutils.viewer.plugins.Plugin;

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
	private JPanel jPanel = null;
	private JCheckBox jCheckBox = null;
	volatile protected boolean freezeView;
	private Logger logger;

	/**
	 * This is the default constructor
	 */
	public ViewerGUI(Logger _logger) {
		super();
		freezeView = false;
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
		logger = _logger;
		initialize();
	}

	public ViewerGUI() {
		this(Logger.getLogger(ViewerGUI.class));
	}

	private synchronized void mapToTableModel() {
		final Vector<Vector<Object>> v = new Vector<Vector<Object>>(
				tableContent.values());
		if (!freezeView) {
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
	}

	/**
	 * This method initializes this
	 * 
	 * @return void
	 */
	private void initialize() {
		this.setSize(1200, 400);
		this.setContentPane(getJContentPane());
		this.setTitle("WorkingMemory Monitor");
		this.pack();
	}

	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJContentPane() {
		if (jContentPane == null) {
			BorderLayout borderLayout = new BorderLayout();
			jContentPane = new JPanel();
			jContentPane.setLayout(borderLayout);
			jContentPane.add(getJScrollPane(), BorderLayout.CENTER);
			jContentPane.add(getJPanel(), BorderLayout.SOUTH);
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
			jTable = new JTable(tableModel) {

				/**
				 * 
				 */
				private static final long serialVersionUID = -2359545345696140088L;

				@Override
				public boolean isCellEditable(int row, int column) {
					return false;
				}

				@Override
				public Component prepareRenderer(TableCellRenderer renderer,
						int rowIndex, int vColIndex) {
					Component c = super.prepareRenderer(renderer, rowIndex,
							vColIndex);
					if (c instanceof JComponent) {
						JComponent jc = (JComponent) c;
						Object v = getValueAt(rowIndex,
								vColIndex);
						if (v instanceof String) {
							jc.setToolTipText((String) v);
						}
					}
					return c;
				}
			};

			jTable.addMouseListener(new MouseAdapter() {
				@Override
				public void mouseClicked(MouseEvent e) {
					showDetails(jTable.getSelectedRow());
				}
			});

			final TableRowSorter<TableModel> sorter;
			sorter = new TableRowSorter<TableModel>(tableModel);
			sorter.setSortsOnUpdates(true);
			jTable.setRowSorter(sorter);
			jTable.setAutoResizeMode(JTable.AUTO_RESIZE_LAST_COLUMN);
			getContentPane().add(new JScrollPane(jTable));
		}
		return jTable;
	}

	protected void showDetails(int selectedRow) {
		String address = (String) jTable.getValueAt(selectedRow, 2);
		// TODO: it's a very ugly way of getting the address
		StringTokenizer st = new StringTokenizer(address, "@");
		WorkingMemoryAddress wma = new WorkingMemoryAddress();
		wma.id = st.nextToken();
		wma.subarchitecture = st.nextToken();
		tableContent.get(wma);
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
				logger.debug("trying to load class " + fullName);
				ClassLoader.getSystemClassLoader().loadClass(fullName);
				pluginToCall = (Plugin) Class.forName(fullName).newInstance();
				logger.debug("succeeded... memorize plugin " + fullName
						+ "for type " + oType.getSimpleName());
				break;
			} catch (ClassNotFoundException e) {
				logger.debug("no class " + fullName + "exists.");
			} catch (InstantiationException e) {
				logger.debug("loading " + fullName + ": ", e);
			} catch (IllegalAccessException e) {
				logger.debug("loading " + fullName + ": ", e);
			}
			oType = oType.getSuperclass();
			if (oType == null) // if no superclass exists, we have to give up
				break;
			if (oType == Ice.Object.class) // we don't need to look up
				// further
				break;

		}
		objectDispatcherMap.put(origType, pluginToCall);
		return pluginToCall;

	}

	private String addrToString(WorkingMemoryAddress wma) {
		return wma.id + "@" + wma.subarchitecture;
	}

	@Override
	public void entryChanged(Map<WorkingMemoryAddress, Ice.ObjectImpl> map,
			final WorkingMemoryChange wmc, final Ice.ObjectImpl motive,
			final Ice.ObjectImpl oldMotive) {

		switch (wmc.operation) {
		case ADD:
		case OVERWRITE:
			Plugin pluginToCall = findPlugin(motive.getClass());

			Vector<Object> row = new Vector<Object>();
			row.add(String.format("%06d", counter++));
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
			{ // log it
				String logString = wmc.operation.name() + ":";
				for (Object o : row) {
					logString += o.toString() + "\n";
				}
				logger.debug(logString);
			}
			tableContent.put(wmc.address, row);
			break;
		case DELETE:
			tableContent.remove(wmc.address);
			break;
		}
		mapToTableModel();
	}

	/**
	 * This method initializes jPanel
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJPanel() {
		if (jPanel == null) {
			jPanel = new JPanel();
			jPanel.setLayout(new GridBagLayout());
			jPanel.add(getJCheckBox());
		}
		return jPanel;
	}

	/**
	 * This method initializes jCheckBox
	 * 
	 * @return javax.swing.JCheckBox
	 */
	private JCheckBox getJCheckBox() {
		if (jCheckBox == null) {
			jCheckBox = new JCheckBox();
			jCheckBox.setText("freeze view");
			jCheckBox.addItemListener(new ItemListener() {

				@Override
				public void itemStateChanged(ItemEvent e) {
					freezeView = (e.getStateChange() == ItemEvent.SELECTED);
					if (!freezeView)
						mapToTableModel();
				}
			});
		}
		return jCheckBox;
	}

} // @jve:decl-index=0:visual-constraint="10,10"
