/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package castutils.viewer;

import java.util.Map;
import java.util.Vector;

import javax.swing.table.AbstractTableModel;

import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;
import castutils.castextensions.WMView.ChangeHandler;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public abstract class WMJTableModel<T extends Ice.ObjectImpl> extends
		AbstractTableModel implements ChangeHandler<T> {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	protected final WMView<T> view;

	private Vector<T> rows = new Vector<T>();

	private final String[] columns;

	/**
	 * 
	 */
	public WMJTableModel(WMView<T> view, String[] columns) {
		super();
		this.view = view;
		this.columns = columns;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see javax.swing.table.TableModel#getColumnCount()
	 */
	@Override
	public int getColumnCount() {
		return columns.length;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see javax.swing.table.TableModel#getRowCount()
	 */
	@Override
	public synchronized int getRowCount() {
		return rows.size();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see javax.swing.table.TableModel#getValueAt(int, int)
	 */
	@Override
	public Object getValueAt(int rowIndex, int columnIndex) {
		T object = rows.get(rowIndex);
		return getValueAt(object, columnIndex);
	}

	protected abstract Object getValueAt(T object, int columnIndex);

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * castutils.castextensions.WMView.ChangeHandler#entryChanged(java.util.Map,
	 * cast.cdl.WorkingMemoryChange, Ice.ObjectImpl, Ice.ObjectImpl)
	 */
	@Override
	public synchronized void entryChanged(Map<WorkingMemoryAddress, T> map,
			WorkingMemoryChange wmc, T newEntry, T oldEntry)
			throws CASTException {
		rows=new Vector<T>(map.values());
		this.fireTableDataChanged();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see javax.swing.table.AbstractTableModel#getColumnName(int)
	 */
	@Override
	public String getColumnName(int column) {
		return columns[column];
	}

}
