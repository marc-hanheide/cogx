package binder.gui;

import javax.swing.table.AbstractTableModel;

import binder.autogen.bayesiannetworks.BayesianNetworkEdge;
import binder.autogen.bayesiannetworks.BayesianNetworkNode;
import binder.autogen.bayesiannetworks.FeatureValueCorrelation;
import binder.utils.FeatureValueUtils;

public class BayesianNodeTable extends AbstractTableModel {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private String[] columnNames = {"Feature", "Probability"};
	
	private Object[][] data;
	
	public BayesianNodeTable(BayesianNetworkNodeWrapper selected_node) {
		columnNames[0] = selected_node.getFeatureLabelName();
		data = new String[selected_node.getNumberOfAlternatives()][2];
		int i = 0;
		for(String alt : selected_node.getAlternativeNames()) {
			data[i][0] = alt;
			data[i][1] = selected_node.getProbabilityOfAlternative(alt).toString();
			i += 1;
		}
	}
	
	public int getColumnCount() {
		return columnNames.length;
	}
	
	public int getRowCount() {
		return data.length;
	}
	
	public String getColumnName(int col) {
		return columnNames[col];
	}
	
	public Object getValueAt(int row, int col) {
		return data[row][col];
	}
	
	public void setValueAt(Object value, int row, int col) {
		if(col == 1) {
			float sum = 0.0f;
			float new_value = 0f;
			float old_value =Float.parseFloat((String)data[row][1]);
			
			try {
				new_value = Float.parseFloat(value.toString());
			} catch(java.lang.NumberFormatException e) {
				new_value = old_value;
			}
			
			for(int i = 0; i < data.length; ++i) {
				sum += Float.parseFloat(data[i][1].toString());
			}
			
			float left = java.lang.Math.max(0, 1f - (sum - old_value));
			
			new_value = java.lang.Math.min(new_value, left);
			
			data[row][col] = new Float(new_value).toString();
			fireTableCellUpdated(row, col);
		}
	}
	
	 public boolean isCellEditable(int row, int col) {
		 return (col > 0);
	 }
}
