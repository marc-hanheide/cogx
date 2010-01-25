package binder.gui;

import javax.swing.table.AbstractTableModel;

import binder.autogen.bayesiannetworks.BayesianNetworkEdge;
import binder.autogen.bayesiannetworks.FeatureValueCorrelation;
import binder.utils.FeatureValueUtils;

public class BayesianEdgeTable extends AbstractTableModel {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private String[] columnNames = {"Feature", "Feature", "Probability"};
	
	private Object[][] data;
	
	public BayesianEdgeTable(BayesianNetworkEdge edge) {
		data = new String[edge.correlations.length][3];
		for(int i = 0; i < edge.correlations.length; i++) {
			data[i][0] = FeatureValueUtils.toString(edge.correlations[i].value2);
			data[i][1] = FeatureValueUtils.toString(edge.correlations[i].value1);
			data[i][2] = new Float(edge.correlations[i].condProb).toString();
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
	
}
