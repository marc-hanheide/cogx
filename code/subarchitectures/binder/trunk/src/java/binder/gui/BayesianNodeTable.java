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
	
	public BayesianNodeTable(BayesianNetworkNode node) {
		data = new String[node.feat.alternativeValues.length][2];
		for(int i = 0; i < node.feat.alternativeValues.length; i++) {
			data[i][0] = FeatureValueUtils.toString(node.feat.alternativeValues[i]);
			data[i][1] = new Float(node.feat.alternativeValues[i].independentProb).toString();
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
