package binder.gui;

import javax.swing.JOptionPane;
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
	
	private BayesianNetworkEdgeWrapper edge;
	
	public BayesianEdgeTable(BayesianNetworkEdgeWrapper edge) {
		columnNames[0] = edge.getNodeTarget().getFeatureLabelName();
		columnNames[1] = edge.getNodeSource().getFeatureLabelName();
		this.edge = edge;
		data = new String[edge.getNumberOfAlternatives()][3];
		int i = 0;
		for(ConditionedAlternative alt : edge.getConditionedAlternatives()) {
			data[i][0] = alt.getAlternative();
			data[i][1] = alt.getAlternativeConditioned();
			data[i][2] = alt.getConditionalProbability().toString();
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
		String alternative = (String) data[row][0];
		String alternative_conditioned = (String) data[row][1];
		float probability = Float.parseFloat((String)data[row][2]);
		
		if(col == 2) {
			float new_probability = Float.parseFloat((String) value);
			try {
				edge.modifyFeatureConditioned(new ConditionedAlternative(
						alternative_conditioned,
						alternative,
						new_probability));
				data[row][col] = value;
				fireTableCellUpdated(row, col);
			} catch(Exception e) {
				JOptionPane.showMessageDialog(null, e.toString());
			}
		}
		else {
			String new_alter = (String) value;
			
			// first we need to check if there is not already a correlation
			// with this alternatives 
			if(col == 1 && !edge.hasFeatureConditioned(alternative, new_alter)) {
				try {
					edge.removeFeatureConditioned(new ConditionedAlternative(
							alternative_conditioned,
							alternative,
							0f));
					edge.addFeatureConditioned(new ConditionedAlternative(
							new_alter,
							alternative,
							0f));
				} catch(Exception e) {
					JOptionPane.showMessageDialog(null, e.toString());
				}
				
				data[row][col] = value;
				fireTableCellUpdated(row, col);
				return;
			}
			if(col == 0 && !edge.hasFeatureConditioned(new_alter, alternative_conditioned)) {
				try {
					edge.removeFeatureConditioned(new ConditionedAlternative(
							alternative_conditioned,
							alternative,
							0f));
					edge.addFeatureConditioned(new ConditionedAlternative(
							alternative_conditioned,
							new_alter,
							0f));
				} catch(Exception e) {
					JOptionPane.showMessageDialog(null, e.toString());
				}
				
				data[row][col] = value;
				fireTableCellUpdated(row, col);
				return;
			}
			return;
		}
	}
	
	 public boolean isCellEditable(int row, int col) {
		 return true;
	 }
}
