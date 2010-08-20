package coma.matrix;

import java.util.TreeMap;

public class Matrix<T> {

	private String[] columnLabels;
	private String[] rowLabels;
	
	private TreeMap<String,Integer> labelToCol;
	private TreeMap<String,Integer> labelToRow;
	
	private Object[][] grid;
	
	public Matrix(int _numCols, int _numRows) {
		this.grid = new Object[_numCols][_numRows];
		this.columnLabels = new String[_numCols];
		this.rowLabels = new String[_numRows];
		this.labelToCol = new TreeMap<String, Integer>();
		this.labelToRow = new TreeMap<String, Integer>();
	}

	
	public Matrix(String[] _colLabels, String[] _rowLabels) throws NonUniqueLabelException {
		this(_colLabels.length, _rowLabels.length);
		this.columnLabels = _colLabels;
		this.rowLabels = _rowLabels;

		for (int i = 0; i < _colLabels.length; i++) {
			if (labelToCol.containsKey(_colLabels[i])) throw new NonUniqueLabelException("Non-unique column label: " + _colLabels[i]); 
			labelToCol.put(_colLabels[i], i);
		}
		for (int j = 0; j < _rowLabels.length; j++) {
			if (labelToRow.containsKey(_rowLabels[j])) throw new NonUniqueLabelException("Non-unique row label: " + _rowLabels[j]); 
			labelToRow.put(_rowLabels[j], j);
		}
	}
	
	public Matrix(String[] _colLabels, String[] _rowLabels, T _initVal) throws NonUniqueLabelException {
		this(_colLabels, _rowLabels);
		for (int i = 0; i < this.grid.length; i++) {
			for (int j=0; j < grid[j].length; j++) {
				grid[j][i] = _initVal;
			}
		}
	}

	
	public int getNumCols() {
		return this.columnLabels.length;
	}
	
	public int getNumRows() {
		return this.rowLabels.length;
	}
	
	@SuppressWarnings("unchecked")
	public T getCell(int _col, int _row) {
		return (T) this.grid[_col][_row];
	}
	
	@SuppressWarnings("unchecked")
	public T getCell(String _colLabel, String _rowLabel) {
		return (T) this.grid[labelToCol.get(_colLabel)][labelToRow.get(_rowLabel)];
	}

	
	public boolean insertCell(T _cell, int _column, int _row) {
		if ((_column >= getNumCols()) || (_row >= getNumRows())) {
			throw new IndexOutOfBoundsException("Error trying to add a cell to " +
					"<" + _column + "," + _row + "> to a matrix of " +
					"size [" + getNumCols() +"," + getNumRows() + "]. Cell not added!");
		} 
		else {
			this.grid[_column][_row] = _cell;
		}
		return false;
	}
	
//	public class Cell {
		//int column;
		//int row;
		// Function fx;
		
//		Object value;
//	}
	
}
