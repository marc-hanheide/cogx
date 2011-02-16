package coma.matrix;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.TreeMap;

public class Matrix<T> implements Serializable {

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
		System.out.println("Successfully executed basic constructor new Matrix("+_numCols+","+_numRows+")");
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
		System.out.println("Successfully executed extended constructor new Matrix("+_colLabels+","+_rowLabels+")");
	}
	
	public Matrix(String[] _colLabels, String[] _rowLabels, T _initVal) throws NonUniqueLabelException {
		this(_colLabels, _rowLabels);
		for (int i = 0; i < this.grid.length; i++) {
			for (int j=0; j < grid[i].length; j++) {
				grid[i][j] = _initVal;
			}
		}
		System.out.println("Successfully executed extended constructor new Matrix("+_colLabels+","+_rowLabels+","+_initVal+")");
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
		return true;
	}
	
	public boolean insertCell(T _cell, String _colLabel, String _rowLabel) {
	//	if (!labelToCol.containsKey(_colLabel)) {
	//			throw new IndexOutOfBoundsException("Error trying to add a cell to " +
	//					"<" + _colLabel + "," + _rowLabel + ">. Column label " + _colLabel + " is unknown! " +
	//							"Cell not added!");
	//	}
	//	else if (!labelToRow.containsKey(_rowLabel)) {
	//		throw new IndexOutOfBoundsException("Error trying to add a cell to " +
	//				"<" + _colLabel + "," + _rowLabel + ">. Row label " + _rowLabel + " is unknown! " +
	//						"Cell not added!");
	//	} 
	//	else {
			this.grid[labelToCol.get(_colLabel)][labelToRow.get(_rowLabel)] =  _cell;
	//	}
		return true;
	}
	
	public void saveToFile(String _filename) throws IOException {
		FileOutputStream fout = new FileOutputStream(_filename);
		ObjectOutputStream oos = new ObjectOutputStream(fout);
		oos.writeObject(this);
		oos.close();
	}
	
	public static Matrix loadFromFile(String _filename) throws IOException, ClassNotFoundException {
		FileInputStream fin = new FileInputStream(_filename);
		ObjectInputStream ois = new ObjectInputStream(fin);
		return (Matrix) ois.readObject();
	}
	
	public String getRowLabel(int _row) {
		return rowLabels[_row];
	}

	public String getColLabel(int _col) {
		return columnLabels[_col];
	}

	
	
}
