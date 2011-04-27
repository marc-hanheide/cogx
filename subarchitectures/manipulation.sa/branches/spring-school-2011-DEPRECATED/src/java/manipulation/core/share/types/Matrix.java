package manipulation.core.share.types;

/**
 * represents a 3D-matrix
 * 
 * @author ttoenige
 * 
 */
public class Matrix {
	private double m00;
	private double m10;
	private double m20;

	private double m01;
	private double m11;
	private double m21;

	private double m02;
	private double m12;
	private double m22;

	/**
	 * constructor of a matrix
	 * 
	 * @param m00
	 *            entry (0,0)
	 * @param m10
	 *            entry (1,0)
	 * @param m20
	 *            entry (2,0)
	 * @param m01
	 *            entry (0,1)
	 * @param m11
	 *            entry (1,1)
	 * @param m21
	 *            entry (2,1)
	 * @param m02
	 *            entry (0,2)
	 * @param m12
	 *            entry (1,2)
	 * @param m22
	 *            entry (2,2)
	 */
	public Matrix(double m00, double m10, double m20, double m01, double m11,
			double m21, double m02, double m12, double m22) {
		this.m00 = m00;
		this.m10 = m10;
		this.m20 = m20;
		this.m01 = m01;
		this.m11 = m11;
		this.m21 = m21;
		this.m02 = m02;
		this.m12 = m12;
		this.m22 = m22;
	}

	/**
	 * constructor of a matrix, initialize the matrix as I
	 */
	public Matrix() {
		this.m00 = 1;
		this.m10 = 0;
		this.m20 = 0;
		this.m01 = 0;
		this.m11 = 1;
		this.m21 = 0;
		this.m02 = 0;
		this.m12 = 0;
		this.m22 = 1;
	}

	/**
	 * gets value (0,0) of the matrix
	 * 
	 * @return value (0,0) of the matrix
	 */
	public double getM00() {
		return m00;
	}

	/**
	 * sets the entry (0,0) of the matrix
	 * 
	 * @param m00
	 *            value for the position (0,0)
	 */
	public void setM00(double m00) {
		this.m00 = m00;
	}

	/**
	 * gets value (1,0) of the matrix
	 * 
	 * @return value (1,0) of the matrix
	 */
	public double getM10() {
		return m10;
	}

	/**
	 * sets the entry (1,0) of the matrix
	 * 
	 * @param m00
	 *            value for the position (1,0)
	 */
	public void setM10(double m10) {
		this.m10 = m10;
	}

	/**
	 * gets value (2,0) of the matrix
	 * 
	 * @return value (2,0) of the matrix
	 */
	public double getM20() {
		return m20;
	}

	/**
	 * sets the entry (2,0) of the matrix
	 * 
	 * @param m00
	 *            value for the position (2,0)
	 */
	public void setM20(double m20) {
		this.m20 = m20;
	}

	/**
	 * gets value (0,1) of the matrix
	 * 
	 * @return value (0,1) of the matrix
	 */
	public double getM01() {
		return m01;
	}

	/**
	 * sets the entry (0,1) of the matrix
	 * 
	 * @param m00
	 *            value for the position (0,1)
	 */
	public void setM01(double m01) {
		this.m01 = m01;
	}

	/**
	 * gets value (1,1) of the matrix
	 * 
	 * @return value (1,1) of the matrix
	 */
	public double getM11() {
		return m11;
	}

	/**
	 * sets the entry (1,1) of the matrix
	 * 
	 * @param m00
	 *            value for the position (1,1)
	 */
	public void setM11(double m11) {
		this.m11 = m11;
	}

	/**
	 * gets value (2,1) of the matrix
	 * 
	 * @return value (2,1) of the matrix
	 */
	public double getM21() {
		return m21;
	}

	/**
	 * sets the entry (2,1) of the matrix
	 * 
	 * @param m00
	 *            value for the position (2,1)
	 */
	public void setM21(double m21) {
		this.m21 = m21;
	}

	/**
	 * gets value (0,2) of the matrix
	 * 
	 * @return value (0,2) of the matrix
	 */
	public double getM02() {
		return m02;
	}

	/**
	 * sets the entry (0,2) of the matrix
	 * 
	 * @param m00
	 *            value for the position (0,2)
	 */
	public void setM02(double m02) {
		this.m02 = m02;
	}

	/**
	 * gets value (1,2) of the matrix
	 * 
	 * @return value (1,2) of the matrix
	 */
	public double getM12() {
		return m12;
	}

	/**
	 * sets the entry (1,2) of the matrix
	 * 
	 * @param m00
	 *            value for the position (1,2)
	 */
	public void setM12(double m12) {
		this.m12 = m12;
	}

	/**
	 * gets value (2,2) of the matrix
	 * 
	 * @return value (2,2) of the matrix
	 */
	public double getM22() {
		return m22;
	}

	/**
	 * sets the entry (2,2) of the matrix
	 * 
	 * @param m00
	 *            value for the position (2,2)
	 */
	public void setM22(double m22) {
		this.m22 = m22;
	}

	/**
	 * gets an (value1, value2) entry of the matrix
	 * 
	 * @param value1
	 * @param value2
	 * @return matrix value
	 */
	public double getValue(int value1, int value2) {
		Double returnValue = null;

		if (value1 == 0 && value2 == 0)
			returnValue = this.m00;
		if (value1 == 0 && value2 == 1)
			returnValue = this.m01;
		if (value1 == 0 && value2 == 2)
			returnValue = this.m02;
		if (value1 == 1 && value2 == 0)
			returnValue = this.m10;
		if (value1 == 1 && value2 == 1)
			returnValue = this.m11;
		if (value1 == 1 && value2 == 2)
			returnValue = this.m12;
		if (value1 == 2 && value2 == 0)
			returnValue = this.m20;
		if (value1 == 2 && value2 == 1)
			returnValue = this.m21;
		if (value1 == 2 && value2 == 2)
			returnValue = this.m22;

		return returnValue;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public String toString() {
		return "{Matrix: [m00: " + m00 + " m10 " + m10 + " m20 " + m20
				+ " m01 " + m01 + " m11 " + m11 + " m21 " + m21 + " m02 " + m02
				+ " m12 " + m12 + " m22 " + m22 + "]}";
	}

}
