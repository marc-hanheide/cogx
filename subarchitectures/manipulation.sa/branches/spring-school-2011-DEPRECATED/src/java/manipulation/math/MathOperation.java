package manipulation.math;

import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;

/**
 * collection of mathematical operations
 * 
 * @author Torben Toeniges
 * 
 */
public class MathOperation {
	/**
	 * calculates the direction between two given 3D points
	 * 
	 * @param source
	 *            source point
	 * @param target
	 *            target point
	 * @return direction between the two points
	 */
	public static Vector3D getDirection(Vector3D source, Vector3D target) {

		Vector3D direction = new Vector3D((target.getX() - source.getX()),
				(target.getY() - source.getY()),
				(target.getZ() - source.getZ()));
		Vector3D normDirection = direction.norm();

		return normDirection;
	}

	/**
	 * calculates rotation around the world x axis by a given angle
	 * 
	 * @param angle
	 *            given angle in radiant
	 * @return rotation matrix corresponding to the angle
	 */
	public static Matrix getRotationAroundX(double angle) {
		return new Matrix(1, 0, 0, 0, Math.cos(angle), Math.sin(angle), 0,
				-Math.sin(angle), Math.cos(angle));

	}

	/**
	 * calculates rotation around the world y axis by a given angle
	 * 
	 * @param angle
	 *            given angle in radiant
	 * @return rotation matrix corresponding to the angle
	 */
	public static Matrix getRotationAroundY(double angle) {
		return new Matrix(Math.cos(angle), 0, -Math.sin(angle), 0, 1, 0, Math
				.sin(angle), 0, Math.cos(angle));
	}

	/**
	 * calculates rotation around the world z axis by a given angle
	 * 
	 * @param angle
	 *            given angle in radiant
	 * @return rotation matrix corresponding to the angle
	 */
	public static Matrix getRotationAroundZ(double angle) {
		return new Matrix(Math.cos(angle), Math.sin(angle), 0,
				-Math.sin(angle), Math.cos(angle), 0, 0, 0, 1);
	}

	/**
	 * converts radiant to degree
	 * 
	 * @param angle
	 *            value in radiant
	 * @return angle value in degree
	 */
	public static double getDegree(double angle) {
		return angle * (180 / Math.PI);
	}

	/**
	 * converts degree to radiant
	 * 
	 * @param angle
	 *            value in degree
	 * @return angle value in radiant
	 */
	public static double getRadiant(double angle) {
		return angle * (Math.PI / 180);
	}

	/**
	 * calculates a multiplication of a 3x3 matrix with a 3D vector
	 * 
	 * @param matrix
	 *            given matrix
	 * @param vector
	 *            given vector
	 * @return solution vector
	 */
	public static Vector3D getMatrixVectorMultiplication(Matrix matrix,
			Vector3D vector) {
		return new Vector3D(matrix.getM00() * vector.getX() + matrix.getM01()
				* vector.getY() + matrix.getM02() * vector.getZ(), matrix
				.getM10()
				* vector.getX()
				+ matrix.getM11()
				* vector.getY()
				+ matrix.getM12() * vector.getZ(), matrix.getM20()
				* vector.getX() + matrix.getM21() * vector.getY()
				+ matrix.getM22() * vector.getZ());
	}

	/**
	 * calculates the multiplication of a 3x3 matrix with a 3x3 matrix
	 * 
	 * @param m1
	 *            given matrix (left)
	 * @param m2
	 *            given matrix (right)
	 * @return solution matrix
	 */
	public static Matrix getMatrixMatrixMultiplication(Matrix m1, Matrix m2) {
		double m00 = m1.getM00() * m2.getM00() + m1.getM01() * m2.getM10()
				+ m1.getM02() * m2.getM20();
		double m01 = m1.getM00() * m2.getM01() + m1.getM01() * m2.getM11()
				+ m1.getM02() * m2.getM21();
		double m02 = m1.getM00() * m2.getM02() + m1.getM01() * m2.getM12()
				+ m1.getM02() * m2.getM22();

		double m10 = m1.getM10() * m2.getM00() + m1.getM11() * m2.getM10()
				+ m1.getM12() * m2.getM20();
		double m11 = m1.getM10() * m2.getM01() + m1.getM11() * m2.getM11()
				+ m1.getM12() * m2.getM21();
		double m12 = m1.getM10() * m2.getM02() + m1.getM11() * m2.getM12()
				+ m1.getM12() * m2.getM22();

		double m20 = m1.getM20() * m2.getM00() + m1.getM21() * m2.getM10()
				+ m1.getM22() * m2.getM20();
		double m21 = m1.getM20() * m2.getM01() + m1.getM21() * m2.getM11()
				+ m1.getM22() * m2.getM21();
		double m22 = m1.getM20() * m2.getM02() + m1.getM21() * m2.getM12()
				+ m1.getM22() * m2.getM22();

		return new Matrix(m00, m10, m20, m01, m11, m21, m02, m12, m22);
	}

	/**
	 * calculates the addition of two 3D vectors
	 * 
	 * @param point1
	 *            given 3D vector
	 * @param point2
	 *            given 3D vector
	 * @return solution 3D vector
	 */
	public static Vector3D getVectorAddition(Vector3D point1, Vector3D point2) {
		return new Vector3D(point1.getX() + point2.getX(), point1.getY()
				+ point2.getY(), point1.getZ() + point2.getZ());
	}

	


	/**
	 * calculates the distance of two 3D vectors and return the dimensions
	 * separated
	 * 
	 * @param point1
	 *            given 3D vector
	 * @param point2
	 *            given 3D vector
	 * @return distance 3D vector (dimensions separated)
	 */
	public static Vector3D getDistanceSeparatedDimensions(Vector3D point1,
			Vector3D point2) {
		Vector3D returnVector = new Vector3D(0, 0, 0);

		returnVector.setX(point1.getX() - point2.getX());
		returnVector.setY(point1.getY() - point2.getY());
		returnVector.setZ(point1.getZ() - point2.getZ());

		return returnVector;
	}


}
