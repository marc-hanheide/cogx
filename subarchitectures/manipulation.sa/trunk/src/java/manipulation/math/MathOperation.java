package manipulation.math;

import java.util.Vector;

import manipulation.core.share.exceptions.MathException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Polarcoordinate;
import manipulation.core.share.types.Quaternion;
import manipulation.core.share.types.Vector2D;
import manipulation.core.share.types.Vector3D;

/**
 * collection of mathematical operations
 * 
 * @author ttoenige
 * 
 */
public class MathOperation {
	/**
	 * calculates an angle to watch from a given point on another given point
	 * 
	 * @param watchAtPoint
	 *            point to watch at
	 * @param watchFromPoint
	 *            point to watch from
	 * @return angle in radiant
	 * @throws MathException
	 */
	public static double getBaseAngleToWatch(Vector2D watchAtPoint,
			Vector2D watchFromPoint) throws MathException {

		double xDiff = watchFromPoint.getX() - watchAtPoint.getX();
		double yDiff = watchFromPoint.getY() - watchAtPoint.getY();
		double angle = Math.atan(Math.abs(xDiff) / Math.abs(yDiff));

		if (xDiff >= 0 && yDiff > 0) {
			// Top right

			return -(Math.PI / 2 + angle);
		} else if (xDiff > 0 && yDiff <= 0) {
			// Down right

			return Math.PI / 2 + angle;
		} else if (xDiff < 0 && yDiff >= 0) {
			// Top left

			return -(Math.PI / 2 - angle);
		} else if (xDiff <= 0 && yDiff < 0) {
			// Down left

			return Math.PI / 2 - angle;
		} else {
			throw new MathException("No angle to calculate, same points");
		}

	}

	/**
	 * converts a Polar coordinate into a Cartesian coordinate
	 * 
	 * @param pol
	 *            Polar coordinate to convert
	 * @return converted Cartesian coordinate
	 */
	public static Vector2D getCartesianCoordinate(Polarcoordinate pol) {
		return new Vector2D(pol.getR() * Math.cos(pol.getPhi()), pol.getR()
				* Math.sin(pol.getPhi()));
	}

	/**
	 * converts a Cartesian Coordinate into a Polar coordinate
	 * 
	 * @param cartPoint
	 *            Cartesian coordinate to convert
	 * @return converted Polar coordinate
	 */
	public static Polarcoordinate getPolarCoordinate(Vector2D cartPoint) {
		double r = Math.sqrt((Math.pow(cartPoint.getX(), 2) + Math.pow(
				cartPoint.getY(), 2)));
		double phi = Math.atan2(cartPoint.getY(), cartPoint.getX());

		if (phi < 0) {
			phi = phi + 2 * Math.PI;
		}

		return new Polarcoordinate(r, phi);
	}

	/**
	 * calculates the direction between two given 2D points
	 * 
	 * @param source
	 *            source point
	 * @param target
	 *            target point
	 * @return direction between the two points
	 */
	public static Vector2D getDirection(Vector2D source, Vector2D target) {

		Vector2D direction = new Vector2D((target.getX() - source.getX()),
				(target.getY() - source.getY()));
		Vector2D normDirection = direction.norm();

		return normDirection;
	}

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
	 * calculates the scalar product of two points
	 * 
	 * @param p1
	 *            given point
	 * @param p2
	 *            given point
	 * @return scalar product value
	 */
	public static double getScalarProduct(Vector2D p1, Vector2D p2) {
		return p1.getX() * p2.getX() + p1.getY() * p2.getY();
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

	public static Vector2D getVectorAddition(Vector2D point1, Vector2D point2) {
		return new Vector2D(point1.getX() + point2.getX(), point1.getY()
				+ point2.getY());
	}

	/**
	 * calculates the subtraction of two 3D vectors
	 * 
	 * @param point1
	 *            given 3D vector
	 * @param point2
	 *            given 3D vector
	 * @return solution 3D vector
	 */
	public static Vector3D getVectorSubstraction(Vector3D point1,
			Vector3D point2) {
		return new Vector3D(point1.getX() - point2.getX(), point1.getY()
				- point2.getY(), point1.getZ() - point2.getZ());
	}

	public static Vector2D getVectorSubstraction(Vector2D point1,
			Vector2D point2) {
		return new Vector2D(point1.getX() - point2.getX(), point1.getY()
				- point2.getY());
	}

	/**
	 * calculates the mean of a 3D-vector-list
	 * 
	 * @param points
	 *            given 3D vectors
	 * @return single 3D mean vector
	 */
	public static Vector3D getMean(Vector<Vector3D> points) {
		Vector3D mean = new Vector3D(0, 0, 0);
		for (Vector3D point3d : points) {
			mean = MathOperation.getVectorAddition(mean, point3d);
		}

		mean.setX(mean.getX() / points.size());
		mean.setY(mean.getY() / points.size());
		mean.setZ(mean.getZ() / points.size());
		return mean;
	}

	/**
	 * calculates the mean of a 2D-vector-list
	 * 
	 * @param points
	 *            given 2D vectors
	 * @return single 2D mean vector
	 */
	public static Vector2D getMean(Vector<Vector2D> points) {
		Vector2D mean = new Vector2D(0, 0);
		for (Vector2D point2d : points) {
			mean = MathOperation.getVectorAddition(mean, point2d);
		}

		mean.setX(mean.getX() / points.size());
		mean.setY(mean.getY() / points.size());
		return mean;
	}

	/**
	 * calculates the distance of two 3D vectors
	 * 
	 * @param point1
	 *            given 3D vector
	 * @param point2
	 *            given 3D vector
	 * @return distance value
	 */
	public static double getDistance(Vector3D point1, Vector3D point2) {
		return Math.sqrt(Math.pow((point1.getX() - point2.getX()), 2)
				+ Math.pow((point1.getY() - point2.getY()), 2)
				+ Math.pow((point1.getZ() - point2.getZ()), 2));
	}

	/**
	 * calculates the distance of two 2D vectors
	 * 
	 * @param point1
	 *            given 2D vector
	 * @param point2
	 *            given 2D vector
	 * @return distance value
	 */
	public static double getDistance(Vector2D point1, Vector2D point2) {
		return Math.sqrt(Math.pow((point1.getX() - point2.getX()), 2)
				+ Math.pow((point1.getY() - point2.getY()), 2));
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

	/**
	 * calculates the sum over a double array
	 * 
	 * @param array
	 *            given array
	 * @return sum value
	 */
	public static double getSumOverArray(double[] array) {
		double sum = 0;
		for (double value : array) {
			sum += value;
		}

		return sum;
	}

	/**
	 * calculates the euclidean distance of a 3D vector
	 * 
	 * @param input
	 *            given 3D vector
	 * @return distance value
	 */
	public static double getEuclDistance(Vector3D input) {
		return Math.sqrt(Math.pow(input.getX(), 2) + Math.pow(input.getY(), 2)
				+ Math.pow(input.getZ(), 2));
	}

	public static double getTrace(Matrix input) {
		return input.getM00() + input.getM11() + input.getM22();
	}

	public static Quaternion getQuaternion(Matrix matrix) {
		double trace = MathOperation.getTrace(matrix);

		double qw, qx, qy, qz;

		if (trace > 0) {
			double S = Math.sqrt(trace + 1.0) * 2; // S=4*qw
			qw = 0.25 * S;
			qx = (matrix.getM21() - matrix.getM12()) / S;
			qy = (matrix.getM02() - matrix.getM20()) / S;
			qz = (matrix.getM10() - matrix.getM01()) / S;
		} else if ((matrix.getM00() > matrix.getM11())
				& (matrix.getM00() > matrix.getM22())) {
			double S = Math.sqrt(1.0 + matrix.getM00() - matrix.getM11()
					- matrix.getM22()) * 2; // S=4*qx
			qw = (matrix.getM21() - matrix.getM12()) / S;
			qx = 0.25 * S;
			qy = (matrix.getM01() + matrix.getM10()) / S;
			qz = (matrix.getM02() + matrix.getM20()) / S;
		} else if (matrix.getM11() > matrix.getM22()) {
			double S = Math.sqrt(1.0 + matrix.getM11() - matrix.getM00()
					- matrix.getM22()) * 2; // S=4*qy
			qw = (matrix.getM02() - matrix.getM20()) / S;
			qx = (matrix.getM01() + matrix.getM10()) / S;
			qy = 0.25 * S;
			qz = (matrix.getM12() + matrix.getM21()) / S;
		} else {
			double S = Math.sqrt(1.0 + matrix.getM22() - matrix.getM00()
					- matrix.getM11()) * 2; // S=4*qz
			qw = (matrix.getM10() - matrix.getM01()) / S;
			qx = (matrix.getM02() + matrix.getM20()) / S;
			qy = (matrix.getM12() + matrix.getM21()) / S;
			qz = 0.25 * S;
		}

		return new Quaternion(qw, qx, qy, qz);

		// System.out.println(trace);
		//
		// double q0 = 0, q1 = 0, q2 = 0, q3 = 0;
		//
		// if (trace >= 0) {
		// s = Math.sqrt(trace + 1);
		// q0 = s * (1 / 2);
		// s = (1 / 2) / s;
		//
		// q1 = (matrix.getM21() - matrix.getM12()) * s;
		// q2 = (matrix.getM02() - matrix.getM20()) * s;
		// q3 = (matrix.getM10() - matrix.getM01()) * s;
		// } else {
		// int idx = 0;
		// if (matrix.getM11() > matrix.getM00())
		// idx = 1;
		// if (matrix.getM22() > matrix.getValue(idx, idx))
		// idx = 2;
		//
		// switch (idx) {
		// case 0:
		// s = Math.sqrt(matrix.getValue(0, 0) - matrix.getValue(1, 1)
		// - matrix.getValue(2, 2) + 1);
		// q1 = s * (1 / 2);
		// s = (1 / 2) / s;
		// q2 = (matrix.getValue(0, 1) + matrix.getValue(1, 0)) * s;
		// q3 = (matrix.getValue(2, 0) + matrix.getValue(0, 2)) * s;
		// q0 = (matrix.getValue(2, 1) - matrix.getValue(1, 2)) * s;
		// break;
		// case 1:
		// s = Math.sqrt(matrix.getValue(1, 1) - matrix.getValue(2, 2)
		// - matrix.getValue(0, 0) + 1);
		// q2 = s * (1 / 2);
		// s = (1 / 2) / s;
		// q3 = (matrix.getValue(1, 2) + matrix.getValue(2, 1)) * s;
		// q1 = (matrix.getValue(0, 1) + matrix.getValue(1, 0)) * s;
		// q0 = (matrix.getValue(0, 2) - matrix.getValue(2, 0)) * s;
		// break;
		// case 2:
		// s = Math.sqrt(matrix.getValue(2, 2) - matrix.getValue(0, 0)
		// - matrix.getValue(1, 1) + 1);
		// q3 = s * (1 / 2);
		// s = (1 / 2) / s;
		// q1 = (matrix.getValue(2, 0) + matrix.getValue(0, 2)) * s;
		// q2 = (matrix.getValue(1, 2) + matrix.getValue(2, 1)) * s;
		// q0 = (matrix.getValue(1, 0) - matrix.getValue(0, 1)) * s;
		// break;
		// default:
		// break;
		// }
		//
		// }
		// return new Quaternion(q0, q1, q2, q3);
	}

	public static double getAngularDist(Quaternion q0, Quaternion q1) {
		// acos(quat0 * quat1): range: <0, Pi>, -Pi - identity, Pi - the largest
		// distance
		Quaternion q1neg = q1.negate();

		double returnValue = (1 / Math.PI)
				* Math.min(Math.acos(q0.dot(q1)), Math.acos(q0.dot(q1neg)));

		return returnValue;
	}

	public static void main(String[] args) {
		Matrix m1 = new Matrix(0.5, 0, 0.7, 0, 0.8, 0, 0, 0, 0.3);
		Matrix m2 = new Matrix(0.5, 0, 0.7, 0, 0.8, 0, 0, 0, 0.3);

		Quaternion q1 = getQuaternion(m1);
		System.out.println(q1);
		Quaternion q2 = getQuaternion(m2);
		System.out.println(q2);

		System.out.println(MathOperation.getAngularDist(q1, q2));
	}
}
