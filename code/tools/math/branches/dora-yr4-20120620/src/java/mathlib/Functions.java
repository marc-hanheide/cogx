/**
 * 
 */
package mathlib;

import cogx.Math.Matrix33;
import cogx.Math.Pose3;
import cogx.Math.Vector3;

/**
 * @author cogx
 * 
 */
public final class Functions {

	/**
	 * Assigns inverse to B, assuming R is orthonormal, i.e. inv(R) = R^T, i.e.
	 * R is a rotation matrix. poses T1 = [R1, p1], T2 = [R2, p2] T2 = [R1^T, -
	 * R1^T * p1]
	 * 
	 * @param T1
	 * @return T2
	 */
	static public Pose3 inverse(Pose3 T1) {
		Pose3 T2 = new Pose3();
		T2.rot = transpose(T1.rot);
		T2.pos = new Vector3();
		T2.pos.x = -T1.pos.x;
		T2.pos.y = -T1.pos.y;
		T2.pos.z = -T1.pos.z;
		T2.pos = mult(T2.rot, T2.pos);
		return T2;
	}

	/** test routine
	 * @param argv
	 */
	public static void main(String[] argv) {
		Pose3 pInWorld = pose3FromEuler(new Vector3(0, 1, 0), Math.PI / 10,
				0.0, 0.0);
		// example for slam pos
		Pose3 tWorldToRobot = pose3FromEuler(new Vector3(3, 2, 0), 0.0, 0.0,
				Math.PI / 4);

		Pose3 pInRobot = transform(tWorldToRobot, pInWorld);
		System.out.println(toString(pInRobot));
		Pose3 pInWorldNew = transformInverse(tWorldToRobot, pInRobot);
		System.out.println(toString(pInWorld));
		System.out.println(toString(pInWorldNew));
	}

	/**
	 * Rotation matrix from Euler angles - rotation about: X (roll) -> Y(pitch)
	 * -> Z(yaw). roll, pitch, yaw in <-PI/2, PI/2>
	 * @param roll
	 * @param pitch
	 * @param yaw
	 * @return rotation matrix
	 */
	static public Matrix33 matrix33FromEuler(double roll, double pitch,
			double yaw) {
		Matrix33 a = new Matrix33();
		double sg = Math.sin(roll), cg = Math.cos(roll);
		double sb = Math.sin(pitch), cb = Math.cos(pitch);
		double sa = Math.sin(yaw), ca = Math.cos(yaw);

		a.m00 = ca * cb;
		a.m01 = ca * sb * sg - sa * cg;
		a.m02 = ca * sb * cg + sa * sg;
		a.m10 = sa * cb;
		a.m11 = sa * sb * sg + ca * cg;
		a.m12 = sa * sb * cg - ca * sg;
		a.m20 = -sb;
		a.m21 = cb * sg;
		a.m22 = cb * cg;
		return a;
	}

	/**
	 * C= A * B
	 * 
	 * @param A
	 * @param B
	 * @return C
	 */
	static public Matrix33 mult(Matrix33 A, Matrix33 B) {
		// note: temps needed in case C and A or B refer to the same matrix
		double a00 = A.m00 * B.m00 + A.m01 * B.m10 + A.m02 * B.m20;
		double a01 = A.m00 * B.m01 + A.m01 * B.m11 + A.m02 * B.m21;
		double a02 = A.m00 * B.m02 + A.m01 * B.m12 + A.m02 * B.m22;

		double a10 = A.m10 * B.m00 + A.m11 * B.m10 + A.m12 * B.m20;
		double a11 = A.m10 * B.m01 + A.m11 * B.m11 + A.m12 * B.m21;
		double a12 = A.m10 * B.m02 + A.m11 * B.m12 + A.m12 * B.m22;

		double a20 = A.m20 * B.m00 + A.m21 * B.m10 + A.m22 * B.m20;
		double a21 = A.m20 * B.m01 + A.m21 * B.m11 + A.m22 * B.m21;
		double a22 = A.m20 * B.m02 + A.m21 * B.m12 + A.m22 * B.m22;

		Matrix33 C = new Matrix33();
		C.m00 = a00;
		C.m01 = a01;
		C.m02 = a02;
		C.m10 = a10;
		C.m11 = a11;
		C.m12 = a12;
		C.m20 = a20;
		C.m21 = a21;
		C.m22 = a22;
		return C;
	}

	/**
	 * c = A * b
	 * 
	 * @param A
	 * @param b
	 * @return c
	 */
	static public Vector3 mult(Matrix33 A, Vector3 b) {
		Vector3 c = new Vector3();
		// note: temps needed in case b and c refer to the same vector
		double x = A.m00 * b.x + A.m01 * b.y + A.m02 * b.z;
		double y = A.m10 * b.x + A.m11 * b.y + A.m12 * b.z;
		double z = A.m20 * b.x + A.m21 * b.y + A.m22 * b.z;

		c.x = x;
		c.y = y;
		c.z = z;
		return c;
	}

	/**
	 * C=A^T * B
	 * 
	 * @param A
	 * @param B
	 * @return C
	 */
	static public Matrix33 multByTranspose(Matrix33 A, Matrix33 B) {
		Matrix33 C = new Matrix33();
		// note: temps needed in case C and A or B refer to the same matrix
		double a00 = A.m00 * B.m00 + A.m10 * B.m10 + A.m20 * B.m20;
		double a01 = A.m00 * B.m01 + A.m10 * B.m11 + A.m20 * B.m21;
		double a02 = A.m00 * B.m02 + A.m10 * B.m12 + A.m20 * B.m22;

		double a10 = A.m01 * B.m00 + A.m11 * B.m10 + A.m21 * B.m20;
		double a11 = A.m01 * B.m01 + A.m11 * B.m11 + A.m21 * B.m21;
		double a12 = A.m01 * B.m02 + A.m11 * B.m12 + A.m21 * B.m22;

		double a20 = A.m02 * B.m00 + A.m12 * B.m10 + A.m22 * B.m20;
		double a21 = A.m02 * B.m01 + A.m12 * B.m11 + A.m22 * B.m21;
		double a22 = A.m02 * B.m02 + A.m12 * B.m12 + A.m22 * B.m22;

		C.m00 = a00;
		C.m01 = a01;
		C.m02 = a02;
		C.m10 = a10;
		C.m11 = a11;
		C.m12 = a12;
		C.m20 = a20;
		C.m21 = a21;
		C.m22 = a22;
		return C;
	}

	/**
	 * C=transpose(A)*b
	 * 
	 * @param A
	 * @param b
	 * @return C
	 */
	static public Vector3 multByTranspose(Matrix33 A, Vector3 b) {
		Vector3 c = new Vector3();
		// note: temps needed in case b and c refer to the same vector
		double x = A.m00 * b.x + A.m10 * b.y + A.m20 * b.z;
		double y = A.m01 * b.x + A.m11 * b.y + A.m21 * b.z;
		double z = A.m02 * b.x + A.m12 * b.y + A.m22 * b.z;

		c.x = x;
		c.y = y;
		c.z = z;
		return c;
	}

	/**
	 * create pose from translation and rotation matrix
	 * 
	 * @param pos
	 * @param rot
	 * @return
	 */
	static public Pose3 pose3(Vector3 pos, Matrix33 rot) {
		Pose3 T = new Pose3();
		T.pos = pos;
		T.rot = rot;
		return T;
	}

	/**
	 * create pose from translation and euler angles
	 * 
	 * @param pos
	 * @param roll
	 * @param pitch
	 * @param yaw
	 * @return pose
	 */
	static public Pose3 pose3FromEuler(Vector3 pos, double roll, double pitch,
			double yaw) {
		Pose3 T = new Pose3();
		T.pos = pos;
		T.rot = matrix33FromEuler(roll, pitch, yaw);
		return T;
	}

	/**
	 * subtract vectors c=a-b
	 * 
	 * @param a
	 * @param b
	 * @return c
	 */
	static public Vector3 sub(Vector3 a, Vector3 b) {
		Vector3 c = new Vector3();
		c.x = a.x - b.x;
		c.y = a.y - b.y;
		c.z = a.z - b.z;
		return c;
	}

	/**
	 * to tring
	 * 
	 * @param p
	 * @return
	 */
	public static String toString(Matrix33 p) {
		String s = new String();
		s = "[[" + Double.toString(p.m00) + ", " + Double.toString(p.m01)
				+ ", " + Double.toString(p.m02) + "]\n";
		s += "     [" + Double.toString(p.m10) + ", " + Double.toString(p.m11)
				+ ", " + Double.toString(p.m12) + "]\n";
		s += "     [" + Double.toString(p.m20) + ", " + Double.toString(p.m21)
				+ ", " + Double.toString(p.m22) + "]]";
		return s;

	}

	/**
	 * o string
	 * 
	 * @param p
	 * @return
	 */
	public static String toString(Pose3 p) {
		String s = new String();
		s = "rot=" + toString(p.rot) + "\n";
		s += "pos=" + toString(p.pos);
		return s;

	}

	/**
	 * to string
	 * 
	 * @param p
	 * @return
	 */
	public static String toString(Vector3 p) {
		String s = new String();
		s = "[" + Double.toString(p.x) + ", " + Double.toString(p.y) + ", "
				+ Double.toString(p.z) + "]";
		return s;

	}

	/**
	 * Transform pose from local to world coordinates, poses T = [R, p], T1 =
	 * [R1, p1], T2 = [R2, p2]. T2 = [R * R1, R * p1 + p]
	 * 
	 * @param T
	 * @param T1
	 * @return T2
	 */
	static public Pose3 transform(Pose3 T, Pose3 T1) {
		Pose3 T2 = new Pose3();
		T2.rot = mult(T.rot, T1.rot);
		T2.pos = transform(T, T1.pos);
		return T2;
	}

	/**
	 * Transform 3D point from local to world coordinates, pose T = [R, p]. b =
	 * R * a + p;
	 * @param T
	 * @param a
	 * @return b
	 */
	static public Vector3 transform(Pose3 T, Vector3 a) {
		Vector3 b = mult(T.rot, a);
		b.x = b.x + T.pos.x;
		b.y = b.y + T.pos.y;
		b.z = b.z + T.pos.z;
		return b;
	}

	/**
	 * Transform pose from global to local coordinates, poses T = [R, p], T1 =
	 * [R1, p1], T2 = [R2, p2]. T2 = [R^T * R1, R^T * (p1 - p)]
	 * 
	 * @param T
	 * @param T1
	 * @return T2
	 */
	static public Pose3 transformInverse(Pose3 T, Pose3 T1) {
		Pose3 T2 = new Pose3();
		T2.rot = multByTranspose(T.rot, T1.rot);
		T2.pos = transformInverse(T, T1.pos);
		return T2;
	}

	/**
	 * Transform 3D point from world to local coordinates, pose T = [R, p]. b =
	 * R^T * (a - p)
	 * 
	 * @param T
	 * @param a
	 * @return b
	 */
	static public Vector3 transformInverse(Pose3 T, Vector3 a) {
		Vector3 b = sub(a, T.pos);
		b = multByTranspose(T.rot, b);
		return b;
	}

	/**
	 * transpose matrix
	 * 
	 * @param A
	 * @return A^T
	 */
	static public Matrix33 transpose(Matrix33 A) {
		Matrix33 B = new Matrix33();
		B.m00 = A.m00;
		B.m01 = A.m10;
		B.m02 = A.m20;
		B.m10 = A.m01;
		B.m11 = A.m11;
		B.m12 = A.m21;
		B.m20 = A.m02;
		B.m21 = A.m12;
		B.m22 = A.m22;
		return B;
	}
}
