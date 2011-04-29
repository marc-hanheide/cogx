/**
 * 
 */
package cogx.mathlib;

import cogx.Math.Matrix33;
import cogx.Math.Pose3;
import cogx.Math.Vector3;

/**
 * @author cogx
 * 
 */
public final class Functions {

	
	
	
	static Pose3 pose3(Vector3 pos, Matrix33 rot) {
		Pose3 T = new Pose3();
		T.pos = pos;
		T.rot = rot;
		return T;
	}

	static Pose3 pose3FromEuler(Vector3 pos, double roll, double pitch, double yaw) {
		Pose3 T = new Pose3();
		T.pos = pos;
		T.rot = matrix33FromEuler(roll, pitch, yaw);
		return T;
	}

	
	/** 
	 * Rotation matrix from Euler angles - rotation about: X (roll) -> Y(pitch) -> Z(yaw).
	 *   roll, pitch, yaw  in <-PI/2, PI/2>
	 */
	static public Matrix33 matrix33FromEuler(double roll, double pitch, double yaw)
	{
		Matrix33 a=new Matrix33();
	  double sg = Math.sin(roll), cg = Math.cos(roll);
	  double sb = Math.sin(pitch), cb = Math.cos(pitch);
	  double sa = Math.sin(yaw), ca = Math.cos(yaw);

	  a.m00 = ca*cb;  a.m01 = ca*sb*sg - sa*cg;   a.m02 = ca*sb*cg + sa*sg;
	  a.m10 = sa*cb;  a.m11 = sa*sb*sg + ca*cg;   a.m12 = sa*sb*cg - ca*sg;
	  a.m20 = -sb;    a.m21 = cb*sg;              a.m22 = cb*cg;
	  return a;
	}
	
	// /**
	// * Set from a 4x4 float array in row major order, where the upper 3x3 part
	// * corresponds to the rotation matrix.
	// */
	// static void setRow44(Pose3 T, float m[])
	// {
	// setRow44(T.rot, m);
	// set(T.pos, (double)m[3], (double)m[7], (double)m[11]);
	// }
	//
	// /**
	// * Set from a 4x4 double array in row major order, where the upper 3x3
	// part
	// * corresponds to the rotation matrix.
	// */
	// static void setRow44(Pose3 T, double m[])
	// {
	// setRow44(T.rot, m);
	// set(T.pos, m[3], m[7], m[11]);
	// }
	//
	// /**
	// * Set from a 4x4 float array in column major order, where the upper 3x3
	// part
	// * corresponds to the rotation matrix.
	// */
	// static void setColumn44(Pose3 T, float m[])
	// {
	// setColumn44(T.rot, m);
	// set(T.pos, (double)m[12], (double)m[13], (double)m[14]);
	// }
	//
	// /**
	// * Set from a 4x4 double array in column major order, where the upper 3x3
	// part
	// * corresponds to the rotation matrix.
	// */
	// static void setColumn44(Pose3 T, double m[])
	// {
	// setColumn44(T.rot, m);
	// set(T.pos, m[12], m[13], m[14]);
	// }
	//
	// /**
	// * Get to a 4x4 float array in row major order, where the upper 3x3 part
	// * corresponds to the rotation matrix.
	// */
	// static void getRow44(Pose3 &T, float m[])
	// {
	// getRow44(T.rot, m);
	// m[3] = (float)T.pos.x;
	// m[7] = (float)T.pos.y;
	// m[11] = (float)T.pos.z;
	// m[12] = m[13] = m[14] = 0.;
	// m[15] = 1.;
	// }
	//
	// /**
	// * Get to a 4x4 double array in row major order, where the upper 3x3 part
	// * corresponds to the rotation matrix.
	// */
	// static void getRow44(Pose3 &T, double m[])
	// {
	// getRow44(T.rot, m);
	// m[3] = T.pos.x;
	// m[7] = T.pos.y;
	// m[11] = T.pos.z;
	// m[12] = m[13] = m[14] = 0.;
	// m[15] = 1.;
	// }
	//
	// /**
	// * Get to a 4x4 float array in column major order, where the upper 3x3
	// part
	// * corresponds to the rotation matrix.
	// */
	// static void getColumn44(Pose3 &T, float m[])
	// {
	// getColumn44(T.rot, m);
	// m[12] = (float)T.pos.x;
	// m[13] = (float)T.pos.y;
	// m[14] = (float)T.pos.z;
	// m[3] = m[7] = m[11] = 0.;
	// m[15] = 1.;
	// }
	//
	// /**
	// * Get to a 4x4 double array in column major order, where the upper 3x3
	// part
	// * corresponds to the rotation matrix.
	// */
	// static void getColumn44(Pose3 &T, double m[])
	// {
	// getColumn44(T.rot, m);
	// m[12] = T.pos.x;
	// m[13] = T.pos.y;
	// m[14] = T.pos.z;
	// m[3] = m[7] = m[11] = 0.;
	// m[15] = 1.;
	// }
	//
	// static void setIdentity(Pose3 &T)
	// {
	// setIdentity(T.rot);
	// setZero(T.pos);
	// }
	//
	// static bool isIdentity(Pose3 &T)
	// {
	// return isIdentity(T.rot) & isZero(T.pos);
	// }
	//
	// static bool isFinite(Pose3 &T)
	// {
	// return isFinite(T.rot) & isFinite(T.pos);
	// }
	//
	// /**
	// * Returns true if A and B's elems are within epsilon of each other.
	// */
	// static bool equals(Pose3 A, Pose3 B, double eps)
	// {
	// return equals(A.pos, B.pos, eps) & equals(A.rot, B.rot, eps);
	// }

	/**
	 * Assigns inverse to B, assuming R is orthonormal, i.e. inv(R) = R^T, i.e.
	 * R is a rotation matrix. poses T1 = [R1, p1], T2 = [R2, p2] T2 = [R1^T, -
	 * R1^T * p1]
	 */
	static public Pose3 inverse(Pose3 T1) {
		Pose3 T2 = new Pose3();
		T2.rot = transpose(T1.rot);
		T2.pos=new Vector3();
		T2.pos.x = -T1.pos.x;
		T2.pos.y = -T1.pos.y;
		T2.pos.z = -T1.pos.z;
		T2.pos = mult(T2.rot, T2.pos);
		return T2;
	}

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

	/**
	 * Transform 3D point from local to world coordinates, pose T = [R, p]. b =
	 * R * a + p;
	 */
	static Vector3 transform(Pose3 T, Vector3 a) {
		Vector3 b = mult(T.rot, a);
		b.x = T.pos.x;
		b.y = T.pos.y;
		b.z = T.pos.z;
		return b;
	}

	/**
	 * Transform pose from local to world coordinates, poses T = [R, p], T1 =
	 * [R1, p1], T2 = [R2, p2]. T2 = [R * R1, R * p1 + p]
	 */
	static public Pose3 transform(Pose3 T, Pose3 T1) {
		Pose3 T2 = new Pose3();
		T2.rot = mult(T.rot, T1.rot);
		T2.pos = transform(T, T1.pos);
		return T2;
	}

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

	public static String toString(Pose3 p) {
		String s = new String();
		s="rot="+toString(p.rot)+"\n";
		s+="pos="+toString(p.pos);
		return s;
		
	}
	public static String toString(Vector3 p) {
		String s = new String();
		s="["+Double.toString(p.x)+", "+Double.toString(p.y)+", "+ Double.toString(p.z)+"]";
		return s;
		
	}
	public static String toString(Matrix33 p) {
		String s = new String();
		s="[["+Double.toString(p.m00)+", "+Double.toString(p.m01)+", "+ Double.toString(p.m02)+"]\n";
		s+="     ["+Double.toString(p.m10)+", "+Double.toString(p.m11)+", "+ Double.toString(p.m12)+"]\n";
		s+="     ["+Double.toString(p.m20)+", "+Double.toString(p.m21)+", "+ Double.toString(p.m22)+"]]";
		return s;
		
	}
	
	public static void main(String[] argv) {
		Pose3 p=pose3FromEuler(new Vector3(0, 1, 0), Math.PI/5, 0.0, 0.0);
		Pose3 pi=inverse(p);
		Pose3 pi2=inverse(pi);
		System.out.println(toString(p));
		System.out.println(toString(pi));
		System.out.println(toString(pi2));
	}
}
