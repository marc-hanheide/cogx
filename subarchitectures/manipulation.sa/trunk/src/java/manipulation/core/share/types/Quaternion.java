package manipulation.core.share.types;

/*************************************************************************
 * Compilation: javac Quaternion.java Execution: java Quaternion
 * 
 * Data type for quaternions.
 * 
 * http://mathworld.wolfram.com/Quaternion.html
 * 
 * The data type is "immutable" so once you create and initialize a Quaternion,
 * you cannot change it.
 * 
 * % java Quaternion
 * 
 *************************************************************************/

public class Quaternion {
	private final double x0, x1, x2, x3;

	// create a new object with the given components
	public Quaternion(double x0, double x1, double x2, double x3) {
		this.x0 = x0;
		this.x1 = x1;
		this.x2 = x2;
		this.x3 = x3;
	}

	// return a string representation of the invoking object
	public String toString() {
		return x0 + " + " + x1 + "i + " + x2 + "j + " + x3 + "k";
	}

	// return the quaternion norm
	public double norm() {
		return Math.sqrt(x0 * x0 + x1 * x1 + x2 * x2 + x3 * x3);
	}

	// return the quaternion conjugate
	public Quaternion conjugate() {
		return new Quaternion(x0, -x1, -x2, -x3);
	}

	public Quaternion negate() {
		return new Quaternion(-x0, -x1, -x2, -x3);
	}

	// return a new Quaternion whose value is (this + b)
	public Quaternion plus(Quaternion b) {
		Quaternion a = this;
		return new Quaternion(a.x0 + b.x0, a.x1 + b.x1, a.x2 + b.x2, a.x3
				+ b.x3);
	}

	// scalar product
	public double dot(Quaternion q) {
		return (x0 * q.x0 + x1 * q.x1 + x2 * q.x2 + x3 * q.x3);
	}

	// return a new Quaternion whose value is (this * b)
	public Quaternion times(Quaternion b) {
		Quaternion a = this;
		double y0 = a.x0 * b.x0 - a.x1 * b.x1 - a.x2 * b.x2 - a.x3 * b.x3;
		double y1 = a.x0 * b.x1 + a.x1 * b.x0 + a.x2 * b.x3 - a.x3 * b.x2;
		double y2 = a.x0 * b.x2 - a.x1 * b.x3 + a.x2 * b.x0 + a.x3 * b.x1;
		double y3 = a.x0 * b.x3 + a.x1 * b.x2 - a.x2 * b.x1 + a.x3 * b.x0;
		return new Quaternion(y0, y1, y2, y3);
	}

	// return a new Quaternion whose value is the inverse of this
	public Quaternion inverse() {
		double d = x0 * x0 + x1 * x1 + x2 * x2 + x3 * x3;
		return new Quaternion(x0 / d, -x1 / d, -x2 / d, -x3 / d);
	}

	// return a / b
	public Quaternion divides(Quaternion b) {
		Quaternion a = this;
		return a.inverse().times(b);
	}

	// sample client for testing
	public static void main(String[] args) {
		Quaternion a = new Quaternion(3.0, 1.0, 0.0, 0.0);
		System.out.println("a = " + a);

		Quaternion b = new Quaternion(0.0, 5.0, 1.0, -2.0);
		System.out.println("b = " + b);

		System.out.println("norm(a)  = " + a.norm());
		System.out.println("conj(a)  = " + a.conjugate());
		System.out.println("a + b    = " + a.plus(b));
		System.out.println("a * b    = " + a.times(b));
		System.out.println("b * a    = " + b.times(a));
		System.out.println("a / b    = " + a.divides(b));
		System.out.println("a^-1     = " + a.inverse());
		System.out.println("a^-1 * a = " + a.inverse().times(a));
		System.out.println("a * a^-1 = " + a.times(a.inverse()));
	}

}