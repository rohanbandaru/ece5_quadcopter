package orientation;

import static java.lang.Math.pow;

public record Vector3(double x, double y, double z) {

	public static Vector3 of(double x, double y, double z) {
		return new Vector3(x, y, z);
	}

	public static Vector3 zero() {
		return ZERO;
	}

	public static Vector3 ofQuaternion(Quaternion q) {
		return new Vector3(q.x1(), q.x2(), q.x3());
	}

	public static final Vector3 ZERO = new Vector3(0, 0, 0);
	public static final Vector3 I = new Vector3(1, 0, 0);
	public static final Vector3 J = new Vector3(0, 1, 0);
	public static final Vector3 K = new Vector3(0, 0, 1);

	public double norm() {
		return Math.sqrt(x*x + y*y + z*z);
	}
	public Vector3 scale(double s) {
		return new Vector3(x * s, y * s, z * s);
	}
	public Vector3 normalized() {
		double mag = this.norm();
		if(mag == 0) return this; // no divide by 0
		return this.scale(1 / mag);
	}

	public Vector3 add(Vector3 b) {
		return Vector3.add(this, b);
	}

	public Vector3 sub(Vector3 b) {
		return add(b.scale(-1));
	}

	public static Vector3 add(Vector3 a, Vector3 b) {
		return new Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
	}

	public double dot(Vector3 b) {
		return Vector3.dot(this, b);
	}

	public static double dot(Vector3 a, Vector3 b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	public Vector3 cross(Vector3 b) {
		return Vector3.cross(this, b);
	}

	public Vector3 projectedOnto(Vector3 b) {
		return b.scale(dot(b) / pow(b.norm(), 2));
	}

	public double comp(Vector3 b) {
		return dot(b) / b.norm();
	}

	public Vector3 hadamard(Vector3 b) {
		return new Vector3(x * b.x, y * b.y, z * b.z);
	}

	public static Vector3 cross(Vector3 a, Vector3 b) {
		return new Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
	}

	public static Vector3 rotateBy(Vector3 v, Quaternion q) {
		return Vector3.ofQuaternion(q.mul(Quaternion.ofVector3(v)).mul(q.inv()));
	}

	public Vector3 rotatedBy(Quaternion q) {
		return Vector3.rotateBy(this, q);
	}
}
