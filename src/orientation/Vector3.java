package orientation;

public record Vector3(double x, double y, double z) {
	public static Vector3 of(double x, double y, double z) {
		return new Vector3(x, y, z);
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
		return this.scale(1 / mag);
	}

	public Vector3 add(Vector3 b) {
		return Vector3.add(this, b);
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

	public static Vector3 cross(Vector3 a, Vector3 b) {
		return new Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
	}
}
