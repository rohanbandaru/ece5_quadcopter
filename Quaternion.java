// mostly copied from https://introcs.cs.princeton.edu/java/32class/Quaternion.java.html
public final class Quaternion {
    // x0 + x1 * i + x2 * j + x3 * k

    public static final Quaternion IDENTITY = new Quaternion(1, 0, 0, 0);
    public static final Quaternion ZERO = new Quaternion(0, 0, 0, 0);
    public static final Quaternion I = new Quaternion(0, 1, 0, 0);
    public static final Quaternion J = new Quaternion(0, 0, 1, 0);
    public static final Quaternion K = new Quaternion(0, 0, 0, 1);

    private final double x0, x1, x2, x3;

    // create a new object with the given components
    public Quaternion(double x0, double x1, double x2, double x3) {
        this.x0 = x0;
        this.x1 = x1;
        this.x2 = x2;
        this.x3 = x3;
    }

    // creates quaternion from euler angles in radians
    public Quaternion(double yaw, double pitch, double roll) {
        double cy = Math.cos(yaw / 2);
        double cp = Math.cos(pitch / 2);
        double cr = Math.cos(roll / 2);
        double sy = Math.sin(yaw / 2);
        double sp = Math.sin(pitch / 2);
        double sr = Math.sin(roll / 2);

        this.x0 = cr * cp * cy + sr * sp * sy;
        this.x1 = sr * cp * cy - cr * sp * sy;
        this.x2 = cr * sp * cy + sr * cp * sy;
        this.x3 = cr * cp * sy - sr * sp * cy;
    }

    // returns quaternion as array of euler angles
    public double[] toEuler() {
        double sinr_cosp = 2 * (x0 * x1 + x2 * x3);
        double cosr_cosp = 1 - 2 * (x1 * x1 + x2 * x2);
        double roll = Math.atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (x0 * x2 + -x3 * x1);
        double pitch = 0;
        if (Math.abs(sinp) >= 1)
            pitch = Math.copySign(Math.PI / 2, pitch); // return 90 if out of range
        else
            pitch = Math.asin(sinp);

        double siny_cosp = 2 * (x0 * x3 + x1 * x2);
        double cosy_cosp = 1 - 2 * (x2 * x2 + x3 * x3);
        double yaw = Math.atan2(siny_cosp, cosy_cosp);

        double ypr[] = {yaw, pitch, roll};
        return ypr;
    }

    // return a string representation of the invoking object
    public String toString() {
        return x0 + " + " + x1 + "i + " + x2 + "j + " + x3 + "k";
    }

    // return the quaternion norm
    public double norm() {
        return Math.sqrt(x0 * x0 + x1 * x1 + x2 * x2 + x3 * x3);
    }

    public Quaternion normalized() {
        double mag = this.norm();
        return this.scale(1 / mag);
    }

    // return the quaternion conjugate
    public Quaternion conj() {
        return new Quaternion(x0, -x1, -x2, -x3);
    }

    // return a new Quaternion whose value is the inverse of this
    public Quaternion inv() {
        double d = x0 * x0 + x1 * x1 + x2 * x2 + x3 * x3;
        return new Quaternion(x0 / d, -x1 / d, -x2 / d, -x3 / d);
    }

    // return a new Quaternion whose value is (this + b)
    public Quaternion add(Quaternion b) {
        return Quaternion.add(this, b);
    }

    public static Quaternion add(Quaternion a, Quaternion b) {
        return new Quaternion(a.x0 + b.x0, a.x1 + b.x1, a.x2 + b.x2, a.x3 + b.x3);
    }

    // multiplies values by a scalar
    public Quaternion scale(double s) {
        return new Quaternion(x0 * s, x1 * s, x2 * s, x3 * s);
    }

    // return a new Quaternion whose value is (this * b)
    public Quaternion mul(Quaternion b) {
        return Quaternion.mul(this, b);
    }

    public static Quaternion mul(Quaternion a, Quaternion b) {
        double y0 = a.x0 * b.x0 - a.x1 * b.x1 - a.x2 * b.x2 - a.x3 * b.x3;
        double y1 = a.x0 * b.x1 + a.x1 * b.x0 + a.x2 * b.x3 - a.x3 * b.x2;
        double y2 = a.x0 * b.x2 - a.x1 * b.x3 + a.x2 * b.x0 + a.x3 * b.x1;
        double y3 = a.x0 * b.x3 + a.x1 * b.x2 - a.x2 * b.x1 + a.x3 * b.x0;
        return new Quaternion(y0, y1, y2, y3);
    }

}