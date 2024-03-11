package orientation;
import static java.lang.Math.*;

// mostly copied from https://introcs.cs.princeton.edu/java/32class/Quaternion.java.html
public record Quaternion(double x0, double x1, double x2, double x3) {
    // x0 + x1 * i + x2 * j + x3 * k

    public static final Quaternion IDENTITY = new Quaternion(1, 0, 0, 0);
    public static final Quaternion ZERO = new Quaternion(0, 0, 0, 0);
    public static final Quaternion I = new Quaternion(0, 1, 0, 0);
    public static final Quaternion J = new Quaternion(0, 0, 1, 0);
    public static final Quaternion K = new Quaternion(0, 0, 0, 1);

    // create a new object with the given components
    public static Quaternion of(double x0, double x1, double x2, double x3) {
        return new Quaternion(x0, x1, x2, x3);
    }
    public static Quaternion ofVector3(Vector3 v) {
        return new Quaternion(0, v.x(), v.y(), v.z());
    }

    // creates quaternion from euler angles in radians
    public static Quaternion ofEuler(double yaw, double pitch, double roll) {
        double cy = cos(yaw / 2);
        double cp = cos(pitch / 2);
        double cr = cos(roll / 2);
        double sy = sin(yaw / 2);
        double sp = sin(pitch / 2);
        double sr = sin(roll / 2);

        double x0 = cr * cp * cy + sr * sp * sy;
        double x1 = sr * cp * cy - cr * sp * sy;
        double x2 = cr * sp * cy + sr * cp * sy;
        double x3 = cr * cp * sy - sr * sp * cy;

        return new Quaternion(x0, x1, x2, x3);
    }

    // returns quaternion as array of euler angles
    public double[] toEuler() {
        double sinr_cosp = 2 * (x0 * x1 + x2 * x3);
        double cosr_cosp = 1 - 2 * (x1 * x1 + x2 * x2);
        double roll = atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (x0 * x2 + -x3 * x1);
        double pitch = 0;
        if (abs(sinp) >= 1)
            pitch = copySign(Math.PI / 2, pitch); // return 90 if out of range
        else
            pitch = asin(sinp);

        double siny_cosp = 2 * (x0 * x3 + x1 * x2);
        double cosy_cosp = 1 - 2 * (x2 * x2 + x3 * x3);
        double yaw = atan2(siny_cosp, cosy_cosp);

        return new double[]{yaw, pitch, roll};
    }

    // return a string representation of the invoking object
    public String toString() {
        return x0 + " + " + x1 + "i + " + x2 + "j + " + x3 + "k";
    }

    // return the quaternion norm
    public double norm() {
        return sqrt(x0 * x0 + x1 * x1 + x2 * x2 + x3 * x3);
    }

    public Quaternion normalized() {
        double mag = this.norm();
        if (mag == 0) return IDENTITY;
        return this.scale(1 / mag);
    }

    // return the quaternion conjugate
    public Quaternion conj() {
        return new Quaternion(x0, -x1, -x2, -x3);
    }

    // return a new quaternion whose value is the inverse of this
    public Quaternion inv() {
        double d = this.norm();
        return new Quaternion(x0 / d, -x1 / d, -x2 / d, -x3 / d);
    }

    // return a new orientation.Quaternion whose value is (this + b)
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

    // return a new orientation.Quaternion whose value is (this * b)
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

    public Quaternion fractional(double t) { // lerp between identity
        // might break idk. returns a fraction t of the original rotation
        return new Quaternion(1-t + t*x0, x1*t, x2*t, x3*t).normalized();
    }

    public static Quaternion slerp(Quaternion a, Quaternion b, double t) {
        return a.mul(a.inv().mul(b).pow(t));
    }

    public static Quaternion pow(Quaternion q, double n) {
        return q.ln().scale(n).exp();
    }

    public Quaternion pow(double n) {
        return Quaternion.pow(this, n);
    }

    public Quaternion ln() {
        double r  = sqrt(x1*x1+x2*x2+x3*x3);
        double t  = r > 0.00001 ? atan2(r, x0) / r : 0.0;
        double x = 0.5 * log(x0*x0+x1*x1+x2*x2+x3*x3);
        return new Quaternion(x, x1*t, x2*t, x3*t);
    }

    public Quaternion exp() {
        double r = sqrt(x1*x1+x2*x2+x3*x3);
        double et = Math.exp(x0);
        double s = r >= 0.00001 ? et * sin(r) / r : 0.0;
        return new Quaternion(et * cos(r), x1*s, x2*s, x3*s);
    }

    public Quaternion rotationBetween(Vector3 a, Vector3 b) {
        // a rotated by output is b
        
    }

}