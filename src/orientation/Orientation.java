package orientation;

import static java.lang.Math.*;

public class Orientation {

    Quaternion orientation_raw_gyro = Quaternion.IDENTITY;
    Quaternion orientation = Quaternion.IDENTITY;

    public void update(double dt, Vector3 gyro_rates, Vector3 accel) {
        double angle = dt * gyro_rates.norm();
        Vector3 n = gyro_rates.normalized();
        double sa = sin(angle / 2);
        Quaternion d_gyro = new Quaternion(cos(angle / 2), n.x() * sa, n.y() * sa, n.z() * sa);
        orientation_raw_gyro = orientation_raw_gyro.mul(d_gyro);

        // accelerations are body-frame

        Vector3 gravity = Vector3.of(1, 0, 0).rotatedBy(orientation_raw_gyro);


    }


}
