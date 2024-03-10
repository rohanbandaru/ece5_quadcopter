package orientation;

import static drone.DroneConstants.*;
import static java.lang.Math.*;

public class Orientation {

    Quaternion orientation_raw_gyro = Quaternion.IDENTITY;
    Quaternion orientation = Quaternion.IDENTITY;

    public Quaternion from_gyro_rates(double dt, Vector3 gyro_rates) {
        double angle = dt * gyro_rates.norm();
        Vector3 n = gyro_rates.normalized();
        double sa = sin(angle / 2);
        return new Quaternion(cos(angle / 2), n.x() * sa, n.y() * sa, n.z() * sa);
    }

    public void update(double dt, Vector3 gyro_rates, Vector3 accel, double thrust) {
        orientation_raw_gyro = orientation_raw_gyro.mul(from_gyro_rates(dt, gyro_rates));

        // accelerations are body-frame

        // Estimate acceleration due to motors, drag, etc.
        Vector3 expected_accel = Vector3.of(0, 0, thrust/MASS);
        Vector3 compensated_accel = accel.add(expected_accel.scale(-1));

        Vector3 gravity = Vector3.of(1, 0, 0).rotatedBy(orientation);






    }


}
