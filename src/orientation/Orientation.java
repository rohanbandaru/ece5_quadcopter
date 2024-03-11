package orientation;

import static drone.DroneConstants.*;
import static java.lang.Math.*;

public class Orientation {

    Quaternion orientation_raw_gyro = Quaternion.IDENTITY;
    Quaternion orientation = Quaternion.IDENTITY;
    Vector3 global_accel = Vector3.ZERO;

    public void update(double dt, Vector3 gyro_rates, Vector3 body_accel, double thrust) {
        orientation_raw_gyro = orientation_raw_gyro.mul(from_gyro_rates(dt, gyro_rates));

        // ACCELERATIONS IN BODY-FRAME
        // Estimate acceleration due to motors, drag, etc.
        Vector3 expected_accel = Vector3.of(0, 0, thrust/MASS);
        Vector3 compensated_accel = body_accel.add(expected_accel.scale(-1));
        /*
        If compensated_accel has a magnitude =/= 1, that means there is still
         some acceleration we failed to account for in the first step. So, subtract some
         vertical component from compensated_accel such that the resultant vector has a magnitude closer to 1
         */
        double horizontal_component = Vector3.of(compensated_accel.x(), compensated_accel.y(), 0).norm();
        if (horizontal_component < 1) {
            // Find vertical component to make compensated_accel have magnitude 1
            double vertical = sqrt(1-pow(horizontal_component, 2));
            double diff = compensated_accel.z()-vertical;
            diff = copySign(min(abs(diff), 0.1), signum(diff)); // Upper bound on the correction
            compensated_accel = compensated_accel.add(Vector3.K.scale(-diff));
        }

        // luh calm complementary filter?
        Vector3 body_gravity_accel = compensated_accel.normalized();
        Quaternion orientation_raw_accel = 



        Vector3 body_gravity_gyro = Vector3.of(1, 0, 0).rotatedBy(orientation); // priori orientation
        Vector3 body_gravity = body_gravity_accel.scale(0.3).add(body_gravity_gyro.scale(0.7));

        orientation = body_gravity.;

        // ACCELERATIONS IN GLOBAL-FRAME
        global_accel = compensated_accel.add(body_gravity.scale(-1)).rotatedBy(orientation.inv());

    }

    public Quaternion from_gyro_rates(double dt, Vector3 gyro_rates) {
        double angle = dt * gyro_rates.norm();
        Vector3 n = gyro_rates.normalized();
        double sa = sin(angle / 2);
        return new Quaternion(cos(angle / 2), n.x() * sa, n.y() * sa, n.z() * sa);
    }

}
