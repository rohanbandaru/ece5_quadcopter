package orientation;

import math.*;

import static drone.DroneConstants.*;
import static java.lang.Math.*;

public class Orientation {

    public Quaternion orientation = Quaternion.IDENTITY;
    public Quaternion orientationRawGyro = Quaternion.IDENTITY;
    public Quaternion orientationRawAccel = Quaternion.IDENTITY;
    public Vector3 globalAccel = Vector3.ZERO;

    public void update(double dt, Vector3 gyroRates, Vector3 bodyAccel, double totalThrust) {
        orientationRawGyro = orientation.mul(fromGyroRates(dt, gyroRates)); // priori orientation

        Vector3 bodyGravityAccel = compensateAccel(bodyAccel, totalThrust);

        orientationRawAccel = Quaternion.rotationBetween(bodyGravityAccel, Vector3.of(0, 0, 1));

        // luh calm complementary filter
        final double gyroTrust = 0;
        orientation = orientationRawAccel.fractional(1-gyroTrust).mul(orientationRawGyro.fractional(gyroTrust));
        // orientation = Quaternion.slerp(orientationRawAccel, orientationRawGyro, gyroTrust);

        // ACCELERATION IN GLOBAL-FRAME
        globalAccel = bodyAccel.rotatedBy(orientation).add(Vector3.of(0, 0, -1));
    }

    public void initFromAccel(Vector3 bodyAccel) {
        // when quad is sitting still, initialize starting orientation from gravity vector
        orientation = Quaternion.rotationBetween(bodyAccel, Vector3.of(0, 0, 1));
    }

    public Vector3 compensateAccel(Vector3 bodyAccel, double totalThrust) {
        // ACCELERATIONS IN BODY-FRAME
        // Estimate acceleration due to motors, drag, etc.
        Vector3 expectedAccel = Vector3.of(0, 0, totalThrust/MASS);
        Vector3 compensatedAccel = bodyAccel.add(expectedAccel.scale(-1));
        /*
        If compensatedAccel has a magnitude =/= 1, that means there is still
         some acceleration we failed to account for in the first step. So, subtract some
         vertical component from compensatedAccel such that the resultant vector has a magnitude closer to 1
         */
        double horizontalComponent = Vector3.of(compensatedAccel.x(), compensatedAccel.y(), 0).norm();
        if (horizontalComponent < 1) {
            // Find vertical component to make compensatedAccel have magnitude 1
            double vertical = sqrt(1-pow(horizontalComponent, 2));
            double diff = compensatedAccel.z()-vertical;
            diff = copySign(min(abs(diff), 0.1), signum(diff)); // Upper bound on the correction
            compensatedAccel = compensatedAccel.add(Vector3.K.scale(-diff));
        }
        return compensatedAccel.normalized();
    }

    public Quaternion fromGyroRates(double dt, Vector3 gyroRates) {
        double angle = dt * gyroRates.norm();
        Vector3 n = gyroRates.normalized();
        double sa = sin(angle / 2);
        return new Quaternion(cos(angle / 2), n.x() * sa, n.y() * sa, n.z() * sa);
    }

}
