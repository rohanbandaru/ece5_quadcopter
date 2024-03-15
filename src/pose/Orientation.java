package pose;

import drone.DroneConstants;
import math.*;

import static drone.DroneConstants.*;
import static java.lang.Math.*;
import static sensors.MPU6050.G;

public class Orientation {

    public volatile Quaternion orientation = Quaternion.IDENTITY;
    public volatile Vector3 globalAccel = Vector3.ZERO;

    public void update(double dt, Vector3 gyroRates, Vector3 bodyAccel, double totalThrust) {
        var gyroQuat = orientation.mul(fromGyroRates(dt, Vector3.of(gyroRates.x(), gyroRates.y(), gyroRates.z())));
        var yawQuat = Quaternion.of(gyroQuat.x0(), 0, 0, gyroQuat.x3()).normalized();

        var bodyAccelNorm = bodyAccel.normalized();
        double accPitch = -asin(bodyAccelNorm.x());
        double accRoll = asin(bodyAccelNorm.y());

        var accelQuat = fromGyroRates(1, Vector3.of(accRoll, accPitch, 0));
        var accelQuatRot = yawQuat.mul(accelQuat);

        double alpha = 0.3;
        orientation = accelQuatRot.fractional(alpha).mul(gyroQuat.fractional(1 - alpha));

        // ACCELERATION IN GLOBAL-FRAME
        globalAccel = bodyAccel.rotatedBy(orientation).add(Vector3.of(0, 0, -G));
    }

    public void initFromAccel(Vector3 bodyAccel) {
        // when quad is sitting still, initialize starting orientation from gravity vector
        double accPitch = -asin(bodyAccel.normalized().x());
        double accRoll = asin(bodyAccel.normalized().y());
        orientation = fromGyroRates(1, Vector3.of(accRoll, accPitch, 0));
    }

    public Vector3 compensateAccel(Vector3 bodyAccel, double totalThrust) {
        // ACCELERATIONS IN BODY-FRAME
        // Estimate acceleration due to motors, drag, etc.
        Vector3 expectedAccel = Vector3.of(0, 0, totalThrust / MASS);
        Vector3 compensatedAccel = bodyAccel.add(expectedAccel.scale(-1));
        /*
        If compensatedAccel has a magnitude =/= 1, that means there is still
         some acceleration we failed to account for in the first step. So, subtract some
         vertical component from compensatedAccel such that the resultant vector has a magnitude closer to 1
         */
        double horizontalComponent = Vector3.of(compensatedAccel.x(), compensatedAccel.y(), 0).norm();
        if (horizontalComponent < 1) {
            // Find vertical component to make compensatedAccel have magnitude 1
            double vertical = sqrt(1 - pow(horizontalComponent, 2));
            double diff = compensatedAccel.z() - vertical;
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
