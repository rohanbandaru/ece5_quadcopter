package drone;

import math.Quaternion;
import sensors.AsyncPoll;
import sensors.BMP388;
import sensors.MPU6050;
import pose.AltitudeFuser;
import pose.Orientation;

import java.time.Duration;
import java.time.Instant;

public class MainLoop {

    public static void main(String[] args) {

        Orientation ori = new Orientation();
        AltitudeFuser altFilter = new AltitudeFuser();

        MPU6050 imu;
        BMP388 baro;
        AsyncPoll sensors = new AsyncPoll(imu, baro);

        //TODO Init and calibrate sensors
        //TODO Arm ESCs
        //TODO Check receiver inputs
        ori.initFromAccel(sensors.acceleration);

        // Main loop
        while(true) {
            //TODO Poll sensors
            Instant lastUpdate = Instant.now();
            var now = Instant.now();
			var dt = Duration.between(lastUpdate, now).toNanos() / 1.e9;

            /*
            ori.update(dt, sensors.gyroRates, sensors.acceleration, totalThrust);
            altFilter.update(dt, ori.globalAccel.z(), // accel measurement
                    imu.accelVariance().rotatedBy(ori.orientation.inv()).z(), // accel variance
                    sensors.altitude, // altitude measurement
                    baro.altitudeVariance()); // altitude variance

            Quaternion error = targetOri.mul(ori.orientation.conj());
            double yawError = error.x1();
            double pitchError = error.x2();
            double rollError = error.x3();

            //TODO PIDs
            pitch_torque = pitchPid.correction(dt, pitchError);

            //TODO Motor thrust
            motorFR.setThrust(throttleThrust - pitchTorque/motorOffset + ...);
            */

        }

    }

}
