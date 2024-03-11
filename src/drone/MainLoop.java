package drone;

import math.Quaternion;
import math.Vector3;
import orientation.Orientation;
import sensors.BMP388;
import sensors.MPU6050;

import static java.lang.Math.*;

public class MainLoop {
    public static void main(String[] args) {

        Orientation ori = new Orientation();

        //TODO Init and calibrate sensors
        //TODO Arm ESCs
        //TODO Check receiver inputs
        ori.initFromAccel(acceleration);

        // Main loop
        double dt;
        while(true) {
            //TODO Poll sensors

            ori.update(dt, gyroRates, acceleration, totalThrust);

            Quaternion error = targetOri.mul(ori.orientation.conj());
            double yawError = error.x1();
            double pitchError = error.x2();
            double rollError = error.x3();

            //TODO PIDs
            pitch_torque = pitchPid.correction(dt, pitchError);

            //TODO Motor thrust
            motorFR.setThrust(throttleThrust - pitchTorque/motorOffset + ...);

        }





    }
}
