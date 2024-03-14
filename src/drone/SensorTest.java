package drone;

import math.Vector3;
import pose.AltitudeFuser;
import sensors.BMP388;
import sensors.MPU6050;

public class SensorTest {
    void main() throws InterruptedException {
        System.out.println("started");

        try (var baro = BMP388.withDefaults(1); var imu = new MPU6050(1)) {
            imu.calibrate(Vector3.K);

            System.out.println("started");
            for (int i = 0; i < 100; i++) {
                System.out.println(baro.read());
                System.out.println(imu.read());
            }
        }
    }
}
