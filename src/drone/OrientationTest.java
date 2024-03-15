package drone;

import math.Vector3;
import pose.AltitudeFuser;
import pose.Orientation;
import sensors.BMP388;
import sensors.MPU6050;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.channels.SocketChannel;
import java.time.Duration;
import java.time.Instant;

public class OrientationTest {
    void main() throws InterruptedException, IOException {
        System.out.println("started");
        Orientation ori = new Orientation();

        try (var socket = DatagramChannel.open();
             var baro = BMP388.withDefaults(1);
             var imu = new MPU6050(1)) {
            imu.calibrate(Vector3.K);
            var af = new AltitudeFuser();

            //ori.initFromAccel(imu.read().accel());

            System.out.println("started");
            Instant lastUpdate = Instant.now();
            while(!Thread.interrupted()) {
                var now = Instant.now();
                var dt = Duration.between(lastUpdate, now).toNanos() / 1.e9;
                var imuReading = imu.read();
                var barometerReading = baro.read();

                ori.update(dt, imuReading.gyro(), imuReading.accel(), 0);
                af.update(dt, ori.globalAccel.z(), imuReading.accelVariance().rotatedBy(ori.orientation).z(), barometerReading.altitude(), barometerReading.altitudeVariance());

                //System.out.println(baro.read());
                //System.out.println(imuReading);
                System.out.printf("%3.3f,%3.3f,%3.3f%n", barometerReading.altitude(), af.altitude(), af.verticalVelocity());
                var orientation = ori.orientation;
                socket.send(ByteBuffer.wrap(orientation.asBytes()), new InetSocketAddress("10.42.42.2", 4444));


                lastUpdate = now;
            }
        }
    }
}
