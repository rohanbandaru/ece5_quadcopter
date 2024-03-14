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

            //ori.initFromAccel(imu.read().accel());

            System.out.println("started");
            Instant lastUpdate = Instant.now();
            while(!Thread.interrupted()) {
                var now = Instant.now();
                var dt = Duration.between(lastUpdate, now).toNanos() / 1.e9;
                var imuReading = imu.read();
                ori.update(dt, imuReading.gyro(), imuReading.accel(), 0);
                //System.out.println(baro.read());
                //System.out.println(imuReading);
                System.out.println(ori.globalAccel.z());
                var orientation = ori.orientation;
                socket.send(ByteBuffer.wrap(orientation.asBytes()), new InetSocketAddress("10.42.42.2", 4444));


                lastUpdate = now;
            }
        }
    }
}
