package drone;

import math.PID;
import math.Quaternion;
import math.Vector3;
import sensors.BMP388;
import sensors.MPU6050;
import pose.AltitudeFuser;
import pose.Orientation;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.time.Duration;
import java.time.Instant;

import static drone.DroneConstants.*;

public class MainLoop {

    void main() throws InterruptedException, IOException {
        System.out.println("started");
        try(
            var baro = BMP388.withDefaults(1);
            var imu = new MPU6050(1);
            var fr = new Motor(FRONT_LEFT_GPIO, 1.1);
            var bl = new Motor(BACK_RIGHT_GPIO, 1.1)) {
            Orientation ori = new Orientation();
            imu.calibrate(Vector3.K);
            var af = new AltitudeFuser();

            PID balancePitch = new PID(0.5, 0, 0);

            ori.initFromAccel(imu.read().accel());
            Quaternion targetOri = ori.orientation;

            System.out.println("started");
            Instant lastUpdate = Instant.now();

            var socket = DatagramChannel.open();
            var addr = new InetSocketAddress("10.42.42.2", 4444);

//            fr.arm();
//            bl.arm();

            while(!Thread.interrupted()) {
                var now = Instant.now();
                var dt = Duration.between(lastUpdate, now).toNanos() / 1.e9;
                var imuReading = imu.read();
                var barometerReading = baro.read();

                ori.update(dt, imuReading.gyro(), imuReading.accel(), 0);
                af.update(dt, ori.globalAccel.z(), imuReading.accelVariance().rotatedBy(ori.orientation).z(), barometerReading.altitude(), barometerReading.altitudeVariance());

                var oriError = targetOri.mul(ori.orientation.conj());
                var quadFrameTransform = Quaternion.fromAxisAngle(Math.PI/4, Vector3.of(0, 0, 1)).normalized();
                oriError = quadFrameTransform.mul(oriError);
                //Vector3 errorAngles = Quaternion.decompose(oriError);
                Vector3 rotK = Vector3.K.rotatedBy(oriError);
                Vector3 rotIJ = Vector3.of(Math.sqrt(2)/2, -Math.sqrt(2)/2, 0).rotatedBy(oriError);
                // System.out.println(rotI);
                double pitchError = Math.atan2(rotK.x(), rotK.z());
                double rollError = Math.atan2(rotK.y(), rotK.z());
                double yawError = Math.atan2(rotIJ.y(), rotIJ.x());
                //System.out.println(ori.orientation);]
                //System.out.println(targetOri);
                var correction = balancePitch.correction(dt, rollError);
                double throttle = 0.3;
//                fr.setPower(throttle + correction);
//                bl.setPower(throttle - correction);
                System.out.printf("dt:%3.3f, err:%3.3f, corr:%3.3f\n", dt, rollError, correction);

                var bytes = new byte[56];
                var bb = ByteBuffer.wrap(bytes);
                bb.put(ori.orientation.asBytes());
                bb.put(rotK.toBytes());
                bb.flip();
                socket.send(bb, addr);
                //System.out.printf("%3.3f,%3.3f,%3.3f%n", barometerReading.altitude(), af.altitude(), af.verticalVelocity());

                lastUpdate = now;
            }
        }
    }

}
