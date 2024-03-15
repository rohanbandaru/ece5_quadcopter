package drone;

import com.diozero.api.PwmOutputDevice;
import com.diozero.devices.PwmServo;
import com.diozero.devices.motor.PwmMotor;

import java.util.Scanner;

import static drone.DroneConstants.*;

public class MotorTest {
    public static void main(String[] args) throws InterruptedException {
        try (var fr = new Motor(FRONT_RIGHT_GPIO, 1.1)) {
            var scanner = new Scanner(System.in);
//            fr.arm();

            while (!Thread.interrupted()) {
                var value = scanner.nextFloat();
                fr.servo.setPulseWidthMs(value);
                System.out.println("set");
            }
        }
    }
}
