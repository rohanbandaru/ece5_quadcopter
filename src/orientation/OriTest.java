package orientation;

import math.Quaternion;
import math.Vector3;

import static java.util.FormatProcessor.FMT;

public class OriTest {

    public void main() {
        test_update();
        test_rotate_between();
    }

    public void test_update() {
        Orientation ori = new Orientation();
        // roll over pi radians between updates
        ori.update(1, Vector3.of(Math.PI, 0, 0), Vector3.of(0, 0, 1), 0);
        printOriEuler(ori.orientation.toEuler());
        ori.update(1, Vector3.of(Math.PI, 0, 0), Vector3.of(0, 0, -1), 0);
        printOriEuler(ori.orientation.toEuler());

        // pitch over pi radians between updates
        ori.update(1, Vector3.of(0, Math.PI, 0), Vector3.of(0, 0, 1), 0);
        printOriEuler(ori.orientation.toEuler());
        ori.update(1, Vector3.of(0, Math.PI, 0), Vector3.of(0, 0, -1), 0);
        printOriEuler(ori.orientation.toEuler());

        // yaw over pi radians between updates
        ori.update(1, Vector3.of(0,0, Math.PI), Vector3.of(0, 0, 1), 0);
        printOriEuler(ori.orientation.toEuler());
        ori.update(1, Vector3.of(0, 0, Math.PI), Vector3.of(0, 0, 1), 0);
        printOriEuler(ori.orientation.toEuler());
    }

    public void test_rotate_between() {
        System.out.println("Angle between opposite vectors:");
        printOriEuler(Quaternion.rotationBetween(Vector3.of(0, 0, -1), Vector3.of(0, 0, 1)).toEuler());
    }

    public void printOriEuler(double[] angles) {
        System.out.println(FMT."Yaw:%.3f\{angles[0]}, Pitch:%.3f\{angles[1]}, Roll:%.3f\{angles[2]}");
    }
}
