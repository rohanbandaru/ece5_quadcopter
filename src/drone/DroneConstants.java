package drone;

/*
Drone Axis:
z  x
|  / --y
           --X--
            //   Front
        ___//___
 |     /       /   |
 X====/       /====X
 |   /_______/     |
        //
Back   //
     --X--
 */

public class DroneConstants {
    public final static double MASS = 2; // total mass of drone in kg
    final static double X_MOI = 0;
    final static double Y_MOI = 0;

    public static final int FRONT_LEFT_GPIO = 24;
    public static final int BACK_RIGHT_GPIO = 13;
    public static final int BACK_LEFT_GPIO = 23;
    public static final int FRONT_RIGHT_GPIO = 12;
}
