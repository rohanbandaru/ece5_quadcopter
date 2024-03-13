package sensors;

import math.Vector3;

public class AsyncPoll {

    public Vector3 acceleration; // Gs
    public Vector3 gyroRates; // Rads/s
    public double temperature; // Celsius
    public double altitude; // meters
    public double batteryVoltage; // volts
    /*
    Poll all sensors asynchronously and combine trivial measurements (avg temperature)
    Set values to public variables
     */
    public AsyncPoll(MPU6050 imu, BMP388 baro) {

    }
}
