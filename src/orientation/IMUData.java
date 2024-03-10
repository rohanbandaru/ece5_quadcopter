package orientation;

public record IMUData(Quaternion angularVelocity, Vector3 acceleration) {
    // Wrapper class to contain data from the IMU

}
