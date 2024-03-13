package sensors;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.StructuredTaskScope;

public class tmp {
    /*

    private double weightedTemperature(DualReading reading) {
        double barVariance = reading.barometer.temperatureVariance();
        double imuVariance = reading.imu.temperatureVariance();

        double barWeight = imuVariance / (barVariance + imuVariance);
        double imuWeight = barVariance / (barVariance + imuVariance);

        return reading.barometer.temperature() * barWeight + reading.imu.temperature() * imuWeight;
    }

    private DualReading readBoth() throws InterruptedException {
        try (var scope = new StructuredTaskScope.ShutdownOnFailure()) {
            var barometerTask = scope.fork(barometer::read);
            var imuTask = scope.fork(imu::read);

            scope.join();
            scope.throwIfFailed();

            return new DualReading(barometerTask.get(), imuTask.get());
        } catch (ExecutionException e) {
            throw new RuntimeException(e);
        }
    }

    private record DualReading(BMP388.Reading barometer, MPU6050.Reading imu) {
    }

    @Override
    public void close() throws InterruptedException {
        thread.interrupt();
        thread.join();
    }
     */
}
