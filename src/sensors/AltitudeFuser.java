package sensors;

import math.KalmanFilter;
import org.apache.commons.math4.legacy.linear.MatrixUtils;

import java.time.Duration;
import java.time.Instant;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.StructuredTaskScope;

import static java.lang.Math.pow;

public class AltitudeFuser implements AutoCloseable {
	private static final double TIME_STEP = 0.05;

	private final BMP388 barometer;
	private final MPU6050 imu;

	private final KalmanFilter altitude;
	private volatile double temperature;

	private final Thread thread;

	public AltitudeFuser(BMP388 barometer, MPU6050 imu) {
		this.barometer = barometer;
		this.imu = imu;

		this.altitude = new KalmanFilter(MatrixUtils.createRealMatrix(new double[][] {{1, 0, 0}, {0, 0, 1}}), MatrixUtils.createRealMatrix(new double[][] {
				{1, TIME_STEP, 0.5 * pow(TIME_STEP, 2)},
				{0, 1, TIME_STEP},
				{0, 0, 1}
		}), MatrixUtils.createRealMatrix(new double[3][4]));
		this.thread = Thread.startVirtualThread(this::updateLoop);
	}

	private void updateLoop() {
		Instant lastUpdate = Instant.now();

		try {
			while (!Thread.interrupted()) {
				var reading = readBoth();
				var now = Instant.now();
				var dt = Duration.between(lastUpdate, now).toNanos() / 1.e9;

				var accel = reading.imu.accel();
				var accelVariance = reading.imu.accelVariance();

				var gravity = reading.imu().gravityDirection();
				var accelUpVar = accelVariance.dot(gravity.hadamard(gravity)) * pow(dt, 2) / pow(gravity.norm(), 2); // assuming Var(gravity) = 0

				var upAccel = accel.comp(gravity);

				temperature = weightedTemperature(reading);

				altitude.predict(MatrixUtils.createColumnRealMatrix(new double[4]));
				altitude.correct(MatrixUtils.createColumnRealMatrix(new double[]{reading.barometer.altitude(), upAccel}), MatrixUtils.createRealDiagonalMatrix(new double[] {reading.barometer.altitudeVariance(), accelUpVar}));
			}
		} catch (InterruptedException e) {
			// ignore
		}
	}

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
}
