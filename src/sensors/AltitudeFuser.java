package sensors;

import java.time.Duration;
import java.time.Instant;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.StructuredTaskScope;

import static java.lang.Math.pow;

public class AltitudeFuser implements AutoCloseable {
	private final BMP388 barometer;
	private final MPU6050 imu;

	private final KalmanFilter1d vertVelocity;
	private final KalmanFilter1d altitude;
	private volatile double temperature;

	private final Thread thread;

	public AltitudeFuser(BMP388 barometer, MPU6050 imu) {
		this.barometer = barometer;
		this.imu = imu;

		this.vertVelocity = new KalmanFilter1d(0, 1000);
		this.altitude = new KalmanFilter1d(0, 1000);
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

				temperature = weightedTemperature(reading);

				var dAltitude = vertVelocity.state() * dt;
				var varianceDAltitude = vertVelocity.variance() * pow(dt, 2);

				altitude.predict(dAltitude, varianceDAltitude);
				altitude.correct(reading.barometer.altitude(), reading.barometer.altitudeVariance());

				var dVertVelocity = accel.comp(gravity) * dt;
				var varianceDVertVelocity = accelVariance.dot(gravity.hadamard(gravity)) * pow(dt, 2) / pow(gravity.norm(), 2); // assuming Var(gravity) = 0
				vertVelocity.predict(dVertVelocity, varianceDVertVelocity);
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
