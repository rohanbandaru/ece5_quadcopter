package sensors;

import com.diozero.api.I2CDevice;
import com.diozero.api.RuntimeIOException;
import math.Vector3;

import java.lang.foreign.MemoryLayout;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;

import static java.lang.Math.PI;
import static java.lang.Math.pow;
import static java.lang.foreign.ValueLayout.JAVA_SHORT_UNALIGNED;
import static java.nio.ByteOrder.BIG_ENDIAN;

public class MPU6050 implements AutoCloseable {
	private static final ValueLayout.OfShort MPU_SHORT = JAVA_SHORT_UNALIGNED.withOrder(BIG_ENDIAN);
	public static final double G = 9.80665;

	private static final double GYRO_VARIANCE_VAL = pow(0.05 * 2 * PI / 360., 2);
	private static final Vector3 GYRO_VARIANCE = Vector3.of(GYRO_VARIANCE_VAL, GYRO_VARIANCE_VAL, GYRO_VARIANCE_VAL);
	private static final double ACCEL_VARIANCE_VAL = pow(0.784, 2);
	private static final Vector3 ACCEL_VARIANCE = Vector3.of(ACCEL_VARIANCE_VAL, ACCEL_VARIANCE_VAL, ACCEL_VARIANCE_VAL);

	//https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
	private static final int DEFAULT_ADDRESS = 0x68;
	private static final int OTHER_ADDRESS = 0x69;

	// default sensitivities
	private static final double GYRO_SENSITIVITY = 131f;
	private static final double ACCEL_SENSITIVITY = 16384f;

	private static final double GYRO_CONVERSION = (1 / 65.5) * PI / 180;
	private static final double ACCEL_CONVERSION = 1. / 4096 * G;

	// temperature values
	private static final double TEMPERATURE_DIVISOR = 340f;
	private static final double TEMPERATURE_OFFSET = 36.53f;

	private static final int CALIBRATION_COUNT = 100;

	private Vector3 accelSpirit = Vector3.zero();
	private Vector3 gyroSpirit = Vector3.zero();

	private final I2CDevice delegate;

	public MPU6050(int controller) {
		delegate = new I2CDevice(controller, DEFAULT_ADDRESS);
		configure();
	}

	private void configure() {
		writeConfiguration("Waking up device",
				Registers.POWER_MANAGEMENT_CONFIG,
				RegisterValues.WAKEUP);
		writeConfiguration("Configuring sample rate",
				Registers.SAMPLE_RATE_DIVISOR,
				RegisterValues.DEFAULT_SAMPLE_DIVISOR);
		writeConfiguration("Configuring gyroscope (500dps full scale)",
				Registers.GYRO_CONFIGURATION, (byte) 0x08);
		writeConfiguration("Configuring accelerometer (+/-8g)",
				Registers.ACCELEROMETER_CONFIGURATION, (byte) 0x10);
		writeConfiguration("Configuring interrupts",
				Registers.ENABLE_INTERRUPTS,
				RegisterValues.INTERRUPT_DISABLED);
		writeConfiguration("Configuring low power operations",
				Registers.STANDBY_MANAGEMENT_CONFIG,
				RegisterValues.STANDBY_DISABLED);
	}

	public void calibrate(Vector3 up) {
		Vector3 gyro = Vector3.zero();
		Vector3 accel = Vector3.zero();

		for (int i = 0; i < CALIBRATION_COUNT; i++) {
			var reading = read();

			gyro = gyro.add(reading.gyro());
			accel = accel.add(reading.accel());
		}

		gyroSpirit = gyroSpirit.add(gyro.scale(1.0 / CALIBRATION_COUNT));
		accelSpirit = accelSpirit.add(accel.scale(1.0 / CALIBRATION_COUNT));

		var gravityAxis = accelSpirit.projectedOnto(up).normalized();
		if (gravityAxis.comp(up) < 0)
			throw new IllegalArgumentException("Vertical axis is not aligned with gravity");

		accelSpirit = accelSpirit.add(gravityAxis.scale(G).scale(-1));
	}

	public Reading read() {
		var data = readArray(Registers.ACCEL_X_REGISTER, MPU_SHORT, 7);

		var accel = processRawAccel(data.getAtIndex(MPU_SHORT, 0), data.getAtIndex(MPU_SHORT, 1), data.getAtIndex(MPU_SHORT, 2));
		var temperature = processRawTemperature(data.getAtIndex(MPU_SHORT, 3));
		var gyro = processRawGyro(data.getAtIndex(MPU_SHORT, 4), data.getAtIndex(MPU_SHORT, 5), data.getAtIndex(MPU_SHORT, 6));

		return new Reading(temperature, 1, gyro, GYRO_VARIANCE, accel, ACCEL_VARIANCE);
// TODO: gyro and accel variance here
//		return new Reading(temperature, 1, gyro, accel);
	}

	public Vector3 readGyro() {
		var data = readArray(Registers.GYRO_X_REGISTER, MPU_SHORT, 3);

		return processRawGyro(data.getAtIndex(MPU_SHORT, 0), data.getAtIndex(MPU_SHORT, 1), data.getAtIndex(MPU_SHORT, 2));
	}

	public Vector3 readAccelerometer() {
		var data = readArray(Registers.ACCEL_X_REGISTER, MPU_SHORT, 3);

		return processRawAccel(data.getAtIndex(MPU_SHORT, 0), data.getAtIndex(MPU_SHORT, 1), data.getAtIndex(MPU_SHORT, 2));
	}

	public double readTemperature() {
		var data = readArray(Registers.TEMPERATURE_REGISTER, MPU_SHORT, 1);
		return processRawTemperature(data.getAtIndex(MPU_SHORT, 0));
	}

	private Vector3 processRawAccel(short x, short y, short z) {
		return Vector3.of(x, y, z).scale(ACCEL_CONVERSION).sub(accelSpirit);
	}

	private double processRawTemperature(short raw) {
		return raw / TEMPERATURE_DIVISOR + TEMPERATURE_OFFSET;
	}

	private Vector3 processRawGyro(short x, short y, short z) {
		return Vector3.of(x, y, z).scale(GYRO_CONVERSION).sub(gyroSpirit);
	}

	/**
	 * Write and verify a device configuration value.
	 *
	 * @param configurationName name of the config for error messages
	 * @param register          the register to write to
	 * @param data              the configuration data
	 */
	private void writeConfiguration(String configurationName, byte register, byte data) {
		delegate.writeByteData(register, data);
		byte actual = delegate.readByteData(register);
		if (actual != data) {
			throw new RuntimeIOException(
					String.format("%s: Tried to write '%02x' to register %02x, but value is '%02x'",
							configurationName, data, register, actual));
		}
	}

	private MemorySegment readArray(int startAddress, MemoryLayout layout, int count) {
		return MemorySegment.ofBuffer(delegate.readI2CBlockDataByteBuffer(startAddress, (int) layout.byteSize() * count));
	}

	@Override
	public void close() {
		delegate.close();
	}

	public record Reading(double temperature, double temperatureVariance, Vector3 gyro, Vector3 gyroVariance, Vector3 accel, Vector3 accelVariance) {

	}

	private interface Registers {
		/**
		 * Set the sample rate (the values are divided by this).
		 */
		byte SAMPLE_RATE_DIVISOR = 0x19;
		/**
		 * Set up the low-pass filter
		 */
		byte LOW_PASS_FILTER = 0x1a;
		/**
		 * Set up the gyroscope
		 */
		byte GYRO_CONFIGURATION = 0x1b;
		/**
		 * Accelerometer configuration
		 */
		byte ACCELEROMETER_CONFIGURATION = 0x1c;
		/**
		 * FIFO (not used)
		 */
		byte FIFO_CONFIGURATION = 0x23;
		/**
		 * Enable interrupts (disabling)
		 */
		byte ENABLE_INTERRUPTS = 0x38;
		/**
		 * Register for high bits of a 2's complement 16-bit value. The low bits
		 * are assumed to be in the next register (this + 1).
		 * <p>
		 * This is the most recent reading from the accelerometer's X axis.
		 */
		byte ACCEL_X_REGISTER = 0x3b;
		/**
		 * Register for high bits of a 2's complement 16-bit value. The low bits
		 * are assumed to be in the next register (this + 1).
		 * <p>
		 * This is the most recent reading from the accelerometer's Y axis.
		 */
		byte ACCEL_Y_REGISTER = 0x3d;
		/**
		 * Register for high bits of a 2's complement 16-bit value. The low bits
		 * are assumed to be in the next register (this + 1).
		 * <p>
		 * This is the most recent reading from the accelerometer's Z axis.
		 */
		byte ACCEL_Z_REGISTER = 0x3f;
		/**
		 * Register for high bits of a 2's complement 16-bit value. The low bits
		 * are assumed to be in the next register (this + 1).
		 * <p>
		 * This is the most recent reading from the temperature sensor.
		 */
		byte TEMPERATURE_REGISTER = 0x41;
		/**
		 * Register for high bits of a 2's complement 16-bit value. The low bits
		 * are assumed to be in the next register (this + 1).
		 * <p>
		 * This is the most recent reading from the gyro's X axis.
		 */
		byte GYRO_X_REGISTER = 0x43;
		/**
		 * Register for high bits of a 2's complement 16-bit value. The low bits
		 * are assumed to be in the next register (this + 1).
		 * <p>
		 * This is the most recent reading from the gyro's Y axis.
		 */
		byte GYRO_Y_REGISTER = 0x45;
		/**
		 * Register for high bits of a 2's complement 16-bit value. The low bits
		 * are assumed to be in the next register (this + 1).
		 * <p>
		 * This is the most recent reading from the gyro's Z axis.
		 */
		byte GYRO_Z_REGISTER = 0x47;
		/**
		 * Resets the various signals.
		 */
		byte SIGNAL_PATH_RESET = 0x68;
		/**
		 * I2C management
		 */
		byte I2C_CONFIG = 0x6a;
		/**
		 * Basic power management
		 */
		byte POWER_MANAGEMENT_CONFIG = 0x6b;
		/**
		 * The I2C address (6-bits)
		 */
		byte WHO_AM_I = 0x75;
		/**
		 * Standby mode management.
		 */
		byte STANDBY_MANAGEMENT_CONFIG = 0x6c;
	}

	private interface RegisterValues {
		/**
		 * Just wakes the device up, because it sets the sleep bit to 0. Also sets
		 * the clock source to internal.
		 */
		byte WAKEUP = 0x0;
		/**
		 * Sets the full scale range of the gyroscopes to ± 2000 °/s
		 */
		byte DEFAULT_GYRO_CONFIGURATION = 0x18;
		/**
		 * Sets the sample rate divider for the gyroscopes and accelerometers. This
		 * means<br> acc-rate = 1kHz / 1+ sample-rate<br> and <br>gyro-rate = 8kHz /
		 * 1+ sample-rate. <br> <br> The concrete value 0 leaves the sample rate on
		 * default, which means 1kHz for acc-rate and 8kHz for gyr-rate.
		 */
		byte DEFAULT_SAMPLE_DIVISOR = 0x0;
		/**
		 * Setting the digital low pass filter to <br>
		 * Acc Bandwidth (Hz) = 184 <br>
		 * Acc Delay (ms) = 2.0 <br>
		 * Gyro Bandwidth (Hz) = 188 <br>
		 * Gyro Delay (ms) = 1.9 <br>
		 * Fs (kHz) = 1
		 */
		byte LOW_PASS_CONFIG = 0x1;
		/**
		 * Setting accelerometer sensitivity to ± 2g
		 */
		byte DEFAULT_ACCEL_CONFIGURATION = 0x0;
		/**
		 * Disabling FIFO buffer
		 */
		byte FIFO_DISABLED = 0x0;
		/**
		 * Disabling interrupts
		 */
		byte INTERRUPT_DISABLED = 0x0;
		/**
		 * Disabling standby modes
		 */
		byte STANDBY_DISABLED = 0x0;
	}
}