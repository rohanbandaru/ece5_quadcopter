package sensors;

import com.diozero.api.I2CDevice;
import com.diozero.api.RuntimeIOException;
import orientation.IMUData;
import orientation.Quaternion;
import orientation.Vector3;

import java.util.HashMap;
import java.util.Map;

public class MPU6050 implements AutoCloseable {
    //https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
    private static final int DEFAULT_ADDRESS = 0x68;
    private static final int OTHER_ADDRESS = 0x69;

    // default sensitivities
    private static final float GYRO_SENSITIVITY = 131f;
    private static final float ACCEL_SENSITIVITY = 16384f;

    // temperature values
    private static final float TEMPERATURE_DIVISOR = 340f;
    private static final float TEMPERATURE_OFFSET = 36.53f;

    private float ACCEL_X_SPIRIT = 0.0F;
    private float ACCEL_Y_SPIRIT = 0.0F;
    private float ACCEL_Z_SPIRIT = 0.0F;

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
                Registers.GYRO_CONFIGURATION, (byte)0x08);
        writeConfiguration("Configuring accelerometer (+/-8g)",
                Registers.ACCELEROMETER_CONFIGURATION, (byte)0x10);
        writeConfiguration("Configuring interrupts",
                Registers.ENABLE_INTERRUPTS,
                RegisterValues.INTERRUPT_DISABLED);
        writeConfiguration("Configuring low power operations",
                Registers.STANDBY_MANAGEMENT_CONFIG,
                RegisterValues.STANDBY_DISABLED);
    }

    public Map<Integer, Byte> dumpRegisters() {
        Map<Integer, Byte> values = new HashMap<>();
        for (int i = 1; i <= 120; i++) {
            byte registerData = delegate.readByteData(i);
            values.put(i, registerData);
        }
        return values;
    }

    public String getImuName() {
        return "MPU-6050";
    }

    public IMUData getImuData() {
        return new IMUData(getGyroData(), getAccelerometerData());
    }

    private Quaternion getGyroData() {
        return Quaternion.ofEuler(
                readRegister(Registers.GYRO_X_REGISTER) / GYRO_SENSITIVITY,
                readRegister(Registers.GYRO_Y_REGISTER) / GYRO_SENSITIVITY,
                readRegister(Registers.GYRO_Z_REGISTER) / GYRO_SENSITIVITY);
    }

    private Vector3 getAccelerometerData() {
        return Vector3.of(
                readRegister(Registers.ACCEL_X_REGISTER) / ACCEL_SENSITIVITY,
                readRegister(Registers.ACCEL_Y_REGISTER) / ACCEL_SENSITIVITY,
                readRegister(Registers.ACCEL_Z_REGISTER) / ACCEL_SENSITIVITY);
    }

    public float getTemperature() {
        return (readRegister(Registers.TEMPERATURE_REGISTER) / TEMPERATURE_DIVISOR) + TEMPERATURE_OFFSET;
    }

    private byte getI2CAddress() {
        return delegate.readByteData(Registers.WHO_AM_I);
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

    /**
     * Read a "double byte" from this register and the next one.
     *
     * @param register the register to read from
     * @return the 2-byte integer value
     */
    private int readRegister(int register) {
        byte high = delegate.readByteData(register);
        byte low = delegate.readByteData(register + 1);
		return (high << 8) + low;
    }

    @Override
    public void close() {
        delegate.close();
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