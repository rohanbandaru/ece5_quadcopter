package sensors;

import com.diozero.api.I2CDevice;

import java.time.Instant;

import static java.lang.Math.exp;
import static java.lang.Math.pow;
import static java.time.Instant.now;
import static sensors.BMP388.IIR.COEF_0;
import static sensors.BMP388.IIR.COEF_7;
import static sensors.BMP388.PressureOversample.*;
import static sensors.BMP388.Register.*;

public class BMP388 implements AutoCloseable {
	private static final int SLAVE_ADDRESS = 0x77;

	private final I2CDevice delegate;
	private final CalibrationData calibration;

	private double zeroAltitude;

	private PressureOversample pressureOversample;
	private IIR iir;

	private BMP388(I2CDevice delegate) throws InterruptedException {
		this.delegate = delegate;
		this.calibration = new CalibrationData(readData(0x31, 21));

		this.delegate.writeByteData(CMD.address, (byte)0xb6);
//		this.delegate.writeByteData(OSR.address, 0b110011);
		iir = COEF_0;
		configureOversampling(ULTRA_LOW);
//		setFifoConfig(FifoConfig.disabled());

		this.zeroAltitude = read().pressure;
	}

	public static BMP388 withDefaults(int controller) throws InterruptedException {
		var device = new BMP388(new I2CDevice(controller, SLAVE_ADDRESS));

		return device;
	}

	public Reading read() throws InterruptedException {
		this.delegate.writeByteData(PWR_CTRL.address, (byte)0b010011);

		while (!dataReady()) {
			if (Thread.interrupted())
				throw new InterruptedException();
		}

		var data = delegate.readI2CBlockDataByteArray(PRESSURE_DATA_0.address, 6);

		var rawPressure = ((data[2] & 0xff) << 16) | ((data[1] & 0xff) << 8) | (data[0] & 0xff);
		var rawTemperature = ((data[5] & 0xff) << 16) | ((data[4] & 0xff) << 8) | (data[3] & 0xff);

		var temperature = compensatedTemperature(rawTemperature);
		var pressure = compensatedPressure(rawPressure, temperature) - zeroAltitude;

		return new Reading(now(), temperature, pressure, temperatureVariance(), pressureVariance());
	}

	public double temperatureVariance() {
		return pow(0.005, 2);
	}

	public double pressureVariance() {
		return pressureOversample.variance(iir);
	}

	public void configureOversampling(PressureOversample pressureOversample) {
		this.pressureOversample = pressureOversample;
		delegate.writeByteData(OSR.address, (byte) pressureOversample.ordinal());
	}

	public void configureIIR(IIR iir) {
		this.iir = iir;
		delegate.writeByteData(CONFIG.address, (byte) iir.ordinal());
	}

	public void setFifoConfig(FifoConfig config) {
		int config0 = 0;
		int config1 = 0;

		config0 |= config.fifoLength != 0 ? (1 << 0) : 0;
		config0 |= config.stopOnFull ? (1 << 1) : 0;
		config0 |= config.time ? (1 << 2) : 0;
		config0 |= config.pressure ? (1 << 3) : 0;
		config0 |= config.temperature ? (1 << 4) : 0;

		config1 |= config.subsampling;
		config1 |= config.filtered ? (1 << 3) : 0;

		delegate.writeByteData(FIFO_CONFIG_0.address, (byte) config0);
		delegate.writeByteData(FIFO_CONFIG_1.address, (byte) config1);

		delegate.writeByteData(FIFO_LENGTH_0.address, (byte) (config.fifoLength & 0xFF));
		delegate.writeByteData(FIFO_LENGTH_1.address, (byte) ((config.fifoLength >> 8) & 0xFF));
	}

	private double compensatedPressure(int uncomp_press, double temperature) {
		/* Variable to store the compensated pressure */
		double comp_press; /* Temporary variables used for compensation */
		double partial_data1;
		double partial_data2;
		double partial_data3;
		double partial_data4;
		double partial_out1;
		double partial_out2; /* Calibration data */
		partial_data1 = calibration.par_p6() * temperature;
		partial_data2 = calibration.par_p7() * (temperature * temperature);
		partial_data3 = calibration.par_p8() * (temperature * temperature * temperature);
		partial_out1 = calibration.par_p5() + partial_data1 + partial_data2 + partial_data3;
		partial_data1 = calibration.par_p2() * temperature;
		partial_data2 = calibration.par_p3() * (temperature * temperature);
		partial_data3 = calibration.par_p4() * (temperature * temperature * temperature);
		partial_out2 = (double) uncomp_press * (calibration.par_p1() + partial_data1 + partial_data2 + partial_data3);
		partial_data1 = (double) uncomp_press * (double) uncomp_press;
		partial_data2 = calibration.par_p9() + calibration.par_p10() * temperature;
		partial_data3 = partial_data1 * partial_data2;
		partial_data4 = partial_data3 + ((double) uncomp_press * (double) uncomp_press * (double) uncomp_press) * calibration.par_p11();
		comp_press = partial_out1 + partial_out2 + partial_data4;
		return comp_press;
	}

	private double compensatedTemperature(int uncomp_temp) {
		double partial_data1 = (double) (uncomp_temp - calibration.par_t1());
		double partial_data2 = (double) (partial_data1 * calibration.par_t2()); /* Update the compensated temperature in calib structure since this is * needed for pressure calculation */
		return partial_data2 + (partial_data1 * partial_data1) * calibration.par_t3(); /* Returns compensated temperature */
	}

	private int rawPressure() {
		var data = delegate.readI2CBlockDataByteArray(Register.PRESSURE_DATA_0.address, 3);

		return ((data[2] & 0xff) << 16) | ((data[1] & 0xff) << 8) | data[0];
	}

	private int rawTemperature() {
		var data = delegate.readI2CBlockDataByteArray(Register.TEMPERATURE_DATA_0.address, 3);
		return ((data[2] & 0xff) << 16) | ((data[1] & 0xff) << 8) | data[0];
	}

	private byte[] readData(int register, int length) {
		byte[] result = new byte[length];

		for (int i = 0; i < length; i++) {
			result[i] = delegate.readByteData(i + register);
		}

		return result;
	}

	private boolean dataReady() {
		var status = delegate.readByteData(STATUS.address);
		var expected = (1 << 5) | (1 << 6);

		return (status & expected) == expected;
	}

	@Override
	public void close() {
		delegate.close();
	}

	public record FifoConfig(int fifoLength, boolean stopOnFull, boolean time, boolean pressure, boolean temperature,
							 boolean filtered, int subsampling) {
		public static FifoConfig disabled() {
			return new FifoConfig(0, false, false, false, false, false, 0);
		}

		public FifoConfig {
			if (subsampling != 0 && subsampling != 1 && subsampling != 2 && subsampling != 4)
				throw new IllegalArgumentException("Subsampling must be 0, 1, 2, or 4");

			if (fifoLength < 0 || fifoLength > (1 << 15 - 1))
				throw new IllegalArgumentException("Fifo length must be between 0 and 32767");
		}
	}

	public enum PressureOversample {
		ULTRA_LOW,
		LOW,
		STANDARD,
		HIGH,
		ULTRA_HIGH,
		HIGHEST;

		private static final double[][] rmsNoise = {
				{6.6, 3.8, 2.5, 1.7, 1.2, 0.8, 0.6, 0.4},
				{4.3, 2.5, 1.6, 1.1, 0.8, 0.5, 0.4, 0.3},
				{3.2, 1.8, 1.2, 0.8, 0.6, 0.4, 0.3, 0.2},
				{2.3, 1.3, 0.9, 0.6, 0.4, 0.3, 0.2, 0.1},
				{1.6, 0.9, 0.6, 0.4, 0.2, 0.2, 0.1, 0.1},
				{1.2, 0.7, 0.4, 0.3, 0.3, 0.1, 0.1, 0.1}
		};

		public double variance(IIR iir) {
			return pow(rmsNoise[ordinal()][iir.ordinal()], 2);
		}
	}

	public enum IIR {
		COEF_0,
		COEF_1,
		COEF_3,
		COEF_7,
		COEF_15,
		COEF_31,
		COEF_63,
		COEF_127
	}

	public record CalibrationData(byte[] data) {
		public double par_t1() {
			return (((data[0x1] & 0xff) << 8) | (data[0x0] & 0xff)) / pow(2, -8);
		}

		public double par_t2() {
			return (((data[0x3] & 0xff) << 8) | (data[0x2] & 0xff)) / pow(2, 30);
		}

		public double par_t3() {
			return data[0x4] / pow(2, 48);
		}

		public double par_p1() {
			return ((short) ((data[0x6] & 0xff) << 8 | (data[0x5] & 0xff)) - (1 << 14)) / pow(2, 20);
		}

		public double par_p2() {
			return ((short) ((data[0x8] & 0xff) << 8 | (data[0x7] & 0xff)) - (1 << 14)) / pow(2, 29);
		}

		public double par_p3() {
			return data[0x9] / pow(2, 32);
		}

		public double par_p4() {
			return data[0xA] / pow(2, 37);
		}

		public double par_p5() {
			return (((data[0xC] & 0xff) << 8) | (data[0xB] & 0xff)) / pow(2, -3);
		}

		public double par_p6() {
			return (((data[0xE] & 0xff) << 8) | (data[0xD] & 0xff)) / pow(2, 6);
		}

		public double par_p7() {
			return data[0xF] / pow(2, 8);
		}

		public double par_p8() {
			return data[0x10] / pow(2, 15);
		}

		public double par_p9() {
			return ((short) ((data[0x12] & 0xff) << 8 | (data[0x11] & 0xff))) / pow(2, 48		);
		}

		public double par_p10() {
			return data[0x13] / pow(2, 48);
		}

		public double par_p11() {
			return data[0x14] / pow(2, 65);
		}
	}

	enum Register {
		CHIP_ID((byte) 0x00),
		ERR_REG((byte) 0x02),
		STATUS((byte) 0x03),

		PRESSURE_DATA_0((byte) 0x04),
		PRESSURE_DATA_1((byte) 0x05),
		PRESSURE_DATA_2((byte) 0x06),

		TEMPERATURE_DATA_0((byte) 0x07),
		TEMPERATURE_DATA_1((byte) 0x08),
		TEMPERATURE_DATA_2((byte) 0x09),

		SENSOR_TIME_0((byte) 0x0C),
		SENSOR_TIME_1((byte) 0x0D),
		SENSOR_TIME_2((byte) 0x0E),

		EVENT((byte) 0x10),
		INT_STATUS((byte) 0x11),

		FIFO_LENGTH_0((byte) 0x12),
		FIFO_LENGTH_1((byte) 0x13),
		FIFO_DATA((byte) 0x14),

		FIFO_WTM_0((byte) 0x15),
		FIFO_WTM_1((byte) 0x16),

		FIFO_CONFIG_0((byte) 0x17),
		FIFO_CONFIG_1((byte) 0x18),

		INT_CTRL((byte) 0x19),
		PWR_CTRL((byte) 0x1B),
		OSR((byte) 0x1C),
		ODR((byte) 0x1D),
		CONFIG((byte) 0x1F),

		CMD((byte) 0x7E);;
		final byte address;

		Register(byte address) {
			this.address = address;
		}
	}

	public record Reading(Instant time, double temperature, double pressure, double temperatureVariance, double pressureVariance) {
		public double altitude() {
				return (pressure) * 0.083226546738322;
		}

		public double altitudeVariance() {
			return pressureVariance / pow(0.12677457000000025, 2);
		}

		@Override
		public String toString() {
			return "[Temperature = %3.3f C, pressure = %3.3f Pa, altitude = %3.3f m]".formatted(temperature, pressure, altitude());
		}
	}
}
