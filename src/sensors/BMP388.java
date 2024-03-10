package sensors;

public class BMP388 implements AutoCloseable {

	private enum Register {
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
		;
		final byte address;

		Register(byte address) {
			this.address = address;
		}
	}
}
