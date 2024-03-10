package orientation;

public record Vector3(double x, double y, double z) {
	public static Vector3 of(double x, double y, double z) {
		return new Vector3(x, y, z);
	}
}
