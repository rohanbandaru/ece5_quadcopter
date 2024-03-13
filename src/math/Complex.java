package math;

public record Complex(double r, double theta) {
	private static final Complex ZERO = Complex.ofCartesian(0, 0);

	public static Complex zero() {
		return ZERO;
	}

	public static Complex ofCartesian(double a, double b) {
		return new Complex(Math.sqrt(a * a + b * b), Math.atan2(b, a));
	}

	public static Complex ofPolar(double r, double theta) {
		return new Complex(r, theta);
	}

	public double a() {
		return r * Math.cos(theta);
	}

	public double b() {
		return r * Math.sin(theta);
	}

	public Complex scale(double s) {
		return Complex.ofPolar(r * s, theta);
	}

	public Complex plus(Complex b) {
		double real = this.a() + b.a();
		double imag = this.b() + b.b();
		return Complex.ofCartesian(real, imag);
	}

	public Complex minus(Complex b) {
		return plus(b.scale(-1));
	}

	public Complex times(Complex b) {
		return Complex.ofPolar(r * b.r, theta + b.theta);
	}

	public Complex times(double s) {
		return Complex.ofPolar(r * s, theta);
	}

	public Complex conjugate() {
		return Complex.ofPolar(r, -theta);
	}
}
