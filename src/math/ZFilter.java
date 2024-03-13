package math;

import math.Complex;
import math.FFT;

public class ZFilter {
	private final ImpulseResponse response;
	private final Complex[] a;

	private final double T;

	private int index;

	public ZFilter(ImpulseResponse response, int order, double t) {
		this.response = response;
		this.a = new Complex[order];
		T = t;
	}

	public void filter(double value) {
		a[index] = Complex.ofCartesian(value, 0);
		index = (index + 1) % a.length;
	}

	public double value() {
		var fft = FFT.fft(a);

		for (int i = 0; i < fft.length; i++) {
			double frequency = i * (1 / T) / fft.length;
			Complex s = Complex.ofCartesian(0, frequency * 2 * Math.PI);
			fft[i] = fft[i].times(response.gain(s));
			// z = e^sT
		}
		var ifft = FFT.ifft(fft);
		return ifft[ifft.length - 1].a();
	}

	public interface ImpulseResponse {
		double gain(Complex z);
	}
}
