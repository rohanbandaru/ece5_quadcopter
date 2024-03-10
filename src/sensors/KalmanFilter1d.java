package sensors;

public class KalmanFilter1d {
	private double state, variance;

	public KalmanFilter1d(double initialState, double initialVariance) {
		this.state = initialState;
		this.variance = initialVariance;
	}

	public synchronized void predict(double delta, double processVariance) {
		state += delta;
		variance += processVariance;
	}

	public synchronized void correct(double measurement, double measurementVariance) {
		var gain = variance / (variance + measurementVariance);
		var innovation = measurement - state;

		state += gain * innovation;
		variance *= (1 - gain);
	}

	public double state() {
		return state;
	}
}
