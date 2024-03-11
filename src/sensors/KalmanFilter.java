package sensors;

import org.apache.commons.math4.legacy.linear.MatrixUtils;
import org.apache.commons.math4.legacy.linear.RealMatrix;

public class KalmanFilter {
	private final RealMatrix observation, dynamics, control;

	private RealMatrix P, x;

	public KalmanFilter(RealMatrix observation, RealMatrix dynamics, RealMatrix control) {
		this.observation = observation;
		this.dynamics = dynamics;
		this.control = control;

		this.P = MatrixUtils.createRealDiagonalMatrix(new double[observation.getRowDimension()]);
		this.x = MatrixUtils.createRealMatrix(new double[observation.getRowDimension()][1]);
	}

	public synchronized void predict(RealMatrix u) {
		x = dynamics.multiply(x).add(control.multiply(u));
		P = dynamics.multiply(P).multiply(dynamics.transpose());
	}

	public synchronized void correct(RealMatrix y, RealMatrix variance) {
		var innovation = y.subtract(observation.multiply(x));
		var innovationCovariance = observation.multiply(P).multiply(observation.transpose()).add(variance);

		var gain = P.multiply(observation.transpose()).multiply(MatrixUtils.inverse(innovationCovariance));
		x = x.add(gain.multiply(innovation));
		P = MatrixUtils.createRealIdentityMatrix(gain.getRowDimension()).subtract(gain.multiply(observation)).multiply(P);
	}

	public double[] state() {
		return x.getColumn(0);
	}
}
