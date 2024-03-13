package pose;

import math.KalmanFilter;
import math.Vector3;
import org.apache.commons.math4.legacy.linear.MatrixUtils;
import org.apache.commons.math4.legacy.linear.RealMatrix;

import java.time.Duration;
import java.time.Instant;

import static java.lang.Math.pow;

public class AltitudeFuser {
	private final KalmanFilter altitudeFilter;

	public AltitudeFuser() {

		this.altitudeFilter = new KalmanFilter(MatrixUtils.createRealMatrix(new double[][] {
				{1, 0, 0}, // observation
				{0, 0, 1}}),
				MatrixUtils.createRealMatrix(new double[3][4])); // control
	}

	public void update(double dt, double verticalAccel, double accelVariance, double rawAltitude, double altitudeVariance) {
		var stateTransition = MatrixUtils.createRealMatrix(new double[][] {
				{1, dt, 0.5 * pow(dt, 2)}, // state transition
				{0, 1, dt},
				{0, 0, 1}});

		altitudeFilter.predict(stateTransition, MatrixUtils.createColumnRealMatrix(new double[4]));
		altitudeFilter.correct(MatrixUtils.createColumnRealMatrix(new double[]{rawAltitude, verticalAccel}), MatrixUtils.createRealDiagonalMatrix(new double[] {altitudeVariance, accelVariance}));
	}

	public double altitude() {
		return altitudeFilter.state()[2];
	}

	public double verticalVelocity() {
		return altitudeFilter.state()[1];
	}

	public double verticalAccel() {
		return altitudeFilter.state()[0];
	}

}
