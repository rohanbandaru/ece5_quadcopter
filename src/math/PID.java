package math;

public class PID {

    public double P_Gain;
    public double I_Gain;
    public double D_Gain;

    private boolean reset = true;

    private volatile double lastError;
    private double errorSum;

    public PID(double kP, double kI, double kD) {
        P_Gain = kP;
        I_Gain = kI;
        D_Gain = kD;
    }

    public double correction(double dt, double error) {

        double p = error * P_Gain;
        double i = errorSum * I_Gain;
        double d = 0;

        if (reset) {
            if (Double.isNaN(error / dt))
                throw new AssertionError();
            reset = false;
        }else {
            errorSum += error / dt;
            var errorDerivative = (error - lastError) / dt;
            d = D_Gain * errorDerivative;
        }

        lastError = error;

        if (Double.isNaN(p))
            throw new AssertionError();
        if (Double.isNaN(i))
            throw new AssertionError();
        if (Double.isNaN(d))
            throw new AssertionError();

        return p + i + d;
    }

    public void reset() {
        errorSum = 0;
        reset = true;
    }

    public void setConstants(double P, double I, double D) {
        this.P_Gain = P;
        this.I_Gain = I;
        this.D_Gain = D;
    }
}