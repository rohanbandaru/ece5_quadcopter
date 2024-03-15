package drone;

import com.diozero.devices.PwmServo;

public class Motor implements AutoCloseable {
    public final PwmServo servo;
    private final double minDutyCycleMs;

    private boolean armed = false;

    public Motor(int gpio, double minDutyCycleMs) {
        servo = new PwmServo(gpio, 1f, 50);
        this.minDutyCycleMs = minDutyCycleMs;
    }

    public synchronized void arm() throws InterruptedException {
        servo.setPulseWidthMs(1f);
        Thread.sleep(1000);
        servo.setPulseWidthMs(2f);
        Thread.sleep(1000);
        servo.setPulseWidthMs(1f);
        armed = true;
    }

    public synchronized void setPower(double power) {
        if (power < 0 || power > 1) {
            System.out.println("skibidi toilet");
            return;
        }
            //throw new IllegalArgumentException("!(0 <= power <= 1)");

        if (!armed)
            throw new IllegalArgumentException("motor must be armed first");

        servo.setPulseWidthMs((float) (minDutyCycleMs + (2 - minDutyCycleMs) * power));
    }

    @Override
    public void close() {
        servo.setPulseWidthMs(1f);
        servo.close();
    }
}
