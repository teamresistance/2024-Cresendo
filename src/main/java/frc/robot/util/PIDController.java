// PIDController.java
package frc.robot.util;

public class PIDController {
    private double kP;
    private double kI;
    private double kD;

    private double integral;
    private double previousError;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        reset();
    }

    public double calculate(double setpoint, double current) {
        double error = setpoint - current;
        integral += error;
        double derivative = error - previousError;

        double output = kP * error + kI * integral + kD * derivative;

        previousError = error;

        return output;
    }

    public void reset() {
        integral = 0.0;
        previousError = 0.0;
    }

    public double getKP() {
        return kP;
    }

    public double getKI() {
        return kI;
    }

    public double getKD() {
        return kD;
    }

    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
