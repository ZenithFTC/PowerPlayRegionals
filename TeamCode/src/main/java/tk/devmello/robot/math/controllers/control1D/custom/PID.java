package tk.devmello.robot.math.controllers.control1D.custom;

public class PID {
    private double kP, kI, kD;
    private double error, lastError, integral, derivative;
    private double setpoint;
    private double output;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getOutput() {
        return output;
    }

    public void update(double input) {
        error = setpoint - input;
        integral += error;
        derivative = error - lastError;
        lastError = error;
        output = kP * error + kI * integral + kD * derivative;
    }

}
