package tk.devmello.robot.math.controllers.control1D.custom;

public class PD {
    public double kp, kd;
    public double sp, pv;
    public double error, lastError, derivative;
    public double output;

    public PD(double kp, double kd, double sp, double pv) {
        this.kp = kp;
        this.kd = kd;
        this.sp = sp;
        this.pv = pv;
    }

    public void setPD(double kp, double kd) {
        this.kp = kp;
        this.kd = kd;
        sp = 0;
        pv = 0;
    }

    public void setSP(double sp) {
        this.sp = sp;
    }

    public void setPV(double pv) {
        this.pv = pv;
    }

    public void update() {
        error = sp - pv;
        derivative = error - lastError;
        lastError = error;
        output = kp * error + kd * derivative;
    }

    public void reset() {
        error = 0;
        lastError = 0;
        derivative = 0;
        output = 0;
    }
}
