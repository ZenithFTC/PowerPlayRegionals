package tk.devmello.robot.math.controllers.control1D.custom;

public class PIDF {
    public double kp, ki, kd, kf;
    public double sp, pv;
    public double error, lastError, integral, derivative;
    public double output;

    public PIDF(double kp, double ki, double kd, double kf, double sp, double pv) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.sp = sp;
        this.pv = pv;
    }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
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
        integral += error;
        derivative = error - lastError;
        lastError = error;
        output = kp * error + ki * integral + kd * derivative + kf * sp;
    }

    public void reset() {
        error = 0;
        lastError = 0;
        integral = 0;
        derivative = 0;
        output = 0;
    }
}
