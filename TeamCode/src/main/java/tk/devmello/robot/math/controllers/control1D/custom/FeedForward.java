package tk.devmello.robot.math.controllers.control1D.custom;

public class FeedForward {
    private double kF;
    private double kS;
    private double kV;

    public FeedForward(double kF, double kS, double kV) {
        this.kF = kF;
        this.kS = kS;
        this.kV = kV;
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

    public double getkS() {
        return kS;
    }

    public void setkS(double kS) {
        this.kS = kS;
    }

    public double getkV() {
        return kV;
    }

    public void setkV(double kV) {
        this.kV = kV;
    }

    public double calculate(double velocity) {
        return kF + kS * Math.signum(velocity) + kV * velocity;
    }
}
