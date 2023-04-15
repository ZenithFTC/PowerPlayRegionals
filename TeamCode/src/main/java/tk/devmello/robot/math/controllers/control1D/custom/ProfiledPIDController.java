package tk.devmello.robot.math.controllers.control1D.custom;

public class ProfiledPIDController {
    private PID m_controller;
    private TrapezoidProfile m_profile;

    private FeedForward feedForward = new FeedForward(0, 0, 0);

    private TrapezoidProfile.Constraints m_constraints;

    public ProfiledPIDController(double kP, double kI, double kD, TrapezoidProfile m_profile) {
        m_controller = new PID(kP, kI, kD);
        this.m_profile = m_profile;
    }

    public double calculate(double currentVelocity, double targetVelocity) {
        double acceleration = m_profile.calculate(currentVelocity, targetVelocity);
        double feedforward = feedForward.calculate(currentVelocity);
        double error = targetVelocity - currentVelocity;
        double output = m_profile.calculate(error, acceleration) + feedforward;
        return output;
    }


}
