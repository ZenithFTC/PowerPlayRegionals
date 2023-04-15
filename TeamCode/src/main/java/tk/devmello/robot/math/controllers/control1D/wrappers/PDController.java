package tk.devmello.robot.math.controllers.control1D.wrappers;

public class PDController extends PIDController{
    public PDController(double kp, double kd) {
        super(kp, 0, kd);
    }

    /**
     * The extended constructor.
     */
    public PDController(double kp, double kd, double sp, double pv) {
        super(kp, 0, kd, sp, pv);
    }
}
