package tk.devmello.robot.teleop;

public class TeleUtil {
    public static double deadzone(double input, double deadzone){
        if(Math.abs(input) < deadzone){
            return 0;
        } else {
            return input;
        }
    }
    // create and execution function
    public static void execute(boolean condition, Runnable code){
        if(condition){
            code.run();
        }
    }

}
