package tk.devmello.robot.hardware;

public abstract class RobotPart {

    public void instantiate() {
        //do nothing
    }

    public abstract void init();

    public abstract void update();

    public void reset(){};

    public abstract void stop();

    public void kill() {
        stop();
    }

}
