package tk.devmello.robot.automodule;

import tk.devmello.robot.util.codeseg.CodeSeg;
import tk.devmello.robot.util.codeseg.ReturnCodeSeg;

public class AutoModule {
    protected ReturnCodeSeg<Boolean> code;
    protected boolean isFinished = false;

    public AutoModule(ReturnCodeSeg<Boolean> code) {
        this.code = code;
    }

    //run the code and check if it's finished
    public void execute() {
        isFinished = code.run();
    }

    //check if the code is finished
    public boolean isFinished() {
        return isFinished;
    }
    //stop the code
    // public void stop() {

}
