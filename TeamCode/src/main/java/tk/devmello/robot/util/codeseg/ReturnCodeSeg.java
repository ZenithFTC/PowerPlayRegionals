package tk.devmello.robot.util.codeseg;

@FunctionalInterface
public interface ReturnCodeSeg<R> {
    R run();
}