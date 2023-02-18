package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ZenithBot {
    public LiftSubsystem lift;

    private boolean isAuto = false;

    public ZenithBot(HardwareMap hardwareMap, boolean isAuto) {
        this.isAuto = isAuto;
        lift = new LiftSubsystem(hardwareMap, isAuto);
    }

    public ZenithBot(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    public void reset(){
        lift.slideRight.resetEncoder();
    }

    public void read(){ lift.read();
    }

    public void loop(){ lift.loop();}

    public void write(){
        lift.write();
    }

}
