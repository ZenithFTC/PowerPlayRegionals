package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.arm.Mid;
import org.firstinspires.ftc.teamcode.command.claw.Release;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;

public class GrabMidReset extends SequentialCommandGroup {
    public GrabMidReset(ArmSubsystem arm, ClawSubsystem claw, WristSubsystem wrist) {
        addCommands(
                new Release(claw),
                new ParallelCommandGroup(new UnFlip(wrist),
                        new Mid(arm))
        );
        addRequirements(arm, claw, wrist);
    }
}
