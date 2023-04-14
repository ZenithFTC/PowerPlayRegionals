package org.firstinspires.ftc.teamcode.command.group;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.arm.Away;
import org.firstinspires.ftc.teamcode.command.arm.Mid;
import org.firstinspires.ftc.teamcode.command.claw.Release;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.wrist.Flip;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;

@Config
public class GrabMid extends SequentialCommandGroup {
    public static int delay = 100;
    public GrabMid(ArmSubsystem arm, ClawSubsystem claw, WristSubsystem wrist) {
        addCommands(
                new Release(claw),
                new ParallelCommandGroup(new UnFlip(wrist),
                        new Mid(arm))
        );
        addRequirements(arm, claw, wrist);
    }
}
