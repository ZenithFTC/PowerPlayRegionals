package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ZenithBot;

@TeleOp
public class Tele extends CommandOpMode {
    private ZenithBot robot;

    private SampleMecanumDrive drive;

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);

        CommandScheduler.getInstance().reset();
        robot = new ZenithBot(hardwareMap, false);

        //photon later maybe
        //
    }

    @Override
    public void run() {
        robot.read();

        drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

        if (gamepad1.a) {
            CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> robot.lift.newProfile(1000,500,500))
            );
        }


        robot.loop();
        CommandScheduler.getInstance().run();
        robot.write();

        telemetry.addData("liftPos:", robot.lift.getPos());
        telemetry.addData("liftPow:", robot.lift.power);
        telemetry.addData("liftVelocity:", robot.lift.curState.getV());
        telemetry.addData("liftTarget:", robot.lift.targetPosition);

        telemetry.update();

    }

    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
        robot.lift.slideRight.resetEncoder();
        robot.lift.slideLeft.resetEncoder();
        robot.lift.slideRight.set(0);
        robot.lift.slideLeft.set(0);
    }

}
