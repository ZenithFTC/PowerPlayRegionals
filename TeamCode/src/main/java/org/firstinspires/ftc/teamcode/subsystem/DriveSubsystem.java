package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.util.DriveMode;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;
import java.util.stream.Collectors;

@Config
public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    public static double antiTipFactor;
    public static double defaultSpeed = 1;
    public static double mediumSpeed = 3;
    public static double highSpeed = 5;
    public static double slowModeFactor = 3;
    public static double slowRotScale = .75;
    public MotorEx leftBack, leftFront, rightBack, rightFront;
    List<MotorEx> motors = new ArrayList<>();
    Predicate<MotorEx> velocityPredicate = motorEx -> motorEx.getVelocity() > 15;
    public DriveSubsystem(MotorEx leftBack, MotorEx leftFront, MotorEx rightBack, MotorEx rightFront){
        this.leftBack = leftBack;
        this.leftFront = leftFront;
        this.rightBack = rightBack;
        this.rightFront = rightFront;
        motors.add(this.leftBack);
        motors.add(this.leftFront);
        motors.add(this.rightBack);
        motors.add(this.rightFront);
        antiTipFactor = defaultSpeed;
        drive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);
    }


    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle){
        drive.driveFieldCentric(-strafeSpeed, -forwardSpeed, -turnSpeed, gyroAngle);
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed){
        drive.driveRobotCentric((-strafeSpeed), -forwardSpeed, -turnSpeed*slowRotScale);
    }

    public void driveRobotCentricSlowMode(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveRobotCentric(-strafeSpeed / antiTipFactor,
                -forwardSpeed / antiTipFactor,
                (-turnSpeed*slowRotScale) / antiTipFactor);
    }
    public void checkCollision() {
        List<MotorEx> velocity = motors.stream().filter(velocityPredicate).collect(Collectors.toList());
        if (velocity.size() > 0){drive.stop();}
    }

    public void setAntiTipFactor(DriveMode mode) {
        switch (mode) {
            case HIGHSPEED:
                antiTipFactor = highSpeed;
                break;
            case MEDIUMSPEED:
                antiTipFactor = mediumSpeed;
                break;
            case DEFAULTSPEED:
                antiTipFactor = defaultSpeed;
                break;
        }
    }

}
