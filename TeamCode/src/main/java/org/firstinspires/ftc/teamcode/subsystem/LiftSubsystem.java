package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.util.Junction;

import java.util.function.DoubleSupplier;

@Config
public class LiftSubsystem extends SubsystemBase {
    private final MotorEx  encoder;
    private final DcMotorSimple liftL;

    private Junction currentGoal;

    // tune
    public static int none = 50;
    public static int ground = 150;
    public static int low = 800; // 837
    public static int medium = 1650;
    public static int high = 2350;

    public static double kP = 0.00;
    public static double kI = 0;
    public static double kD = 0.0005;
    public static double kG = 0.041;
    public static double maxVelocity = 30000;
    public static double maxAcceleration = 30000;
    private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

    public static double power = 1;
    public static int threshold = 20;

    public static double slowFactor = 1.5;

    private final DoubleSupplier doubleSupplier;

    private int currentTarget;

    private double output;

    public LiftSubsystem(DcMotorSimple liftL,  MotorEx encoder, DoubleSupplier doubleSupplier) {
        this.liftL = liftL;
        this.encoder = encoder;
        this.doubleSupplier = doubleSupplier;
    }

    public void update(){
        liftL.setPower(power);
    }

    public void setJunction(Junction junction){
        currentGoal = junction;
        switch (junction) {
            case NONE:
                currentTarget = none;
                controller.setGoal(none);
                break;
            case GROUND:
                currentTarget = ground;
                controller.setGoal(ground);
                break;
            case LOW:
                currentTarget = low;
                controller.setGoal(low);
                break;
            case MEDIUM:
                currentTarget = medium;
                controller.setGoal(medium);
                break;
            case HIGH:
                currentTarget = high;
                controller.setGoal(high);
                break;
        }
    }

    public void setHigh() {
        setJunction(Junction.HIGH);
    }
    public void setNone() {
        setJunction(Junction.NONE);
    }
    public void setLow() {
        setJunction(Junction.LOW);
    }
    public void setMedium() {
        setJunction(Junction.MEDIUM);
    }

    public boolean atTarget(){
        switch(currentGoal){
            case NONE:
                return encoder.getCurrentPosition()<none+ threshold && encoder.getCurrentPosition()>none- threshold;
            case GROUND:
                return encoder.getCurrentPosition()<ground+ threshold && encoder.getCurrentPosition()>ground- threshold;
            case LOW:
                return encoder.getCurrentPosition()<low+ threshold && encoder.getCurrentPosition()>low- threshold;
            case MEDIUM:
                return encoder.getCurrentPosition()<medium+ threshold && encoder.getCurrentPosition()>medium- threshold;
            case HIGH:
                return encoder.getCurrentPosition()<high+ threshold && encoder.getCurrentPosition()>high- threshold;
        }
        return false;
    }

    public Junction getCurrentJunction() {
        return currentGoal;
    }

    public double getOutput() {
        return output;
    }

    public int getCurrentTarget() {
        return currentTarget;
    }

    public boolean isHigh() {
        return currentGoal == Junction.HIGH;
    }

    public boolean isMedium() {
        return currentGoal == Junction.MEDIUM;
    }
    @Override
    public void periodic() {
        if(doubleSupplier.getAsDouble() != 0) {
            liftL.setPower(doubleSupplier.getAsDouble()/slowFactor);
            controller.setGoal(encoder.getCurrentPosition());
        } else {
            output = controller.calculate(encoder.getCurrentPosition()) + kG;
            liftL.setPower(output);
        }

    }

}