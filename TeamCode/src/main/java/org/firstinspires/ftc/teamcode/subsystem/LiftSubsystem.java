package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class LiftSubsystem extends SubsystemBase {
    public final MotorEx slideRight, slideLeft;
    public final Servo v4bLeft, v4bRight, claw, clawPitch, turret;

    private MotionProfile profile;
    public MotionState curState;
    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private final PIDController controller;
    private final VoltageSensor voltageSensor;

    private double voltage;
    private int liftPosition;

    //TODO: TUNE PID
    private static final double P = 0.0000;
    private static final double I = 0;
    private static final double D = 0;
    private static final double F = 0.0000;

    //TODO: FIND SERVO POSITIONS

    public static double claw_pos_open = 0.65;
    public static double claw_pos_closed = 0.82;

    public static double v4b_extended = 0.45;
    public static double v4b_retracted = 0.05;

    public double power = 0;
    public double targetPosition = 0;

    private boolean moving = false;

    private PitchState pitchState;
    private ClawState clawState;
    private V4bState v4bState;
    private TurretState turretState;
    private LiftState liftState;

    public enum LiftState {
        HIGH,
        MIDDLE,
        LOW,
        RETRACTED
    }

    public enum V4bState {
        INTAKE,
        DEPOSIT,
        MANUAL
    }

    public enum TurretState {
        INTAKE,
        MANUAL,
        DEPOSIT
    }

    public enum ClawState {
        OPEN,
        CLOSED,
        MANUAL
    }

    public enum PitchState {
        INTAKE,
        DEPOSIT,
        MANUAL
    }

    public LiftSubsystem(HardwareMap hardwareMap, boolean isAuto) {
        this.slideRight = new MotorEx(hardwareMap, "rightSlide");
        this.slideLeft = new MotorEx(hardwareMap, "leftSlide");
        this.claw = hardwareMap.get(Servo.class, "claw");
        this.turret = hardwareMap.get(Servo.class, "turret");
        this.clawPitch = hardwareMap.get(Servo.class, "pitch");
        this.v4bLeft = hardwareMap.get(Servo.class, "leftArm");
        this.v4bRight = hardwareMap.get(Servo.class, "rightArm");

        this.timer = new ElapsedTime();
        timer.reset();

        //TODO: UPDATE POSITIONS IF AUTO

        //TODO: UPDATE SLIDE POSITIONS

        //TODO: SET REVERSE MOTORS

        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), 15, 10);
        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        this.controller = new PIDController(P, I, D);
        controller.setPID(P, I, D);

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();
    }

    public void loop() {
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        curState = profile.get(timer.time());
        if (curState.getV() != 0) {
            targetPosition = curState.getX();
            moving = false;
        } else {
            moving = true;
        }

        power = (controller.calculate(liftPosition, targetPosition) + F) / voltage * 12;
    }

    public void update(LiftState state) {
        switch(state) {
            case RETRACTED:
                //TODO: TUNE
                profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(),0), new MotionState(0,0), 300, 750);
                resetTimer();
                break;
            case LOW:
                profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(),0), new MotionState(50,0), 300, 750);
                resetTimer();
                break;
            case MIDDLE:
                profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(),0), new MotionState(100,0), 300, 750);
                resetTimer();
                break;
            case HIGH:
                profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(),0), new MotionState(150,0), 300, 750);
                resetTimer();
                break;
        }

        liftState = state;

    }

    public void update(TurretState state) {
        switch(state) {
            case INTAKE:
                //TODO: TUNE
                turret.setPosition(0.4);
                break;
            case DEPOSIT:
                //TODO: TUNE
                turret.setPosition(1);
                break;
        }

        turretState = state;
    }

    public void update(PitchState state) {
        switch(state) {
            case INTAKE:
                //TODO: TUNE
                clawPitch.setPosition(0.4);
                break;
            case DEPOSIT:
                //TODO: TUNE
                clawPitch.setPosition(1);
                break;
        }

        pitchState = state;
    }

    public void update(ClawState state) {
        switch(state) {
            case OPEN:
                claw.setPosition(claw_pos_open);
                break;
            case CLOSED:
                claw.setPosition(claw_pos_closed);
                break;
        }

        clawState = state;
    }

    public void update(V4bState state) {
        switch(state) {
            case INTAKE:
                setV4B(v4b_extended);
                break;
            case DEPOSIT:
                setV4B(v4b_retracted);
                break;
        }

        v4bState = state;

    }

    public void read() {
        liftPosition = slideRight.encoder.getPosition();
    }

    public void write() {
        slideRight.set(power);
        slideLeft.set(power);
    }

    public void setV4B(double pos) {
        v4bLeft.setPosition(pos);
        v4bRight.setPosition(1-pos);
    }

    public int getPos() {
        return (int) liftPosition;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void newProfile(double targetPos, double max_v, double  max_a) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new com.acmerobotics.roadrunner.profile.MotionState(getPos(), 0), new com.acmerobotics.roadrunner.profile.MotionState(targetPos, 0), max_v, max_a);
        targetPosition = targetPos;
        resetTimer();
    }

}
