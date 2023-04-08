package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.Junction;
import org.firstinspires.ftc.teamcode.vision.CVMaster;

import java.lang.reflect.Field;
import java.util.Collections;
import java.util.HashSet;
import java.util.function.Predicate;

public class BaseOpMode extends CommandOpMode {

    protected MotorEx leftBack, leftFront, rightBack, rightFront; //liftLeft;
    protected DcMotorSimple liftLeft;

    protected SimpleServo  armL, armR, wristServo, clawServo;

    protected ClawSubsystem claw;
    protected DriveSubsystem drive;
    protected LiftSubsystem lift;
    protected ArmSubsystem arm;
    protected WristSubsystem wrist;
    protected SampleMecanumDrive autoDrive;
//    protected WristSubsystem wrist1;
    protected NormalizedColorSensor colorSensor;
    protected ColorSensorSubsystem color;
    protected RevIMU imu;

    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;

    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        autoDrive = new SampleMecanumDrive(hardwareMap);
        initHardware();
        setUp();
        CVMaster cv = new CVMaster(this);
//      call the function to startStreaming
        cv.observeStick();
        drive = new DriveSubsystem(leftBack, leftFront, rightBack, rightFront);
        lift = new LiftSubsystem(liftLeft,  rightFront, gamepadEx1::getRightY );
//        wrist1 = new WristSubsystem(clawServo);
        claw = new ClawSubsystem(clawServo);
        arm = new ArmSubsystem(armL,armR);
        wrist = new WristSubsystem(wristServo);
        color = new ColorSensorSubsystem(colorSensor);
        lift.setJunction(Junction.NONE);
        arm.mid();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Mode", "Done initializing");
        telemetry.update();
    }

    protected void initHardware(){
//        leftBack = new MotorEx(hardwareMap, "leftBack");
//        leftFront = new MotorEx(hardwareMap, "leftFront");
//        rightBack = new MotorEx(hardwareMap, "rightBack");
//        rightFront = new MotorEx(hardwareMap, "rightFront");
        initHardwareMotors(getClass());
        liftLeft  = hardwareMap.get(DcMotorSimple.class, "slideL");

        clawServo = new SimpleServo(hardwareMap, "claw", 0, 360);
        armL = new SimpleServo(hardwareMap, "armL", 0, 360);

        armR = new SimpleServo(hardwareMap, "armR", 0, 360);

        wristServo = new SimpleServo(hardwareMap, "wrist", 0, 180);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        imu = new RevIMU(hardwareMap);
        imu.init();

    }
    protected void setUp(){
        //leftBack.setInverted(true);
        wristServo.setInverted(true);
        clawServo.setInverted(true);
        armR.setInverted(true);
        rightBack.resetEncoder();
        rightFront.resetEncoder();
        leftFront.resetEncoder();
        leftBack.resetEncoder();
    }

    @Override
    public void run() {
        super.run();
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        autoDrive.update();
        tad("leftBack Power", leftBack.motor.getPower());
        tad("leftFront Power", leftFront.motor.getPower());
        tad("rightBack Power", rightBack.motor.getPower());
        tad("rightFront Power", rightFront.motor.getPower());
        tad("liftLeft Power", liftLeft.getPower());
        tad("liftLeft Position", leftBack.getCurrentPosition());
        tad("Heading", imu.getHeading());
        tad("Current Junction", lift.getCurrentJunction());
        tad("output", lift.getOutput());
        tad("current target", lift.getCurrentTarget());
        tad("current junction", lift.getCurrentJunction());
        tad("wrist servo position", wristServo.getPosition());
        tad("armL servo position", armL.getPosition());
        tad("armR servo position", armR.getPosition());
        tad("claw servo position", clawServo.getPosition());
        telemetry.update();

    }

    // gamepad button 1 = gb1
    protected GamepadButton gb1(GamepadKeys.Button button){
        return gamepadEx1.getGamepadButton(button);
    }

    // gamepad button 2 = gb2
    protected GamepadButton gb2(GamepadKeys.Button button){
        return gamepadEx2.getGamepadButton(button);
    }

    protected void tad(String tag, Object data){
        telemetry.addData(tag, data);
    }

    //create a method that all fields in this class and all subclasses of type HardwareDevice (MotorEx) and assigns them to a value from the hardware map the same as the field name.
    //this is a recursive method that will go through all fields in this class and all subclasses of this class and assign them to a value from the hardware map
    /**
     *  If you have a field in the class like this:
     *  {@code private MotorEx slideMotor;}
     *  It will automatically initialize it to {@code new MotorEx(hardwareMap, "slideMotor");}
     */
    @SuppressWarnings("rawtypes")
    private void initHardwareMotors(Class<?> clazz) {
        try {
            Field[] fields = clazz.getDeclaredFields();
            for (Field field : fields) {
                if (field.getType().isAssignableFrom(HardwareDevice.class)) {
                    field.set(this, new MotorEx(hardwareMap, field.getName()));
                }
            }
            if (clazz.getSuperclass() != null) {
                initHardwareMotors(clazz.getSuperclass());
            }
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
    }
    /**
     * If you have a field in the class like this:
     * {@code private SimpleServo claw;}
     * It will automatically initialize it to {@code new SimpleServo(hardwareMap, "claw", 0,255);}
     */
    private void initHardwareServos(Class<?> clazz) {
        try {
            Field[] fields = clazz.getDeclaredFields();
            for (Field field : fields) {
                if (field.getType().isAssignableFrom(SimpleServo.class)) {
                    field.set(this, new SimpleServo(hardwareMap, field.getName(), 0, 255));
                }
            }
            if (clazz.getSuperclass() != null) {
                initHardwareServos(clazz.getSuperclass());
            }
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
    }



}
