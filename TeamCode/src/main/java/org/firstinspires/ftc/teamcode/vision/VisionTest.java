package org.firstinspires.ftc.teamcode.vision;

import android.annotation.SuppressLint;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.command.arm.Mid;
import org.firstinspires.ftc.teamcode.command.claw.Release;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.opmode.auton.AutonLeftCycle;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.openftc.apriltag.AprilTagDetection;


import java.util.ArrayList;

@Autonomous(name = "Signal Sleeve Test")
@Config
public class VisionTest extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    static int forward = 20;
    static int strafe = 20;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    public static boolean hasRanAuto;

    AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        DcMotorSimple liftMotor = hardwareMap.get(DcMotorSimple.class, "slideL");
        MotorEx rightFront = new MotorEx(hardwareMap, "rightFront");
        SimpleServo clawServo = new SimpleServo(hardwareMap, "claw", 0, 360);
        SimpleServo armL = new SimpleServo(hardwareMap, "armL", 0, 360);
        SimpleServo armR = new SimpleServo(hardwareMap, "armR", 0, 360);
        SimpleServo wristServo = new SimpleServo(hardwareMap, "wrist", 0, 180);
        DcMotorSimple liftLeft = hardwareMap.get(DcMotorSimple.class, "slideL");
        AutonLeftCycle.DRIVE_PHASE currentState = AutonLeftCycle.DRIVE_PHASE.IDLE;
        ArmSubsystem arm = new ArmSubsystem(armL, armR);
        ClawSubsystem claw = new ClawSubsystem(clawServo);
        WristSubsystem wrist = new WristSubsystem(wristServo);
        claw.release();
        arm.mid();
        wrist.home();

        waitForStart();


        initShit();
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        TrajectorySequence parkTrajectory = null;
        if(tagOfInterest == null){
            //default trajectory here if preferred
        }else if(tagOfInterest.id == LEFT){
            parkTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(-forward)
                    .strafeLeft(strafe)
                    //.strafeLeft(40)
                    .build();

            //left trajectory
        }else if(tagOfInterest.id == MIDDLE){
            parkTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(-forward)
                    .build();
            //middle trajectory

        }else{
            parkTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(-forward)
                    .strafeRight(strafe)
                    .build();
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            while (opModeIsActive() && !isStopRequested()) {
                switch (currentState) {
                    case WAIT_FOR_PRELOAD:
                        drive.followTrajectorySequence(parkTrajectory);
                        currentState = AutonLeftCycle.DRIVE_PHASE.SLIDE;
                        break;
                }
            }
        }
    }
    public void initShit(){

    }
    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}