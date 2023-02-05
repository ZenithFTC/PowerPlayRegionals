package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.onbot.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.lib.onbot.GamepadWrapper;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.math.TimeManager;

@Config
@TeleOp(name="Austin Test Bot" )
public class TestTele extends LinearOpMode {

    public static double slowSpeedMultiplier = .3;

    protected DcMotorEx motor1;
    protected DcMotorEx motor2;
    protected DcMotorEx motor3;
    protected DcMotorEx[] motors = {motor1, motor2, motor3};

    private GamepadWrapper gamepad1Wrapper;
    private GamepadWrapper gamepad2Wrapper;

    @Override
    public void runOpMode() throws InterruptedException {

        this.gamepad2Wrapper = new GamepadWrapper()
                .setGamepad(gamepad2);

        GamepadEx gamepadLib = new GamepadEx(gamepad2);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        motor1 = (DcMotorEx) hardwareMap.dcMotor.get("1");
        motor2 = (DcMotorEx) hardwareMap.dcMotor.get("2");
        motor3 = (DcMotorEx) hardwareMap.dcMotor.get("3");



       waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            Telemetry dashboardTelemetry = dashboard.getTelemetry();


            double x = gamepad1.x;
            

            telemetry.addData("RRM Ticks", 1);
            dashboardTelemetry.addData("Claw Power", 1);
            dashboardTelemetry.update();
            telemetry.update();
            prevTime = System.currentTimeMillis();
        }


    }



}