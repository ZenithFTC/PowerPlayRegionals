package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.subsystem.constants.ArmConstants;

import java.util.function.DoubleSupplier;

@Config
public class ArmSubsystem extends SubsystemBase {

    private final ServoEx left;
    private final ServoEx right;
    String mode="";
    private final DoubleSupplier doubleSupplier;

    public ArmSubsystem(ServoEx left, ServoEx right, DoubleSupplier doubleSupplier){
        this.left = left;
        this.right = right;
        this.doubleSupplier = doubleSupplier;
    }

    //write the function to rotate the servos between 0 and 1
    public void home(){
        left.setPosition(ArmConstants.startPosition);
        right.setPosition(ArmConstants.startPosition);
        mode="home";
    }

    public void away(){
        left.setPosition(ArmConstants.endPosition);
        right.setPosition(ArmConstants.endPosition);
        mode="away";
    }

    public Command runHomeCommand() {
        return new InstantCommand(this::home,this);
    }

    public Command runAwayCommand() {
        return new InstantCommand(this::away,this);
    }

    public String getMode() {
        return mode;
    }
}
