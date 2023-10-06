package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class JoystickMovement extends CommandBase {

    ArmSubsystem armSubsystem;
    private final Joystick operator;

    public JoystickMovement(ArmSubsystem arm, Joystick joystick) {
        armSubsystem = arm;
        addRequirements(armSubsystem);
        operator = joystick;

    }
    
    private boolean arePositionsSet() {
        if (operator.getRawButton(2) || operator.getRawButton(3) || operator.getRawButton(4) || operator.getRawButton(5) || operator.getRawButton(6)) {
            return true;
        }  else {
            return false;
        }
    }

    @Override
    public void initialize(){
        armSubsystem.isCommandRunning = true;
    }
    

    @Override
    public void execute() {
        
        if (!arePositionsSet()) {
            armSubsystem.joystickMovement(operator.getX(), -operator.getY(), operator.getZ());
        }

        SmartDashboard.putNumber("X Arm", operator.getX());
        SmartDashboard.putNumber("Y Arm", operator.getY());
        SmartDashboard.putNumber("Z Arm", operator.getZ());
    }


}
