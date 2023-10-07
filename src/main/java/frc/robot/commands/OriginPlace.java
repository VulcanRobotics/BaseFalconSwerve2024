package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class OriginPlace extends CommandBase {

    ArmSubsystem armSubsystem;
    boolean cube;


    public OriginPlace(ArmSubsystem arm, boolean cube) {
        this.armSubsystem = arm;
        this.cube = cube;
    }
    
    @Override
    public void initialize(){
        armSubsystem.isCommandRunning = true;
    }
    

    @Override
    public void execute() {
        armSubsystem.Origin(cube);
        
    }

    @Override
    public boolean isFinished(){
        if (armSubsystem.Origin(cube)) {
            armSubsystem.isCommandRunning = false;
        }
        return armSubsystem.Origin(cube);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopMovement();
    }

}
