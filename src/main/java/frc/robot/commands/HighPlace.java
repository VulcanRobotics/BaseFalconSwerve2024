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



public class HighPlace extends CommandBase {

    ArmSubsystem armSubsystem;

    public HighPlace(ArmSubsystem arm) {
        armSubsystem = arm;
    }
    
    @Override
    public void initialize(){
        armSubsystem.isCommandRunning = true;
    }

    @Override
    public void execute() {
        armSubsystem.highPlace();
    }

    @Override
    public boolean isFinished(){
        

        if (armSubsystem.highPlace()){
            armSubsystem.isCommandRunning = false;
            System.out.println("GGGGGGGGGGGGGGGGGG");
        }

        return armSubsystem.highPlace();
    }

}
