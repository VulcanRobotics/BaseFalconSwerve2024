package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class BrushIntake extends CommandBase {

    Intake s_Intake; 
    double speed;

    public BrushIntake(Intake intake, double speed) {
        this.s_Intake = intake;
        this.speed = speed;
     }
     @Override
    public void execute() {
        s_Intake.setIntakeSpeed(speed);
    }


    @Override
    public void end(boolean interrupted) {
        
    }

}