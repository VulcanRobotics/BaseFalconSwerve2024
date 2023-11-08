package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class LinearGoToPosition extends CommandBase {

    private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(2, 3);
    private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(2, 3);
    private static final TrapezoidProfile.Constraints rotConstraints = new TrapezoidProfile.Constraints(6, 7);

    private final ProfiledPIDController xController = new ProfiledPIDController(.01, 0, 0, xConstraints);//5.5
    private final ProfiledPIDController yController = new ProfiledPIDController(.01, 0, 0, yConstraints);//4
    private final ProfiledPIDController omegaController = new ProfiledPIDController(.01, 0, 0, rotConstraints);//6

    Swerve swerveDriveSubsystem;
    Pose2d desiredPose2d;
    
    public LinearGoToPosition(Swerve swerve, Pose2d desiredPose)  {
        swerveDriveSubsystem = swerve;
        desiredPose2d = desiredPose;

        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        omegaController.setTolerance(1.0*180.0/3.1415); //1 degree in radians
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize(){
        var robotPose = swerveDriveSubsystem.getPose();
        var robotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                swerveDriveSubsystem.getSpeeds(), swerveDriveSubsystem.getRotation());

        omegaController.reset(robotPose.getRotation().getRadians(), robotVelocity.omegaRadiansPerSecond);
        xController.reset(robotPose.getX(), robotVelocity.vxMetersPerSecond);
        yController.reset(robotPose.getY(), robotVelocity.vyMetersPerSecond);
    }

    @Override
    public void execute() {
        var robotPose = swerveDriveSubsystem.getPose();
        var targetPose = desiredPose2d;

        // Update controllers
        xController.setGoal(targetPose.getX());
        yController.setGoal(targetPose.getY());
        omegaController.setGoal(targetPose.getRotation().getRadians());

        var xSpeed = xController.calculate(robotPose.getX()) + xController.getSetpoint().velocity;
        var ySpeed = yController.calculate(robotPose.getY()) + yController.getSetpoint().velocity;
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians())
                + omegaController.getSetpoint().velocity;

        if (xController.atGoal()) xSpeed = 0;
        if (yController.atGoal()) ySpeed = 0;
        if (omegaController.atGoal()) omegaSpeed = 0;

        swerveDriveSubsystem.setModuleStates(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed), true);
    }

    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}

