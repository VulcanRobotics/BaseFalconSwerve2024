package frc.robot.commands;

import java.util.function.Supplier;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToPosition extends CommandBase {
    
    private final double AnglePIDValues[] = {4.0, 0.0, 0.2};
    private final double DrivePIDValues[] = {2.0, 0.0, 0.1};

    private PIDController xController = new PIDController(DrivePIDValues[0], DrivePIDValues[1], DrivePIDValues[2]);
    private PIDController yController = new PIDController(DrivePIDValues[0], DrivePIDValues[1], DrivePIDValues[2]);
    private ProfiledPIDController mAngleController = new ProfiledPIDController(
        AnglePIDValues[0], AnglePIDValues[1], AnglePIDValues[2],
         new Constraints(Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity));

    private final Swerve swerveDriveSubsystem;
    private Pose2d targetPoseSupplier;

    public DriveToPosition(Swerve swerveDriveSubsystem, Pose2d targetPoseSupplier) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;
        mAngleController.setTolerance(Math.toRadians(0.25));
        mAngleController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerveDriveSubsystem);
    }

    private boolean atX, atY, atTheta;

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();

        xController.setTolerance(Units.inchesToMeters(2.0));
        yController.setTolerance(Units.inchesToMeters(0.5));

        atX = atY = atTheta = false;
    }
    @Override

    public void execute() {
        Pose2d robotPose = swerveDriveSubsystem.getPose2();
        Pose2d targetPose = targetPoseSupplier;

        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        mAngleController.setGoal(targetPose.getRotation().getRadians());

        // Drive to the target
        var xSpeed = xController.calculate(robotPose.getX());
        var ySpeed = yController.calculate(robotPose.getY());

        var thetaSpeed = mAngleController.calculate(robotPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        if (xController.atSetpoint() || atX) {
            xSpeed = 0;
            // atX = true;

        }

        if (yController.atSetpoint() || atY) {
            ySpeed = 0;

        }

        if (mAngleController.atSetpoint() || atTheta) {
            thetaSpeed = 0;

        }

        swerveDriveSubsystem.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed, true, false);
    }
        
    @Override
    public boolean isFinished() {

        return xController.atSetpoint() && yController.atSetpoint() && mAngleController.atSetpoint();

    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, false, false);
    }
}