package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public AHRS m_gyro;
    private Pose2d pose = new Pose2d();
    public SwerveDriveKinematics kinematics;
    public ChassisSpeeds currentChassisSpeeds;
    public Rotation2d lastGyroRotation;
    private ProfiledPIDController mAngleController = new ProfiledPIDController(1.5, 0.1, 0.2,
         new Constraints(Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity));

    

    public Swerve() {
        currentChassisSpeeds =  new ChassisSpeeds(0.0, 0.0, 0.0);
        mAngleController.setTolerance(Math.toRadians(0.25));
        mAngleController.enableContinuousInput(Math.PI, 3*Math.PI);
        m_gyro = new AHRS(SPI.Port.kMXP);
        zeroGyro();
        mAngleController.reset(getYaw().getRadians());
        lastGyroRotation = getYaw();
        this.kinematics = Constants.Swerve.swerveKinematics; //May be redundant

        mSwerveMods = new SwerveModule[] {
            
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        //Timer.delay(0.5);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
                                                                getYaw(),
                                                                getModulePositions(),
                                                                new Pose2d(),
                                                                VecBuilder.fill(
                                                                    Constants.VisionConstants.kOdometryStdevs[0],
                                                                    Constants.VisionConstants.kOdometryStdevs[1],
                                                                    Constants.VisionConstants.kOdometryStdevs[2]),
                                                                VecBuilder.fill(
                                                                    Constants.VisionConstants.kVisionStdevs[0],
                                                                    Constants.VisionConstants.kVisionStdevs[1],
                                                                    Constants.VisionConstants.kVisionStdevs[2]));
    }

       
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, double rawRotation) {

        //If statement for "lane-assist" (ensures last heading set is kept despite module inconsistencies)
        //adjust (lower) deadband to make "lastrotation more accurate"
        if (Math.abs(rawRotation) > 0.04) {
            lastGyroRotation = getYaw();
        } else {
            mAngleController.setGoal(lastGyroRotation.getRadians());
            rotation = mAngleController.calculate(getYaw().getRadians()); 
        }

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {


        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(ChassisSpeeds desiredChassisSpeeds) {
        currentChassisSpeeds = desiredChassisSpeeds;
        SwerveModuleState[] desiredStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public void setModuleStates(ChassisSpeeds desiredChassisSpeeds, boolean isOpenLoop) {
        currentChassisSpeeds = desiredChassisSpeeds;
        SwerveModuleState[] desiredStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
        }
    } 

    


    public Pose2d getPose2() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getPose(){
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation(){
        return getPose().getRotation();
    }

    public void resetPose(Pose2d pose) {
        this.pose = pose;

    }

    public ChassisSpeeds getSpeeds() {
        return currentChassisSpeeds;
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        m_gyro.zeroYaw();
        lastGyroRotation = new Rotation2d(0.0);

    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw()) : Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        //swerveOdometry.update(getYaw(), getModulePositions());
        swerveDrivePoseEstimator.update(getYaw(), getModulePositions());
        pose = swerveDrivePoseEstimator.update(getYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
        }
        SmartDashboard.putNumber("Odometry X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry Rot", swerveOdometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("Estimator X", swerveDrivePoseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Estimator Y", swerveDrivePoseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Estimator Rot", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees());

        SmartDashboard.putNumber("Gyro YAW", getYaw().getDegrees());
        SmartDashboard.putNumber("True Gyro Yaw", m_gyro.getYaw());
        
    }
}