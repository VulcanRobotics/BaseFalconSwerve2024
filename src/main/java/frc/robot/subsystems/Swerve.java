package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.SwerveModuleInputsAutoLogged;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.drive.PhysicsSim;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.hal.simulation.*;
import edu.wpi.first.hal.*;

//import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public static SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public SwerveModule[] mSwerveMods;
    private double[] lastModulePositionsRad = new double[] {0.0, 0.0, 0.0, 0.0};
    private Translation2d fieldVelocity = new Translation2d();

    public static boolean cubeSeekToggle;

    private final SwerveModuleInputsAutoLogged[] swerveModuleInputs =
    new SwerveModuleInputsAutoLogged[] {new SwerveModuleInputsAutoLogged(),
        new SwerveModuleInputsAutoLogged(), new SwerveModuleInputsAutoLogged(),
        new SwerveModuleInputsAutoLogged()};

    public AHRS m_gyro;
    private double simYaw = 0.0;

    public Swerve() {
        m_gyro = new AHRS(SPI.Port.kMXP);
        zeroGyro();

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
                                                                   getPose(),
                                                                   VecBuilder.fill(0.05, 0.05, 0.05),
                                                                   VecBuilder.fill(0.05, 0.05, 0.05));
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


    public void cubeSeekToggle() {
        cubeSeekToggle = true;
    }
    
    /*public void cubeSeek() {
        double translationVal = 0.5;
        double strafeVal = 0.0;
        double rotationVal = VisionSubsystem.XDist;


        if (VisionSubsystem.seeCube == false || IntakeSubsystem.haveCube == true) {
            translationVal = 0.0;
            rotationVal = 0.0;
        }

        Rotation2d translation = 

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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);



        drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            //!robotCentricSup.getAsBoolean(), 
            true, 
            true
        );
    } */

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(ChassisSpeeds desiredChassisSpeeds) {
        // System.out.println("Auton: setModuleStates()....... ");
        SwerveModuleState[] desiredStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
            //SmartDashboard.putNumber("Cancoder" + mod.toString(), mod.getCanCoder().getDegrees());
        }
    }    

    public Pose2d getPose() { //This pose takes into account the Swerve Odometry, which is taking the wheel positions to calculate its place on the field
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getPose2(){ //This pose takes into account the Prediction Odometry, which is taking the camera inputs to calculate its position on the field
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) { //Resets the wheel odometry, should be paired with the getPose command
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetEstimator(Pose2d pose) { //Resets the Prediction odometry, should be paired with the getPose2 command
        swerveDrivePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
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
    }

    public Rotation2d getYaw() {
        //return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw()) : Rotation2d.fromDegrees(m_gyro.getYaw());
        // I think the Gyro returns -180 to +180 degrees, so if inverted, we should just flip the sign
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(- m_gyro.getYaw()) : Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    /** New method for Swerve(), necessary for Logging and Simulation with AdvantageKit **/
    public void periodic(){

        SmartDashboard.putNumber("GYRO ROLL", m_gyro.getRoll());

        if (Constants.getRobot() == RobotType.ROBOT_SIMBOT) {
            // If we're in simulaton, run the next iteration of physics simulation for the Falcon motors
            PhysicsSim.getInstance().run();
        }
        
        // Log the inputs for each SwerveModule
        for (int i = 0; i < 4; i++) {
            mSwerveMods[i].updateInputs(swerveModuleInputs[i]);
            Logger.getInstance().processInputs("Drive/Module" + Integer.toString(i),
                swerveModuleInputs[i]);
        }
        //System.out.println("turn rad: " + swerveModuleInputs[0].turnVelocityRadPerSec);
        //System.out.println("cancoder: " + mSwerveMods[0].getCanCoder().getDegrees());

        // Update each turn angle measurements for each SwerveModule
        Rotation2d[] turnPositions = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            turnPositions[i] =
                new Rotation2d(swerveModuleInputs[i].turnAbsolutePositionRad);
        }
        //System.out.println("turnPosition[0]: " + turnPositions[0].getDegrees());
        
        /** Update robot odometry **/

        //
        SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStatesDiff[i] = new SwerveModuleState(
                (swerveModuleInputs[i].drivePositionRad - lastModulePositionsRad[i])
                * Constants.Swerve.chosenModule.wheelDiameter/2.0, turnPositions[i]);
            lastModulePositionsRad[i] = swerveModuleInputs[i].drivePositionRad;
        }

        ChassisSpeeds chassisStateDiff =
            Constants.Swerve.swerveKinematics.toChassisSpeeds(measuredStatesDiff);

        if (Constants.getRobot() == RobotType.ROBOT_SIMBOT) {
            double simYawChangeRad = chassisStateDiff.omegaRadiansPerSecond * 0.02 * 10;
            // x 10 fudge factor for more realistic response in simulator
            // (Otherwise the Gyro yaw update is too slow)
            double simYawChangeDegrees = 360 * simYawChangeRad / (2*Math.PI);
            //System.out.println("Yaw change: " + simYawChangeDegrees);

            // Update the simulated Gyro with the new yaw value, flipped (minus) for inverted Gyro
            simYaw -= simYawChangeDegrees;
            if (simYaw > 180.0) {simYaw -= 360.0;}
            if (simYaw < -180.0) {simYaw += 360.0; }
            int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
            angle.set(simYaw);
        }

        if (cubeSeekToggle == true) {
            //cubeSeek();
        }
        
        // Update and log Gyro value (whether it's simulated or real)
        Rotation2d currentYaw = getYaw();
        // System.out.println("Yaw: " + getYaw());
        swerveOdometry.update(currentYaw, getModulePositions());  
        swerveDrivePoseEstimator.update(currentYaw, getModulePositions());
        Logger.getInstance().recordOutput("Gyro_Yaw", currentYaw.getDegrees());
        
        // Log odometry pose
        Logger.getInstance().recordOutput("Odometry/Robot", swerveOdometry.getPoseMeters());

        // Update field velocity
        SwerveModuleState[] measuredStates =
            new SwerveModuleState[] {null, null, null, null};
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = new SwerveModuleState(
                swerveModuleInputs[i].driveVelocityRadPerSec * Constants.Swerve.chosenModule.wheelDiameter/2.0,
                turnPositions[i]);
            //System.out.println("turnPositions " + i + ": " + turnPositions[i]);
        }

        ChassisSpeeds chassisState = Constants.Swerve.swerveKinematics.toChassisSpeeds(measuredStates);
        fieldVelocity = new Translation2d(chassisState.vxMetersPerSecond,
            chassisState.vyMetersPerSecond).rotateBy(getRotation());

        // Log measured SwerveModule states
        Logger.getInstance().recordOutput("SwerveModuleStates/Measured",
            measuredStates);
        
        // Update values on Dashboard
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
        }
        SmartDashboard.putNumber("Odometry X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry Rot", swerveOdometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Yaw", getYaw().getDegrees());

        SmartDashboard.putNumber("Estimator X", swerveDrivePoseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Estimator Y", swerveDrivePoseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Estimator Rot", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees());

        
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return swerveOdometry.getPoseMeters().getRotation();
    }
}