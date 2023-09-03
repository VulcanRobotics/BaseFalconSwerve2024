package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.drive.PhysicsSim;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private WPI_TalonFX mAngleMotor;
    private WPI_TalonFX mDriveMotor;
    private WPI_CANCoder angleEncoder;
    private CANCoderSimCollection cancoder_sim;
    private double turnAbsolutePositionRad = 0.0;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    @AutoLog
    /** Static class to record module inputs for logging **/
    public static class SwerveModuleInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        // public double driveVelocityFilteredRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double[] {};
        public double[] driveTempCelcius = new double[] {};

        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[] {};
        public double[] turnTempCelcius = new double[] {};
    }

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        
        if (Constants.getRobot() == RobotType.ROBOT_SIMBOT) {
            this.angleOffset = new Rotation2d(0.0);
        }
        else {
            this.angleOffset = moduleConstants.angleOffset;
        }
        
        /* Angle Encoder Config */
        angleEncoder = new WPI_CANCoder(moduleConstants.cancoderID, "DriveSubsystemCANivore");
        configAngleEncoder();
        cancoder_sim = angleEncoder.getSimCollection();

        /* Angle Motor Config */
        mAngleMotor = new WPI_TalonFX(moduleConstants.angleMotorID, "DriveSubsystemCANivore");
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new WPI_TalonFX(moduleConstants.driveMotorID, "DriveSubsystemCANivore");
        configDriveMotor();

        lastAngle = getState().angle;

        // If we're in Simulation, initialize Falcon motor simulation
        if (Constants.getRobot() == RobotType.ROBOT_SIMBOT) {
          simulationInit(); 
        }
    }

    /** Update input values for Logging and Simulation **/
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.drivePositionRad =
            Units.rotationsToRadians(mDriveMotor.getSelectedSensorPosition() / 2048) /
            1.0; // Constants.Swerve.driveGearRatio;        
        inputs.driveVelocityRadPerSec = 
            Units.rotationsToRadians(mDriveMotor.getSelectedSensorVelocity() / 2048 * 10) /
            1.0; // Constants.Swerve.driveGearRatio;
        inputs.driveAppliedVolts = mDriveMotor.getMotorOutputVoltage();
        inputs.driveCurrentAmps = new double[] {mDriveMotor.getStatorCurrent()};
        inputs.driveTempCelcius = new double[] {mDriveMotor.getTemperature()};
        
        inputs.turnPositionRad =
            Units.rotationsToRadians( mAngleMotor.getSelectedSensorPosition() / 2048 ) /
            Constants.Swerve.angleGearRatio;
        inputs.turnVelocityRadPerSec = 
            Units.rotationsToRadians( mAngleMotor.getSelectedSensorVelocity() / 2048 * 10 ) /
            Constants.Swerve.angleGearRatio;
        inputs.turnAppliedVolts = mAngleMotor.getMotorOutputVoltage();
        inputs.turnCurrentAmps = new double[] {mAngleMotor.getStatorCurrent()};
        inputs.turnTempCelcius = new double[] {mAngleMotor.getTemperature()};
        
        if (Constants.getRobot() == RobotType.ROBOT_SIMBOT) {
            double angleDiffRad = inputs.turnVelocityRadPerSec * 0.02; // / Constants.Swerve.angleGearRatio;
            turnAbsolutePositionRad += angleDiffRad;
            while (turnAbsolutePositionRad < 0) {
                turnAbsolutePositionRad += 2.0 * Math.PI;
            }
            while (turnAbsolutePositionRad > 2.0 * Math.PI) {
                turnAbsolutePositionRad -= 2.0 * Math.PI;
            }

            // System.out.println("turnVelocity RadsPerSec: " + inputs.turnVelocityRadPerSec);
            long cancoder_units = Math.round( 4096 * (turnAbsolutePositionRad/(2.0 * Math.PI)) );
            cancoder_sim.setRawPosition( (int)cancoder_units);

            inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
        }
        else { // This is real, not simulation
            inputs.turnAbsolutePositionRad =
                Math.toRadians(Conversions.falconToDegrees(
                    angleEncoder.getAbsolutePosition(),
                    Constants.Swerve.angleGearRatio)
                );
        }
        //System.out.println("cancoder: " + angleEncoder.getAbsolutePosition());
    }

    /** Initialize Falcon motor simulation **/
    public void simulationInit() {
		PhysicsSim.getInstance().addTalonFX(mDriveMotor, 0.5, 6800);
		PhysicsSim.getInstance().addTalonFX(mAngleMotor, 0.5, 6800);
	}
	
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
        //System.out.println("Out percent: " + mDriveMotor.getMotorOutputPercent());
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
}