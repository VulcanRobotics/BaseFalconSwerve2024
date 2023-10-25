package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;
import frc.robot.subsystems.LightingSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class ArmSubsystem extends SubsystemBase {

    public boolean isCommandRunning = false;
    //stating the proximity switches
    DigitalInput m_towerDownProximity = new DigitalInput(3);
    static DigitalInput m_towerUpProximity = new DigitalInput(2);
    DigitalInput m_ElbowDownProximity = new DigitalInput(8);
    static DigitalInput m_ElbowUpProximity = new DigitalInput(7);
    //stating the string potentiometers
    private static AnalogPotentiometer m_stringPotentiometerTower = new AnalogPotentiometer(new AnalogInput(0));
    private static AnalogPotentiometer m_stringPotentiometerElbow = new AnalogPotentiometer(new AnalogInput(1));
    private static AnalogPotentiometer m_PotentiometerWrist = new AnalogPotentiometer(new AnalogInput(2));
    //stating the PID controllers
    private static ProfiledPIDController towerPID = new ProfiledPIDController(10,0.0, 0.1, new TrapezoidProfile.Constraints(1, 1));
    private static ProfiledPIDController elbowPID = new ProfiledPIDController(10, 0.0, 0.1, new TrapezoidProfile.Constraints(1, 1));
    //stating the Talon 500 motors for the tower
    private static TalonFX mTower = new TalonFX(13, "roborio");
    private static TalonFX mElbow = new TalonFX(12, "roborio");
    //stating the wrist motors and their relative encoder
    private static CANSparkMax m_Wrist = new CANSparkMax(3, MotorType.kBrushless);
    private RelativeEncoder m_encoderWrist = m_Wrist.getEncoder();

    //All variables used within the class:
//*******************************************************//
    private double m_towerEncoderZero = 0;
    private double m_elbowEncoderZero = 0;
    
    private boolean m_useTowerEncoder = false;
    private boolean m_useElbowEncoder = false;

    private static double towerEncoder180Ticks = 400.0;
    private static double elbowEncoder180Ticks = 49.65;
    private static double m_towerEncoderMax = -383.340546 - 135.450546; 
    private static double m_elbowEncoderMax = 73.00 - (-8.50); 
    
    private static double m_stringTower = 0.0;
    private static double m_stringElbow = 0.0;
    private double m_wristEncoder = 0.0;

    private static double mTowerSpeed = 0;
    private static double mElbowSpeed = 0;
    private double mWristSpeed = 0;

    private boolean firstLoop = true;

    //public static boolean noIntake = false;

    

    private double initialWristEncoder = m_encoderWrist.getPosition();
    public final double wristEncoder180 = 47.642;
    XboxController controller;

    public static Joystick extraController = new Joystick(2);

    




    public static boolean autoEject = false;
//*******************************************************//

    public ArmSubsystem() {
        // This brake mode for the two motors ensure that the motors will try to maintain it position while pulled down by gravity
        mTower.setNeutralMode(NeutralMode.Brake);
        mElbow.setNeutralMode(NeutralMode.Brake);
    }

     //This function is used everytime we need to automatically move the arm/elbow, uitilizes PID for smooth movement
    private boolean goToPosition(double Tvalue, double Evalue, boolean towerPriority, boolean flipArm) {
        
        /* Grabs values from the string potentiometers built on the robot */
        m_stringTower = m_stringPotentiometerTower.get();
        m_stringElbow = m_stringPotentiometerElbow.get();

        /* used the  */
        boolean elbowDone = false;
        boolean towerDone = false;
        SmartDashboard.putBoolean("TOWER DONE", towerPID.atGoal());

        if (towerPriority){ //The elbow only starts moving when the tower reaches its target point
            

            if (m_stringTower < Tvalue + 0.01 && m_stringTower > Tvalue - 0.01){ //towerDone = towerPID.atSetpoint()
                towerDone = true;

                if (m_stringElbow < Evalue + 0.01 && m_stringElbow > Evalue - 0.01){
                    elbowDone = true;
                } else{
                    mElbowSpeed = -elbowPID.calculate(m_stringElbow,Evalue);
                }

            } else {
                mTowerSpeed = -towerPID.calculate(m_stringTower,Tvalue);
            }
            
        }else if (flipArm){ //The elbow only starts moving if the arm is past the halfway point (reaching behind)
            if (m_stringTower > 0.43){

                if (m_stringTower < Tvalue + 0.01 && m_stringTower > Tvalue - 0.01){ //towerDone = towerPID.atSetpoint()
                    towerDone = true;
                }

                if (m_stringElbow < Evalue + 0.01 && m_stringElbow > Evalue - 0.01){
                    elbowDone = true;
                }

                mTowerSpeed = -towerPID.calculate(m_stringTower,Tvalue);
                mElbowSpeed = -elbowPID.calculate(m_stringElbow,Evalue);

            } else{
                if (m_stringTower < Tvalue + 0.01 && m_stringTower > Tvalue - 0.01){ 
                    towerDone = true;
                }

                mTowerSpeed = -towerPID.calculate(m_stringTower,Tvalue);
            }
        } else{ //If neither of the two previous conditions are needed, then move normally
            mTowerSpeed = -towerPID.calculate(m_stringTower,Tvalue);

            if (m_stringTower < Tvalue + 0.01 && m_stringTower > Tvalue - 0.01){ 
                towerDone = true;
            }

            if (m_stringElbow < Evalue + 0.01 && m_stringElbow > Evalue - 0.01){
                elbowDone = true;
            }

            mElbowSpeed = -elbowPID.calculate(m_stringElbow,Evalue);
        }

        if (towerDone && elbowDone){
            mElbowSpeed = 0.0;
            mTowerSpeed = 0.0;
            return true;
        } else{
            return false;
        }
        
        




    }

    //these are the multiple position functions used to move the two arms. They are all functions in order to uitilize a name rather than a number to ask for a position
//*******************************************************//
    public boolean highPlace(){
        return goToPosition(.512, .230, false, true);
    }
    public boolean midPlace(){
        return goToPosition(.353, .259, false, false);
    }
    public boolean humanPlayerGrab(){
        return goToPosition(.395, .3087, false, false);
    }
    public boolean humanPlayerGrabDrop(){
        return goToPosition(.377, .701, false, false);
    }
  
    public boolean pickUpFromIntake() {
        return goToPosition(0.322, 0.71, false, false);
    }
    public  boolean tuckArm(){
        return goToPosition(0.27, 0.911, false, false);
    }

    public  boolean Origin(boolean cube){ //the setup condition is used initially if we want the arm to be within the frame perimeter
        if (!cube){
            return goToPosition(0.395, 0.372, false, false);
        } else {
            return goToPosition(0.375, 0.372, false, false);
        }
    }

    //This helps to move the wrist automatically both in Auton mainly and telop for the operator
    public void goToWristPosition(double wristValue){
        SmartDashboard.putNumber("wrist target", wristValue);

        if (m_wristEncoder <  wristValue - 3) {
            mWristSpeed = 0.04;
        } else if (m_wristEncoder > wristValue + 3) {
            mWristSpeed = -0.04;
        } else{
            mWristSpeed = 0;
        }
    }

    public void stopMovement(){
        mElbowSpeed = 0.0;
        mTowerSpeed = 0.0;
    }




    public void joystickMovement(double x, double y, double z) {
        mElbowSpeed = x;
        mTowerSpeed = y;
        if (Math.abs(z) > 0.5) {
            mWristSpeed = z/3;
        } else {
            mWristSpeed = 0.0;
        }

        boolean isArmOnTop = m_towerUpProximity.get();
        boolean isArmOnBottom = m_towerDownProximity.get();
        
        boolean isElbowOnTop = !m_ElbowUpProximity.get();
        boolean isElbowOnBottom = !m_ElbowDownProximity.get();

        //Limit Switches to make any last changes to the speed before setting it, must go last! KEEP
        if (isArmOnTop && mTowerSpeed > 0) {
            mTowerSpeed = 0;
            if (m_useTowerEncoder == false) {
                m_useTowerEncoder = true;
                m_towerEncoderZero = mTower.getSelectedSensorPosition();
            }
        } else if (isArmOnBottom && mTowerSpeed < 0) {
            mTowerSpeed = 0;
            if (m_useTowerEncoder == false) {
                m_useTowerEncoder = true;
                m_towerEncoderZero = mTower.getSelectedSensorPosition() - m_towerEncoderMax;
            }
        }

        if (isElbowOnTop && mElbowSpeed > 0) { 
            mElbowSpeed = 0;
            if (m_useElbowEncoder == false) {
                m_useElbowEncoder = true;
                m_elbowEncoderZero = mElbow.getSelectedSensorPosition();
            }
        } else if (isElbowOnBottom && mElbowSpeed < 0) { 
            mElbowSpeed = 0;
            if (m_useElbowEncoder == false) {
                m_useElbowEncoder = true;
                m_elbowEncoderZero = mElbow.getSelectedSensorPosition() - m_elbowEncoderMax;
            }
        }
        //////////
        //Claw limit?
        if (m_PotentiometerWrist.get() < -5 && mWristSpeed < 0){
            mWristSpeed = 0;
        } else if (m_PotentiometerWrist.get() > 5 && mWristSpeed > 0){
            mWristSpeed = 0;
        }

        mTower.set(ControlMode.PercentOutput, mTowerSpeed);
        mElbow.set(ControlMode.PercentOutput, mElbowSpeed);
        m_Wrist.set(mWristSpeed);

    }

    
    @Override
    public void periodic(){
        if (firstLoop){
            firstLoop = false;

            towerPID.reset(m_stringPotentiometerTower.get());
            elbowPID.reset(m_stringPotentiometerElbow.get());
        }

        boolean isArmOnTop = m_towerUpProximity.get();
        boolean isArmOnBottom = m_towerDownProximity.get();
        
        boolean isElbowOnTop = !m_ElbowUpProximity.get();
        boolean isElbowOnBottom = !m_ElbowDownProximity.get();

        towerPID.setTolerance(0.01, 0);
        
        

        //Built in encoder values 
         m_wristEncoder = m_encoderWrist.getPosition();


        //Limit Switches to make any last changes to the speed before setting it, must go last! KEEP
        if (isArmOnTop && mTowerSpeed > 0) {
            mTowerSpeed = 0;
            if (m_useTowerEncoder == false) {
                m_useTowerEncoder = true;
                m_towerEncoderZero = mTower.getSelectedSensorPosition();
            }
        } else if (isArmOnBottom && mTowerSpeed < 0) {
            mTowerSpeed = 0;
            if (m_useTowerEncoder == false) {
                m_useTowerEncoder = true;
                m_towerEncoderZero = mTower.getSelectedSensorPosition() - m_towerEncoderMax;
            }
        }

        if (isElbowOnTop && mElbowSpeed > 0) { 
            mElbowSpeed = 0;
            if (m_useElbowEncoder == false) {
                m_useElbowEncoder = true;
                m_elbowEncoderZero = mElbow.getSelectedSensorPosition();
            }
        } else if (isElbowOnBottom && mElbowSpeed < 0) { 
            mElbowSpeed = 0;
            if (m_useElbowEncoder == false) {
                m_useElbowEncoder = true;
                m_elbowEncoderZero = mElbow.getSelectedSensorPosition() - m_elbowEncoderMax;
            }
        }
        //////////
        //Claw limit?
        if (m_PotentiometerWrist.get() < -5 && mWristSpeed < 0){
            mWristSpeed = 0;
        } else if (m_PotentiometerWrist.get() > 5 && mWristSpeed > 0){
            mWristSpeed = 0;
        }

        mTower.set(ControlMode.PercentOutput, mTowerSpeed);
        mElbow.set(ControlMode.PercentOutput, mElbowSpeed);
        m_Wrist.set(mWristSpeed); 

        SmartDashboard.putNumber("String Potentiometer Tower", m_stringPotentiometerTower.get());
        SmartDashboard.putNumber("String Potentiometer Elbow", m_stringPotentiometerElbow.get());
        


        
        
        //SmartDashboard.putNumber("mTowerSpeed", mTowerSpeed);

        
        //m_Wrist.set(ControlMode.PercentOutput, mWristSpeed);

        //gripMotor.set(ControlMode.PercentOutput, mGripSpeed);
         /* 
        if(false){
            SmartDashboard.putNumber("mElbow", m_stringPotentiometerElbow.get());
            SmartDashboard.putNumber("mTowerSpeed", mTowerSpeed);
            SmartDashboard.putBoolean("upperProximity", isElbowOnTop);
            SmartDashboard.putBoolean("lowerProximity", isElbowOnBottom);
            SmartDashboard.putBoolean("Elbow down Prox", m_ElbowDownProximity.get());
            SmartDashboard.putBoolean("ikmode", IKMode);
            SmartDashboard.putNumber("mElbowSpeed", mElbowSpeed);
            SmartDashboard.putNumber("mWristSpeed", mWristSpeed);
        
            SmartDashboard.putNumber("initial wrist encoder", initialWristEncoder);

            SmartDashboard.putNumber("String Potentiometer Tower", m_stringPotentiometerTower.get());
            SmartDashboard.putNumber("String Potentiometer Elbow", m_stringPotentiometerElbow.get());
            SmartDashboard.putNumber("Wrist Encoder", m_wristEncoder);

        	SmartDashboard.putNumber("Tower Encoder", towerEncoder);
        	SmartDashboard.putNumber("Elbow Encoder", elbowEncoder);

        	SmartDashboard.putNumber("Tower Encoder", mTower.getSelectedSensorPosition());
        	SmartDashboard.putNumber("Elbow Encoder", mElbow.getSelectedSensorPosition());
        
        	SmartDashboard.putNumber("Tower IK Angle", 180*theta1/ Math.PI);
        	SmartDashboard.putNumber("Elbow IK Angle", 180*theta2/ Math.PI);

        	SmartDashboard.putNumber("Target X", targetX);
        	SmartDashboard.putNumber("Target Y", targetY);

        } */



    }

    
}
    


