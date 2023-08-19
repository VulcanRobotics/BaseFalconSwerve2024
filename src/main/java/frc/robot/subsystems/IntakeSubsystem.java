package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.plaf.synth.SynthDesktopIconUI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Robot;


public class IntakeSubsystem extends SubsystemBase {
    public static CANSparkMax m_rightPincerMotor = new CANSparkMax(17, MotorType.kBrushless);
    public static CANSparkMax m_leftPincerMotor = new CANSparkMax(18, MotorType.kBrushless);

    public static final DigitalInput m_intakePhotogate = new DigitalInput(9);


    public static double intakeSpeed = 0.0;
    public static double leftPincerSpeed = 0.0;

    private double startTime = System.currentTimeMillis();
    private double elapsedtime = 0.0;

    private double holdstartTime = System.currentTimeMillis();
    private double holdelapsedtime = 0.0;

    private boolean startClock = true;
    private static boolean haveCube = false;
    private boolean haveCubeOnce = false;
    private boolean becreamptuous = false; 
    public static boolean dontBringIn = false;
    public boolean spitting = false;
    public boolean startHolding = false;
    private boolean firstPass = true;

    public boolean getIntakePhotogate() {
        return m_intakePhotogate.get();
    }

    public void keepSpinning(double time){
        
        if (startClock == true){
        startTime = System.currentTimeMillis();
        startClock = false;
        elapsedtime = 0.0;
        }
        else {
            elapsedtime = System.currentTimeMillis() - startTime;
        }
    

        if (elapsedtime < time) { //currently 0.5 seconds
            intakeSpeed = 0.25;
        } else {
            intakeSpeed = 0.0;
        }
   
    }

    public void hold3(){


        if (startClock == true){
            startTime = System.currentTimeMillis();
            startClock = false;
            elapsedtime = 0.0;
        }
        else {
            elapsedtime = System.currentTimeMillis() - startTime;
        }
    

        if (elapsedtime < 3000) { //currently 0.5 seconds
            intakeSpeed = 0.5;
        } else {
            intakeSpeed = 0.0;
        }
   
    }



    public static boolean getHaveCube() {
        if (haveCube) {
            return true;
        } else {
            return false;
        }
    }

    public void intake() {
        intakeSpeed = 0.75;
    }

    public void spit() {
        intakeSpeed = -0.15;
    }

    public void holdBall() {
        intakeSpeed = 0.0;
    }


    @Override
    public void periodic(){

        //SmartDashboard.putBoolean("PHOTOGATE", getIntakePhotogate());

        
        //Detects whether the photogate senses something or not, changing the variable depending on the value
        haveCube = getIntakePhotogate();
        
        
        //This finally goes over the conditions and gives the motor power dependent on which is satified
        
        if (haveCube) { //If you have the cube, stop the motors
            //keepSpinning(250);
            intakeSpeed = 0.0;
            m_leftPincerMotor.set(intakeSpeed);
        }
        else {
            m_leftPincerMotor.set(intakeSpeed);
        }
        

        SmartDashboard.putBoolean("Intake Deployed", PneumaticSubsystem.intakeDeployed);

    }

    
    

    //This is a function used in auton to spin the motors in
    public static void spinMotors(boolean reversed) {
        if (!reversed) {
            m_leftPincerMotor.set(0.5);
        } else {
            m_leftPincerMotor.set(0.5*-1);
        }
    }

} 

