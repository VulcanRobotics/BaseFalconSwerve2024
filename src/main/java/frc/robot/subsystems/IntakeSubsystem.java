/*package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.plaf.synth.SynthDesktopIconUI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Inputs;
import frc.robot.Robot;
import frc.robot.Constants.Tower;


public class IntakeSubsystem extends SubsystemBase {
    public static CANSparkMax m_rightPincerMotor = new CANSparkMax(17, MotorType.kBrushless);
    public static CANSparkMax m_leftPincerMotor = new CANSparkMax(18, MotorType.kBrushless);

    public static final DigitalInput m_intakePhotogate = new DigitalInput(9);

    private XboxController m_driverXbox = Inputs.m_driverXbox;

    public static double intakeSpeed = 0.0;
    public static double leftPincerSpeed = 0.0;

    private double startTime = System.currentTimeMillis();
    private double elapsedtime = 0.0;
    private boolean startClock = true;
    private static boolean haveCube = false;
    private boolean haveCubeOnce = false;
    private boolean becreamptuous = false; 
    public static boolean dontBringIn = false;
    
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
        intakeSpeed = 1;
        becreamptuous = true;
        break;
    }


    @Override
    public void periodic(){

        SmartDashboard.putBoolean("PHOTOGATE", getIntakePhotogate());

        
        //Detects whether the photogate senses something or not, changing the variable depending on the value
        if (getIntakePhotogate()) {
            haveCube = true;
        } else {
            haveCube = false;
        }

        //if the left trigger is pushed, the intake goes down and the clock (for the function) turns on if the cube is sensed
        if (m_driverXbox.getLeftTriggerAxis() > 0.1){
            PneumaticSubsystem.setIntakeState(true);
            intakeSpeed = 0.75;
            dontBringIn = false;
            if (!haveCube) {
                haveCubeOnce = false;
            }
        } else {
            PneumaticSubsystem.setIntakeState(false);
        }
        //this just reverses the motors, spitting the cube out
        if (m_driverXbox.getRightTriggerAxis() > 0.1 || Inputs.m_operatorControl.getRawButton(9) || TowerSubsystem.autoEject == true){
            intakeSpeed = -0.15;
            haveCube = false;
            becreamptuous = true;
            dontBringIn = true;

            if (TowerSubsystem.autoEject == true) {
                TowerSubsystem.autoEject = false;
            }

            //PneumaticSubsystem.setEjectState(true);
        } else {
            //PneumaticSubsystem.setEjectState(false);
        }




        //these are cases used for autonomous
        switch (Inputs.autonRequestIntakeGoTo){
            case IGNORE: 

                break;

            case DOWN:
                PneumaticSubsystem.setIntakeState(true);
                break;
            case INTAKE:
                if (!PneumaticSubsystem.intakeDeployed){
                    PneumaticSubsystem.setIntakeState(true);
                }
                
                
                intakeSpeed = 1;
                //m_leftPincerMotor.set(intakeSpeed);
                becreamptuous = true;
                break;

            



            
            case UP:
                PneumaticSubsystem.setIntakeState(false);

                break;

            case GRABINTAKE:
                intakeSpeed = -0.2;
                haveCube = false;
                becreamptuous = true;
                dontBringIn = true;
                //m_leftPincerMotor.set(intakeSpeed);

                //autoPinch();

                break;
                
            default:
                break;
        }
        

        
 
        //This finally goes over the conditions and gives the motor power dependent on which is satified
        if (haveCube) { //If you have the cube, stop the motors
            if (haveCubeOnce == false) {
                startClock = true;
                haveCubeOnce = true;
            }
            firstPass = false;
            keepSpinning(250);
            m_leftPincerMotor.set(intakeSpeed);
        } else if (m_driverXbox.getLeftTriggerAxis() < 0.1 && m_driverXbox.getRightTriggerAxis() < 0.1 && becreamptuous == false && dontBringIn == false){ //if nothing is being pressed, have the motors spin in slowly
            if (firstPass == false) {
                startClock = true;
                firstPass = true;
            }
            keepSpinning(3000);
            m_leftPincerMotor.set(intakeSpeed);
        } else { //if you are pressing a trigger though, set the speed normally
            m_leftPincerMotor.set(intakeSpeed);
            becreamptuous = false;
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

} */

