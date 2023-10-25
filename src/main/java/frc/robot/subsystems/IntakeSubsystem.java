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

    public static CANSparkMax m_intakeMotor = new CANSparkMax(18, MotorType.kBrushless);

    public static final DigitalInput m_intakePhotogate = new DigitalInput(9);


    public static double intakeSpeed = 0.0;

    public static boolean haveCube = false;
    private static boolean haveCubeOnce = false;

    public static boolean dontBringIn = false;
    public boolean spitting = false;
    public boolean startHolding = false;
    
    /* 
    CLOCK VARIABLES!!! 
    Due to the migration to a one time command based function for Pathplanner and the Robot in general, clocks have been proved useful to run a function multiple times for a set duration. 
    Due to the use of multiple seperate timers for certain actions, the clocks all have different iterations of the same functioning variables to prevent overusing one of them:
    - startTime
    - elapsedtime
    - startClock
    The function for each set of these variables are below:
    */

    // For the "spit" function
    private double startTime = 0.0;
    private double elapsedtime = 0.0;
    private boolean startClock = true;
    // For the "keepSpinning" function
    private boolean kstartClock = false;
    private double kstartTime = 0.0;
    private double kelapsedtime = 0.0;
    // For the "forceSpit" function
    private boolean sstartClock = false;
    private double sstartTime = 0.0;
    private double selapsedtime = 0.0;

    public boolean getIntakePhotogate() { // this helps get the status on whether or not the intake has a cube or not
        return m_intakePhotogate.get();
    }

    public void spit(double time){ // This function helps the CubeTransfer AUTON command, it slightly eases the intake's grip to allow the arm to grab it
        
        if (startClock == true){
            startTime = System.currentTimeMillis();
            startClock = false;
            elapsedtime = 0.0;
        }
        else {
            elapsedtime = System.currentTimeMillis() - startTime;
        }
    

        if (elapsedtime < time) {
            intakeSpeed = -0.15;
        } else {
            intakeSpeed = 0.0;
        }
   
    }

    public void keepSpinning(double ktime){ // This function helps make sure that when the cube it taken in from the intake, that the intake rollers keep spinning for a split second to encure that the cube is held tight

        if (haveCubeOnce){
            kstartClock = true;
            haveCubeOnce = false;
        }
        if (kstartClock == true){
            kstartTime = System.currentTimeMillis();
            kstartClock = false;
            kelapsedtime = 0.0;
        }
        else {
            kelapsedtime = System.currentTimeMillis() - kstartTime;
        }
        System.out.println(kelapsedtime);

        if (kelapsedtime < ktime) { //currently 0.5 seconds
            intakeSpeed = 0.25;
        } 
    }

    public void forceSpit(double ktime){ // This overrides most intake speed changes as it comes last before going to the motor, its enabled from the driver to forcefully spit the cube out for a split second during TELEOP

        if (sstartClock == true){
            sstartTime = System.currentTimeMillis();
            sstartClock = false;
            selapsedtime = 0.0;
        }
        else {
            selapsedtime = System.currentTimeMillis() - sstartTime;
        }
        

        if (selapsedtime < ktime) { //currently 0.5 seconds
            intakeSpeed = -1.00;
        } 
    }

    public static boolean getHaveCube() { // Used for outside commands/if statements to see if it has a cube or not. Not rly for this class specifically
        if (haveCube) {
            return true;
        } else {
            return false;
        }
    }

    public static void intake() { // Turns on the intake
        intakeSpeed = 0.75;
        if (!haveCube) {
            haveCubeOnce = true;
            
        }
    }


    /* These three are used to enable/reset the timers to zero. */
    public void spit() {
        startClock = true;
    }
    public void forceSpit() {
        sstartClock = true;
    }
    public void holdBall() {
        intakeSpeed = 0.0;
    }


    @Override
    public void periodic(){


        
        //Detects whether the photogate senses something or not, changing the variable depending on the value
        haveCube = getIntakePhotogate();
        
        
        //This finally goes over the conditions and gives the motor power dependent on which is satified
        if (haveCube) { //If you have the cube, ensure the cube is held tight and stop the motors
            intakeSpeed = 0.02;
            
            spit(500);
            keepSpinning(250);
            //keepSpinning(250);
            
        }
        else { // Don't spit anymore if the cube is out, otherwise, don't change the intakespeed from what it currently is
            if (intakeSpeed < 0.0) {
                intakeSpeed = 0.0;
            }
        }
        
        forceSpit(250); // If the driver needs the cube out, this makes it happen.

        m_intakeMotor.set(intakeSpeed); //Finally, sets it to the intake motor

    }

    
    

    //This is a function used in auton to spin the motors 
    public static void spinMotors(boolean reversed) {
        if (!reversed) {
            m_intakeMotor.set(0.5);
        } else {
            m_intakeMotor.set(0.5*-1);
        }
    }

} 

