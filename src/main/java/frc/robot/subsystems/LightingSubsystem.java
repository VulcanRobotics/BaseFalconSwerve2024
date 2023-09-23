package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.CancellationException;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DriverStation;

public class LightingSubsystem extends SubsystemBase{
    private final Spark m_light1 = new Spark(9);

    public static String kAllianceColorName = DriverStation.getAlliance().toString().strip().toLowerCase();
    public static boolean kInAuton = false;
    

    public double LEDColor = 0.93; //solid white
    public double DefaultLEDColor = 0.93; //solid white
    private boolean redLED = false;
    private boolean blueLED = false;
    private boolean yellowLED = false;
    private boolean greenLED = false;
    private boolean orangeLED = false;
    private boolean heartbeatLED = false;
    private boolean purpleLED = false;

    boolean startClock;
    double startTime;
    double elapsedtime;
    double n;
    boolean flipColor;


    public void setLight(String color) {
        if (color == "red") {
            redLED = true;
        } 
        if (color == "blue") {
            blueLED = true;
        } 
        if (color == "yellow") {
            yellowLED = true;
        } 
        if (color == "green") {
            greenLED = true;
        } 
        if (color == "orange") {
            orangeLED = true;
        } 
        if (color == "heartbeat") {
            heartbeatLED = true;
        } 
        if (color == "purple") {

        }
    }

    public void autonFlash(){
        
            
        if (startClock == true){
            startTime = System.currentTimeMillis();
            startClock = false;
            elapsedtime = 0.0;
            n = 0.0; 
        }
        else {
            elapsedtime = System.currentTimeMillis() - startTime;
        }
    

        if (elapsedtime > (2000*n)+startTime) { //currently 0.5 seconds
            flipColor = !flipColor;
            n = n+1;
        } 
        if (kInAuton) {
            if (flipColor) {
                setLight("yellow");
            } else {
                setLight("blue");
            }
    
        }
        
   
    }

    @Override
    public void periodic() {


        if (kAllianceColorName.startsWith("b")) {
            DefaultLEDColor = -0.51;
        } else if (kAllianceColorName.startsWith("r")) {
            DefaultLEDColor = -0.49;
        } else {
            DefaultLEDColor = 0.93;
        }

        if (yellowLED) {
            LEDColor = 0.67; //solid gold
            yellowLED = false;
        } else if (purpleLED) {
            LEDColor = 0.91; //solid violet
            purpleLED = false;
        } else if (heartbeatLED) {
            LEDColor = -0.25; //Heartbeat, red
            redLED = false;
        } else if (greenLED) {
            LEDColor = -0.53; //Twinkles, party palette
            greenLED = false;                  
        } else if (redLED) {
            LEDColor = 0.57;
            redLED = false;
        } else if (blueLED) {
            LEDColor = 0.85;
            blueLED = false;
        } else {
            LEDColor = DefaultLEDColor;
        }

        autonFlash();

        m_light1.set(LEDColor);
        

    }

    

}
