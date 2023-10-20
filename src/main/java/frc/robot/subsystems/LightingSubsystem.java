package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj.DriverStation;

public class LightingSubsystem extends SubsystemBase{

    //Where the Sparkmax light controller is
    private final Spark m_light1 = new Spark(9);

    //Gets alliance name to get the correct alliance color during match
    public static String kAllianceColorName = DriverStation.getAlliance().toString().strip().toLowerCase();
    
    //Helps determine which lights should be on during auton
    public static boolean kInAuton = false;
    

    public double LEDColor = 0.93; //Currently solid white, but will change throughout match to correspond with action

    public double DefaultLEDColor = 0.93; //Default color will be solid white to indicate whether a color has been assigned yet

    /* List of potential colors that would be used if any of them become true */
    private static boolean redLED = false;
    private static boolean blueLED = false;
    private static boolean yellowLED = false;
    private static boolean greenLED = false;
    private static boolean orangeLED = false;
    private static boolean heartbeatLED = false;
    private static boolean purpleLED = false;

    /* Some clock stuff to add a nice flashing feature during AUTON to help indicate end of auton */
    static boolean startClock;
    double startTime;
    double elapsedtime;
    double n;
    boolean flipColor;
    double timeToNextFlash;

    String currentColor = "Red";
    /* Some more clock stuff to add a nice flashing feature during TELOP to help indicate if the robot sees a cube during autoTurn*/
    private static boolean kstartClock = true;
    private static double kstartTime;
    private static double kelapsedtime;
    private static boolean kflipColor;
    private static double ktimeToNextFlash;

    public void toggleLight() { //This is a list of if statements that helps check to see if a certain color is being called for, if so, assign it to the corresponding boolean
        if (currentColor == "Purple") {
            currentColor = "Yellow";
        } else {
            currentColor = "Purple";
        }
    }

    public static void startAutonFlash() { //Helps assign the autonflash to the AutoManager
        startClock = true;
    }

    public void autonFlash(){ /* Some clock stuff to add a nice flashing feature during AUTON to help indicate end of auton */
        
        /*    
        if (startClock == true){
            startTime = System.currentTimeMillis(); //All clocks are in Milliseconds
            startClock = false;
            elapsedtime = 0.0;
            timeToNextFlash = elapsedtime+200;
        }
        else {
            elapsedtime = System.currentTimeMillis() - startTime;
        }
    

        if (elapsedtime > timeToNextFlash) { 
            flipColor = !flipColor;
            timeToNextFlash = elapsedtime+200;
        } 
        
        if (kInAuton) { //just a safety feature to prevent this from flashing while not in auton
            if (flipColor) {
                setLight("yellow");
            } else {
                setLight("blue");
            }
    
        }*/
        
   
    }

    public static void flash(){  /* Some more clock stuff to add a nice flashing feature during TELOP to help indicate if the robot sees a cube during autoTurn*/
        
            
        if (kstartClock == true){
            kstartTime = System.currentTimeMillis(); //All clocks are in Milliseconds
            kstartClock = false;
            kelapsedtime = 0.0;
            ktimeToNextFlash = kelapsedtime+200;
        }
        else {
            kelapsedtime = System.currentTimeMillis() - kstartTime;
        }
    

        if (kelapsedtime > ktimeToNextFlash) { 
            kflipColor = !kflipColor;
            ktimeToNextFlash = kelapsedtime+200;
        } 

        if (kflipColor) {
            //setLight("yellow");
        } else {
            //setLight("blue");
        }
        
   
    }

    @Override
    public void periodic() {


        /*if (kAllianceColorName.startsWith("b")) {
            DefaultLEDColor = -0.51; //A twinkled blue color
        } else if (kAllianceColorName.startsWith("r")) {
            DefaultLEDColor = -0.49; //A lava red color
        } else {
            DefaultLEDColor = 0.93; //White if somehow it can't find the team color
        }

        //NEEDS TO BE OPTIMIZED:
        /* From the booleans, they set the LEDColor which then get sent to the Spark controller */
       /* if (yellowLED) {
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


        m_light1.set(LEDColor);*/

        switch(currentColor) {
            case "Red":
                m_light1.set(-0.25);
                break;
            case "Yellow":
                m_light1.set(0.67);
                break;
            case "Purple":
                m_light1.set(0.91);
                break;
            default: 
                break;
        }
    }

    

}
