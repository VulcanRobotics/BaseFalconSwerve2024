package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Balance extends CommandBase {    
    private Swerve s_Swerve;    
    private final boolean fieldCentric = false;
    private double rotationVal = 0.0;
    private double balanceScale = 100;

    private double changeThreshold = 0.85;

    private double lastRoll = 0.0;

    private boolean onRamp = false;

    private boolean done = false;
    //private boolean firstOvershot = false;

    //public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    public Balance(Swerve s_Swerve) {        
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }


    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = -s_Swerve.m_gyro.getRoll()/balanceScale; //Slowly move forward
        double strafeVal = 0.0;
        rotationVal = 0.0;

        /*if (!done) {
            if (onRamp) {
                if (Math.abs(-s_Swerve.m_gyro.getRoll() - lastRoll) > changeThreshold) {
                    strafeVal = 0.1;
                    done = true;
                } else {
                    translationVal = -0.15;
                }
            } else if (Math.abs(s_Swerve.m_gyro.getRoll()) > 5) {
                onRamp = true;
            }
        } else {
            translationVal = 0.0;
            strafeVal = 0.0;
            rotationVal = 0.0;
        } */

        /*if (Math.abs(s_Swerve.m_gyro.getRoll()) < 0.1) {
            balanceScale = 150;
        }*/

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity,  
            fieldCentric, 
            true
        );

        lastRoll = -s_Swerve.m_gyro.getRoll();
    }

    @Override
    public boolean isFinished(){
        /*if (rotationVal > -0.1 && rotationVal < 0.1) {
            double translationVal = 0.0;
            double strafeVal = 0.0;
            double rotationVal = 0.001;

            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                //!robotCentricSup.getAsBoolean(), 
                fieldCentric, 
                true
            );

            return true;
        } else {
            return false;
        } */
        return false;
    }
}