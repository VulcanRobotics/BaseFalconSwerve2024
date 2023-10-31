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


public class QuickLook extends CommandBase {    
    private Swerve s_Swerve;    
    private final boolean fieldCentric = false;

    //public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    public QuickLook(Swerve s_Swerve) {        
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);


    }

    @Override
    public void initialize(){
        VisionSubsystem.setFrontLimeLight(false);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = 0.0; //Slowly move forward
        double strafeVal = 0.0;
        double rotationVal = VisionSubsystem.XDist;
        double rotationKp = 0.8;

        
        

        /*if (!VisionSubsystem.seeCube) {
            rotationVal = 0.0;
            translationVal = 0.0;
        }*/

        

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity * rotationKp,  
            fieldCentric, 
            true
        );

        
    }

    @Override
    public boolean isFinished(){
        if (Math.abs(VisionSubsystem.XDist) < 0.01) {
            return true;
        } else {
            return false;
        }
    }
}