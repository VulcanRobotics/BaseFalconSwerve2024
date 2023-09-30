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


public class CubeSeek extends CommandBase {    
    private Swerve s_Swerve;    
    private final boolean fieldCentric = false;

    //public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    public CubeSeek(Swerve s_Swerve) {        
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);


    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = 0.5; //Slowly move forward
        double strafeVal = 0.0;
        double rotationVal = VisionSubsystem.XDist;

        
        

        /*if (!VisionSubsystem.seeCube) {
            rotationVal = 0.0;
            translationVal = 0.0;
        }*/

        if (VisionSubsystem.aSize >= 0.5) {
            PneumaticSubsystem.setIntakeState(true);
            IntakeSubsystem.intake(); //Begin to intake - Do we need this?
            translationVal = 0.5;
        } 

        if (IntakeSubsystem.haveCube == true) { //VisionSubsystem.seeCube == false || 
            translationVal = 0.0;
            rotationVal = 0.0;
        }


        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity,  
            fieldCentric, 
            true
        );

        
    }

    @Override
    public boolean isFinished(){
        if (IntakeSubsystem.haveCube) {
            double translationVal = 0.0;
            double strafeVal = 0.0;
            double rotationVal = 0.0;
            PneumaticSubsystem.setIntakeState(false);
            /* Drive */
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                //!robotCentricSup.getAsBoolean(), 
                fieldCentric, 
                true
            );
        }
        return IntakeSubsystem.haveCube;
    }
}