package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    //private BooleanSupplier robotCentricSup;
    private BooleanSupplier fieldCentricSup;

    //public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier fieldCentricSup) {        
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        
        
        //this.robotCentricSup = robotCentricSup;
        this.fieldCentricSup = fieldCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if (VisionSubsystem.seeCube == true && VisionSubsystem.autoTurn == true) {
            LightingSubsystem.flash();
        }

        if (VisionSubsystem.autoTurn == true && VisionSubsystem.XDist != 0.0) {
            RobotContainer.controllerRumble(2.0);
            rotationVal = VisionSubsystem.XDist;
        } else {
            RobotContainer.controllerRumble(0.0);
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        }

        //System.out.println("trans: " + translationVal);
        //System.out.println("rot: " + rotationVal);
        //System.out.println("Field centric: " + fieldCentricSup.getAsBoolean());

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            //!robotCentricSup.getAsBoolean(), 
            fieldCentricSup.getAsBoolean(), 
            true
        );
    }
}