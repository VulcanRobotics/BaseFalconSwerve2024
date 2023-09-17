package frc.robot;

import javax.swing.JOptionPane;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
//import edu.wpi.first.wpilibj.XboxController.Axis;
//import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.AutoManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final Joystick operator = new Joystick(1);
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    //private final float spit = XboxController.Axis.kLeftTrigger.value;
    //private final float intake = XboxController.Axis.kRightTrigger.value;
    
    /* Driver Buttons */
    
    private final JoystickButton robotCentric = new JoystickButton(driver.getHID(), XboxController.Button.kBack.value);
    //private final JoystickButton calibrateEncoders = new JoystickButton(driver, XboxController.Button.kStart.value);
    
    //private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton highPlace = new JoystickButton(operator, 5);
    private final JoystickButton midPlace = new JoystickButton(operator, 4);
    private final JoystickButton originPlace = new JoystickButton(operator, 2);
    private final JoystickButton claw = new JoystickButton(operator, 1);
    /*private final JoystickButton drop = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton intake = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);
    private final JoystickButton spit = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final VisionSubsystem s_Vision = new VisionSubsystem(s_Swerve);
    private final ArmSubsystem s_Arm = new ArmSubsystem(driver.getHID());
    private final PneumaticSubsystem s_Pneumatic = new PneumaticSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    public AutoManager autoManager = new AutoManager(s_Swerve, s_Arm, s_Pneumatic, s_Intake);

    private Boolean isFieldRelative = true;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //System.out.println("Alliance: " + DriverStation.getAlliance() + "...................");

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis),
                //() -> driver.back().getAsBoolean(),
                () -> isFieldRelative
            )
        );

        s_Arm.setDefaultCommand(
            new JoystickMovement(s_Arm, operator)
        );

        // Configure the button bindings
        configureButtonBindings();

        //s_Arm.setDefaultCommand(new InstantCommand(() -> s_Arm.joystickMovement(operator.getX(), operator.getY(), operator.getZ()), s_Arm));
    }

    public Swerve getSwerve() {
        if (s_Swerve != null) {
            return s_Swerve;
        }
        else return null;
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        // Switch between field-relative and robot-centric driving
        robotCentric.onTrue(new InstantCommand(() -> {
          isFieldRelative = !isFieldRelative;
        }));

        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        // calibrateEncoders.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        highPlace.whileTrue(new HighPlace(s_Arm));
        //highPlace.onFalse(new JoystickMovement(s_Arm, operator));
        midPlace.whileTrue(new MidPlace(s_Arm));
        //midPlace.onFalse(new JoystickMovement(s_Arm, operator));
        originPlace.whileTrue(new OriginPlace(s_Arm));
        //originPlace.onFalse(new JoystickMovement(s_Arm, operator));
        claw.onTrue(new InstantCommand(() -> s_Pneumatic.toggleClawState()));
        /*drop.onTrue(new InstantCommand(() -> s_Pneumatic.setIntakeState(true)));
        drop.onFalse(new InstantCommand(() -> s_Pneumatic.setIntakeState(false)));
        intake.whileTrue(new InstantCommand(() -> s_Intake.intake()));
        intake.whileFalse(new InstantCommand(() -> s_Intake.holdBall()));
        spit.whileTrue(new InstantCommand(() -> s_Intake.spit()));*/

        
        driver.leftTrigger(0.5).onTrue(new InstantCommand(() -> s_Intake.spit()));
        driver.rightTrigger(0.5).onTrue(new InstantCommand(() -> s_Intake.intake()));
        driver.rightTrigger(0.5).onFalse(new InstantCommand(() -> s_Intake.holdBall()));
        driver.rightTrigger(0.5).onTrue(new InstantCommand(() -> s_Pneumatic.setIntakeState(true)));
        driver.rightTrigger(0.5).onFalse(new InstantCommand(() -> s_Pneumatic.setIntakeState(false)));
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driver.start().onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        
        
        /*if (spit > 0.5){
            new InstantCommand(() -> s_Intake.spit());
        }
        
        if (intake > 0.5){
            new InstantCommand(() -> s_Intake.intake());
            new InstantCommand(() -> s_Pneumatic.setIntakeState(true));
        } else if (spit < 0.5) {
            new InstantCommand(() -> s_Intake.holdBall());
            new InstantCommand(() -> s_Pneumatic.setIntakeState(false));
        }*/
        

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        //System.out.println("getAutonomousCommand()......................");
        return new SequentialCommandGroup(
            new InstantCommand(()->s_Swerve.zeroGyro()), 
            new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()),
            new WaitCommand(0.2),
            autoManager.getAuton());
        
    }
}
