package frc.robot;

import javax.swing.JOptionPane;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
    private final XboxController driver = new XboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton calibrateEncoders = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton highPlace = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton midPlace = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton claw = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton drop = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton intake = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final VisionSubsystem s_Vision = new VisionSubsystem(s_Swerve);
    private final ArmSubsystem s_Arm = new ArmSubsystem(driver);
    private final PneumaticSubsystem s_Pneumatic = new PneumaticSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    public AutoManager autoManager = new AutoManager(s_Swerve, s_Arm, s_Pneumatic, s_Intake);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

       ///s_Arm.setDefaultCommand(new InstantCommand(() -> s_Arm.stopMovement(), s_Arm));

       



        // Configure the button bindings
        configureButtonBindings();
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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        calibrateEncoders.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        highPlace.onTrue(new HighPlace(s_Arm));
        midPlace.onTrue(new MidPlace(s_Arm));
        claw.onTrue(new InstantCommand(() -> s_Pneumatic.toggleClawState()));
        drop.onTrue(new InstantCommand(() -> s_Pneumatic.setIntakeState(true)));
        drop.onFalse(new InstantCommand(() -> s_Pneumatic.setIntakeState(false)));
        intake.whileTrue(new InstantCommand(() -> s_Intake.intake()));
        intake.whileFalse(new InstantCommand(() -> s_Intake.holdBall()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        return autoManager.getAuton();
        
    }
}
