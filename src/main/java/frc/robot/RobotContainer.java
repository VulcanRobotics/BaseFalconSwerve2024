package frc.robot;

import javax.swing.JOptionPane;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
    private final static XboxController rumble = new XboxController(0);
    private final Joystick operator = new Joystick(1);
    /* Drive Controls */
    private final int seekRotationAxis = 0;
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value + seekRotationAxis;
    /* Operator Buttons */
    private final JoystickButton robotCentric = new JoystickButton(driver.getHID(), XboxController.Button.kBack.value);
    private final JoystickButton highPlace = new JoystickButton(operator, 5);
    private final JoystickButton midPlace = new JoystickButton(operator, 4);
    private final JoystickButton originPlace = new JoystickButton(operator, 2);
    private final JoystickButton claw = new JoystickButton(operator, 1);
    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    private final VisionSubsystem s_Vision = new VisionSubsystem(s_Swerve);
    private final ArmSubsystem s_Arm = new ArmSubsystem();
    private final PneumaticSubsystem s_Pneumatic = new PneumaticSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    private final LightingSubsystem s_Light = new LightingSubsystem();
    public AutoManager autoManager = new AutoManager(s_Swerve, s_Arm, s_Pneumatic, s_Intake, s_Vision, s_Light);

    private Boolean isFieldRelative = true;

    public static void controllerRumble(double amount) {
        rumble.setRumble(RumbleType.kBothRumble, amount);
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //System.out.println("Alliance: " + DriverStation.getAlliance() + "...................");

        s_Swerve.setDefaultCommand( //Unless overrided, this command takes in xbox inputs and controls the robot's swerve modules
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis),
                //() -> driver.back().getAsBoolean(),
                () -> isFieldRelative
            )
        );
        
        s_Arm.setDefaultCommand( //Unless overrided, this command takes in joystick inputs and controls the robot's arm
            new JoystickMovement(s_Arm, operator)
        );

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
        driver.leftTrigger(0.5).whileTrue(new InstantCommand(() -> s_Intake.forceSpit()));
        driver.leftBumper().onTrue(new CubeSeek(s_Swerve));
        driver.rightTrigger(0.5).onTrue(new InstantCommand(() -> s_Intake.intake()));
        driver.rightTrigger(0.5).onFalse(new InstantCommand(() -> s_Intake.holdBall()));
        driver.rightTrigger(0.5).onTrue(new InstantCommand(() -> s_Pneumatic.setIntakeState(true)));
        driver.rightTrigger(0.5).onFalse(new InstantCommand(() -> s_Pneumatic.setIntakeState(false)));
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        
        //driver.x().whileTrue(new DriveToPosition(s_Swerve, new Pose2d(1.65, 4.41, new Rotation2d(0.0))));
        driver.x().whileTrue(new SequentialCommandGroup(
            new DriveToPosition(s_Swerve, new Pose2d(1.65, 4.41, new Rotation2d(0.0))),
             new HighPlace(s_Arm),
             new InstantCommand(() -> s_Pneumatic.toggleClawState())));
        driver.start().onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));

        /* Operator Buttons */
        highPlace.whileTrue(new HighPlace(s_Arm));
        midPlace.whileTrue(new MidPlace(s_Arm));
        originPlace.whileTrue(new OriginPlace(s_Arm, false));
        claw.onTrue(new InstantCommand(() -> s_Pneumatic.toggleClawState()));


        // Switch between field-relative and robot-centric driving
        robotCentric.onTrue(new InstantCommand(() -> {
            isFieldRelative = !isFieldRelative;
            }));

        

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
