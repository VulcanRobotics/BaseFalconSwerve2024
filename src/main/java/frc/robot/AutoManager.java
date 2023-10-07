package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

 
public class AutoManager {
    SwerveAutoBuilder autoBuilder;

    HashMap<String, Command> eventMap = new HashMap<>();



    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("3PieceLow", new PathConstraints(3.0, 4)); //3.5 before open lab




    public AutoManager(Swerve swerveDriveSubsystem, ArmSubsystem armSubsystem, PneumaticSubsystem pneumaticSubsystem, IntakeSubsystem intakeSubsystem, VisionSubsystem visionsubsystem, LightingSubsystem lightingSubsystem) {
        
        /* ALL COMMANDS FOR PATHPLANNER AUTON. The eventMaps below each assign a commmand a specific word for the PathPlanner software to recognize
            new SequentialCommandGroup: this trait for a command sets up multiple instances of commands and runs them one by one, each one waiting for the command to finish before going to the next one
            new InstantCommand: this common trait for a command helps apply a static and public command from another subsystme to be uitilized here. It doesn't have to be classified as a command, it can just be a function
            new (Some name like "CubeSeek"): This is a custom built command used to specifically have an initiallized, execute, and finished portion throughgout its instance. These commands can be accessed in the commands folder
         */
        eventMap.put("Hello", new PrintCommand("Hello")); //This is the basic structure of how eventMaps should go
        eventMap.put("HighPlace", new SequentialCommandGroup(new HighPlace(armSubsystem), new InstantCommand(() -> pneumaticSubsystem.toggleClawState())));
        eventMap.put("Claw", new InstantCommand(() -> pneumaticSubsystem.toggleClawState()));
        eventMap.put("QuickLook", new InstantCommand(() -> visionsubsystem.timedFindIt()));
        eventMap.put("LookMode", new InstantCommand(() -> visionsubsystem.findIt()));
        eventMap.put("ReturnOrigin", new OriginPlace(armSubsystem, false));
        eventMap.put("MidPlace", new SequentialCommandGroup(new MidPlace(armSubsystem), new InstantCommand(() -> pneumaticSubsystem.toggleClawState())));
        eventMap.put("ToggleIntake", new InstantCommand(() -> pneumaticSubsystem.toggleIntakeState()));
        eventMap.put("Intake", new InstantCommand(() -> intakeSubsystem.intake()));
        eventMap.put("Spit", new InstantCommand(() -> intakeSubsystem.forceSpit()));
        eventMap.put("CubeSeek", new CubeSeek(swerveDriveSubsystem));
        eventMap.put("CubeTransfer", new SequentialCommandGroup(new OriginPlace(armSubsystem, true), new InstantCommand(() -> intakeSubsystem.spit()), new WaitCommand(0.1), new InstantCommand(() -> pneumaticSubsystem.toggleClawState() )));
        autoBuilder = new SwerveAutoBuilder(
                
                /*This code below are the final (and most important parts) of the auton, the swerve drive assignment!
                  The general orientation and relative positioning are here*/
                //swerveDriveSubsystem::getPose, //This pose takes into account the Swerve Odometry, which is taking the wheel positions to calculate its place on the field
                swerveDriveSubsystem::getPose2, //This pose takes into account the Prediction Odometry, which is taking the camera inputs to calculate its position on the field
                //swerveDriveSubsystem::resetOdometry, //Resets the wheel odometry, should be paired with the getPose command
                swerveDriveSubsystem::resetEstimator, //Resets the Prediction odometry, should be paired with the getPose2 command
                new PIDConstants(5.6, 0.0, 0.001),
                new PIDConstants(4.3, 0.0, 0.001),
                swerveDriveSubsystem::setModuleStates,
                eventMap,
                true,  
                swerveDriveSubsystem);

        
    }
    
    public Command getAuton() {
        return autoBuilder.fullAuto(path);
        /*return new SequentialCommandGroup(
            new FollowPathWithEvents(autoBuilder.fullAuto(path.get(0)), path.get(0).getMarkers(), eventMap),
            //new WaitCommand(1.0),
            //eventMap.get("LookMode"),
            //new WaitCommand(1.0),
            new FollowPathWithEvents(autoBuilder.fullAuto(path.get(1)), path.get(1).getMarkers(), eventMap)
        );*/
    } 
}
