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



    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("test", new PathConstraints(1.0, 4)); //3.5 before open lab




    public AutoManager(Swerve swerveDriveSubsystem, ArmSubsystem armSubsystem, PneumaticSubsystem pneumaticSubsystem, IntakeSubsystem intakeSubsystem, VisionSubsystem visionsubsystem, LightingSubsystem lightingSubsystem) {
        eventMap.put("Hello", new PrintCommand("Hello"));
        eventMap.put("HighPlace", new SequentialCommandGroup(new HighPlace(armSubsystem), new InstantCommand(() -> pneumaticSubsystem.toggleClawState())));
        eventMap.put("Claw", new InstantCommand(() -> pneumaticSubsystem.toggleClawState()));
        eventMap.put("QuickLook", new InstantCommand(() -> visionsubsystem.timedFindIt()));
        eventMap.put("LookMode", new InstantCommand(() -> visionsubsystem.findIt()));
        eventMap.put("OriginPlace", new OriginPlace(armSubsystem));
        eventMap.put("MidPlace", new SequentialCommandGroup(new MidPlace(armSubsystem), new InstantCommand(() -> pneumaticSubsystem.toggleClawState())));
        eventMap.put("ToggleIntake", new InstantCommand(() -> pneumaticSubsystem.toggleIntakeState()));
        eventMap.put("Intake", new InstantCommand(() -> intakeSubsystem.intake()));
        eventMap.put("CubeSeek", new CubeSeek(swerveDriveSubsystem));
        eventMap.put("CubeTransfer", new SequentialCommandGroup(new InstantCommand(() -> pneumaticSubsystem.toggleIntakeState()), new WaitCommand(1), new InstantCommand(() -> intakeSubsystem.spit()), new WaitCommand(0.1), new InstantCommand(() -> pneumaticSubsystem.toggleClawState()) ));
        autoBuilder = new SwerveAutoBuilder(
                swerveDriveSubsystem::getPose,
                //swerveDriveSubsystem::getPose2,
                swerveDriveSubsystem::resetOdometry,
                //swerveDriveSubsystem::resetEstimator,
                new PIDConstants(5.6, 0.0, 0.001),
                new PIDConstants(4.3, 0.0, 0.001),
                swerveDriveSubsystem::setModuleStates,
                eventMap,
                true,  
                swerveDriveSubsystem);

        
    }
    
    public Command getAuton() {
        //return autoBuilder.fullAuto(path);
        return new SequentialCommandGroup(
            new FollowPathWithEvents(autoBuilder.fullAuto(path.get(0)), path.get(0).getMarkers(), eventMap),
            new WaitCommand(1.0),
            eventMap.get("LookMode"),
            new WaitCommand(1.0),
            new FollowPathWithEvents(autoBuilder.fullAuto(path.get(1)), path.get(1).getMarkers(), eventMap)
        );
    } 
}
