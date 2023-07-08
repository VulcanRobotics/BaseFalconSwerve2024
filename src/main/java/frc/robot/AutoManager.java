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
import frc.robot.subsystems.Swerve;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

 
public class AutoManager {
    SwerveAutoBuilder autoBuilder;

    HashMap<String, Command> eventMap = new HashMap<>();



    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("test", new PathConstraints(1, 1));



    public AutoManager(Swerve swerveDriveSubsystem) {
        eventMap.put("Hello", new PrintCommand("Hello"));
        autoBuilder = new SwerveAutoBuilder(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::resetOdometry,
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
            new WaitCommand(3.0),
            new FollowPathWithEvents(autoBuilder.fullAuto(path.get(1)), path.get(1).getMarkers(), eventMap)
        );*/
    } 
}
