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
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

 
public class AutoManager {
    SwerveAutoBuilder autoBuilder;
    VisionSubsystem visionSubsystem;
    HashMap<String, Command> eventMap = new HashMap<>();

    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("thing", new PathConstraints(.5, .5));

    public AutoManager(Swerve swerveDriveSubsystem, VisionSubsystem visionSubsystem) {
        autoBuilder = new SwerveAutoBuilder(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::resetPose,
                new PIDConstants(5.6, 0.0, 0.001), //5.6
                new PIDConstants(4.3, 0.0, 0.001), //4.3
                swerveDriveSubsystem::setModuleStates,
                eventMap,
                true,
                swerveDriveSubsystem);
        this.visionSubsystem = visionSubsystem;
    }
    
    public Command getAuton() {
        return autoBuilder.fullAuto(path);
    } 
}
