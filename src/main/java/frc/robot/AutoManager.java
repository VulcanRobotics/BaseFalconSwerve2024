package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

 
public class AutoManager {
    SwerveAutoBuilder autoBuilder;

    HashMap<String, Command> eventMap = new HashMap<>();
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("thing", new PathConstraints(4, 3));

    public AutoManager(Swerve swerveDriveSubsystem) {
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
        Command fullAuto = autoBuilder.fullAuto(path);
        return fullAuto;
    } 
}
