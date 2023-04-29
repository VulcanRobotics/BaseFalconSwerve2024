package frc.robot.subsystems;

    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
    import edu.wpi.first.networktables.NetworkTableEntry;
    import edu.wpi.first.networktables.NetworkTableInstance;

    import frc.lib.vision.LLPoseEstimate;;

    //oonga boonga robot bullshit
    public class VisionSubsystem extends SubsystemBase{

        public VisionSubsystem() {

        }
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        
        //Latencies below are necessary for pose estimation with WPIlib.
        NetworkTableEntry doesTargetExist = table.getEntry("tv"); 
        NetworkTableEntry pipelineLatency = table.getEntry("tl");
        NetworkTableEntry captureLatency = table.getEntry("cl");

        //Botpose values
        double[] bluePose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        double[] redPose = table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        NetworkTableEntry id = table.getEntry("tid");

        Optional<LLPoseEstimate> aprilTagEstimate = Optional.empty();


        Swerve swerveDriveSubsystem;

        public VisionSubsystem(Swerve swerve) {
            swerveDriveSubsystem = swerve;
        }
    
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double tl = pipelineLatency.getDouble(0.0);
        double cl = captureLatency.getDouble(0.0);
        double area = ta.getDouble(0.0);

        public Pose2d getVisionEstimate() {
            double xb = bluePose[0];
            double yb = bluePose[1]; //gets
            Rotation2d rot = new Rotation2d(bluePose[5]);
            return new Pose2d(xb, yb, rot);
        }
    
        @Override
        public void periodic() {
            swerveDriveSubsystem.swerveDrivePoseEstimator.addVisionMeasurement(getVisionEstimate(), (Timer.getFPGATimestamp()-(cl/1000)-(tl/1000)));
            SmartDashboard.putString("test", "Mogus");
        }

        
    }
        
    
    