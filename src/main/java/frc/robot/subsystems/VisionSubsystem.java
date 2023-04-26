package frc.robot.subsystems;

    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import edu.wpi.first.networktables.NetworkTable;
    import edu.wpi.first.networktables.NetworkTableEntry;
    import edu.wpi.first.networktables.NetworkTableInstance;
    import edu.wpi.first.wpilibj.Timer;

    
    public class VisionSubsystem extends SubsystemBase{
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        
        //Latencies below are necessary for pose estimation with WPIlib.
        NetworkTableEntry doesTargetExist = table.getEntry("tv"); 
        NetworkTableEntry pipelineLatency = table.getEntry("tl");
        NetworkTableEntry captureLatency = table.getEntry("cl");

        Swerve swerveDriveSubsystem;

        public VisionSubsystem(Swerve swerve) {
            swerveDriveSubsystem = swerve;
        }
    
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
    
        @Override
        public void periodic() {
        }
    }
        
    
    