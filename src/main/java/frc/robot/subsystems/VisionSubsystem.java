package frc.robot.subsystems;

    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import java.util.Optional;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.lib.vision.LLPoseEstimate;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

    //oonga boonga robot bullshit
    public class VisionSubsystem extends SubsystemBase{

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-rear");
        NetworkTable front = NetworkTableInstance.getDefault().getTable("limelight-front");


        //stating the data given from the tables that the limelightes write on
        NetworkTableEntry f_tx = front.getEntry("tx");
        NetworkTableEntry f_ty = front.getEntry("ty");
        NetworkTableEntry f_ta = front.getEntry("ta");
        NetworkTableEntry f_tv = front.getEntry("tv");


        //Latencies below are necessary for pose estimation with WPIlib.
        NetworkTableEntry doesTargetExist = table.getEntry("tv"); 
        NetworkTableEntry pipelineLatency = table.getEntry("tl");
        NetworkTableEntry captureLatency = table.getEntry("cl");

        ProfiledPIDController visionAdjustPID = new ProfiledPIDController(0.02, 0, 0, new TrapezoidProfile.Constraints(1, 1));
        ProfiledPIDController turnAdjustPID = new ProfiledPIDController(0.03, 0, 0, new TrapezoidProfile.Constraints(1, 1));

        boolean startClock;
        double startTime;
        double elapsedtime;

        //Botpose value

        public static double XDist = 0.0;
        public static boolean autoTurn = false;
        public static boolean kInAuton;

        private double translationStdDevCoefficient = 0.3;
        private double rotationStdDevCoefficient = 0.9;

        Optional<LLPoseEstimate> poseEstimate  = Optional.empty();
        Swerve swerveDriveSubsystem;

        public VisionSubsystem(Swerve swerve) {
            this.swerveDriveSubsystem = swerve;
            
        }
        
        public void timer(double time){
        
            
            if (startClock == true){
                startTime = System.currentTimeMillis();
                startClock = false;
                elapsedtime = 0.0;
            }
            else {
                elapsedtime = System.currentTimeMillis() - startTime;
            }
        
    
            if (elapsedtime < time) { //currently 0.5 seconds
                autoTurn = true;
            } else if (elapsedtime < time+1000) {
                autoTurn = false;
            }
       
        }

        //read values periodically
        private Matrix<N3, N1> calculateVisionStdDevs(double distance) {
            var translationStdDev = translationStdDevCoefficient * Math.pow(distance, 2);
            var rotationStdDev = rotationStdDevCoefficient * Math.pow(distance, 2);
    
            return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
        }

        public Optional<LLPoseEstimate> getEstimate(double[] positions, double id, double latency ) {
            double[] botposeArray = positions;

            if (id != -1) {
                
                Pose3d botPose = new Pose3d(
                                botposeArray[0],
                                botposeArray[1],
                                botposeArray[2],
                                new Rotation3d(
                                        Math.toRadians(botposeArray[3]),
                                        Math.toRadians(botposeArray[4]),
                                        Math.toRadians(botposeArray[5])));
    
                return Optional.of(new LLPoseEstimate(botPose, latency));
            } else {
                return Optional.empty();
            }
        }
        


        private void addVisionEstimate(LLPoseEstimate estimate, double id){
            if (id != -1) {
            var aprilTagPose =
                Constants.FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose((int) id);
            if (aprilTagPose.isPresent()) {
            double distanceFromPrimaryTag =
                aprilTagPose.get().getTranslation().getDistance(estimate.estimatedPose.getTranslation());
            swerveDriveSubsystem.swerveDrivePoseEstimator.addVisionMeasurement(
                estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, calculateVisionStdDevs(distanceFromPrimaryTag));
            }
            }
        }
    
        public double visionAdjustX() {
            //System.out.println("NOTICE ME");
            
            double dist = f_tx.getInteger(0);
            return -dist/50;
            //RobotContainer.seekRotationAxis = (int) dist;
            //, turnAdjustPID.calculate(dist, 0);
            
        }

        public void timedFindIt() {
            startClock = true;
        }

        public void findIt() {
            autoTurn = !autoTurn;
        }

        @Override
        public void periodic() {
            double[] bluePose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]); 
            double id = table.getEntry("tid").getDouble(0);
            double latency = Timer.getFPGATimestamp() - (pipelineLatency.getDouble(0.0) + captureLatency.getDouble(0.0))/1000;
            poseEstimate = getEstimate(bluePose, id, latency);


            XDist = visionAdjustX();
            timer(1000);
            
            SmartDashboard.putNumber("x", bluePose[0]);
            SmartDashboard.putNumber("id", id);
            SmartDashboard.putNumber("latency", latency);

            if (poseEstimate.isPresent()) {
                addVisionEstimate(poseEstimate.get(), id);
            }


        
        }

    }
        
    
    