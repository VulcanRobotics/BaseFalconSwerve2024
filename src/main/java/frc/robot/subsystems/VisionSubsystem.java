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
import frc.lib.vision.LLPoseEstimate;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


    //oonga boonga robot bullshit
    public class VisionSubsystem extends SubsystemBase{

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        //Latencies below are necessary for pose estimation with WPIlib.
        NetworkTableEntry doesTargetExist = table.getEntry("tv"); 
        NetworkTableEntry pipelineLatency = table.getEntry("tl");
        NetworkTableEntry captureLatency = table.getEntry("cl");

        //Botpose value

        private double translationStdDevCoefficient = Constants.VisionConstants.kTranslationStdDevCoefficient;
        private double rotationStdDevCoefficient = Constants.VisionConstants.kRotationStdDevCoefficient;

        Optional<LLPoseEstimate> poseEstimate  = Optional.empty();
        Swerve swerveDriveSubsystem;

        public VisionSubsystem(Swerve swerve) {
            this.swerveDriveSubsystem = swerve;
            
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
    
        @Override
        public void periodic() {

            double[] bluePose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]); 
            double[] redPose = table.getEntry("botpose_wpired").getDoubleArray(new double[6]); 

            double id = table.getEntry("tid").getDouble(0);
            double latency = Timer.getFPGATimestamp() - (pipelineLatency.getDouble(0.0) + captureLatency.getDouble(0.0))/1000;

            poseEstimate = (Constants.GameConstants.kAllianceColorName.startsWith("b")) ? getEstimate(bluePose, id, latency) : getEstimate(redPose, id, latency);

            if (poseEstimate.isPresent()) {
                addVisionEstimate(poseEstimate.get(), id);
            }

        }

    }
        
    
    