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
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


    //ty krypton for some of this code
    public class VisionSubsystem extends SubsystemBase{

        //Initiallizes communication to the two limelights from here and sets up a table of values
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

        NetworkTableEntry doesTargetExistFront = front.getEntry("tv"); 
        NetworkTableEntry pipelineLatencyFront = front.getEntry("tl");
        NetworkTableEntry captureLatencyFront = front.getEntry("cl");

        ProfiledPIDController visionAdjustPID = new ProfiledPIDController(0.02, 0, 0, new TrapezoidProfile.Constraints(1, 1));
        ProfiledPIDController turnAdjustPID = new ProfiledPIDController(0.03, 0, 0, new TrapezoidProfile.Constraints(1, 1));

        boolean startClock;
        double startTime;
        double elapsedtime;

        //Botpose value
        public String kAllianceColorName = DriverStation.getAlliance().toString().strip().toLowerCase();


        public static double XDist = 0.0;
        public static double aSize = 0.0;
        public static boolean autoTurn = false;
        public static boolean kInAuton;

        private double translationStdDevCoefficient = 1.5;
        private double rotationStdDevCoefficient = 0.9;

        public static boolean seeCube = false;

        Optional<LLPoseEstimate> poseEstimateFront  = Optional.empty();

        Optional<LLPoseEstimate> poseEstimate  = Optional.empty();
        Swerve swerveDriveSubsystem;

        public VisionSubsystem(Swerve swerve) {
            this.swerveDriveSubsystem = swerve;
        }
        
        public static void setFrontLimeLight(boolean seekAprilTags) {
            if (seekAprilTags) {
                NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("pipeline").setNumber(1);
            } else {
                NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("pipeline").setNumber(0);
            }

            
        }


        public void timer(double time){ //This function is used to temporalily take control over rotation of robot for a split moment in order to turn towards cube
            
            
            if (startClock == true){
                startTime = System.currentTimeMillis();
                startClock = false;
                elapsedtime = 0.0;
            }
            else {
                elapsedtime = System.currentTimeMillis() - startTime;
            }
        
    
            if (elapsedtime < time) { 
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

        private void addVisionEstimateFront(LLPoseEstimate estimate, double id){
            if (id != -1) {
            var aprilTagPoseFront =
                Constants.FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose((int) id);
            if (aprilTagPoseFront.isPresent()) {
            double distanceFromPrimaryTagFront =
                aprilTagPoseFront.get().getTranslation().getDistance(estimate.estimatedPose.getTranslation());
            swerveDriveSubsystem.swerveDrivePoseEstimator.addVisionMeasurement(
                estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, calculateVisionStdDevs(distanceFromPrimaryTagFront));
            }
            }
        }
    
        public double visionAdjustX() { //This is the backbone of the autoturn feature, it helps calculate how far the distance the cube is from the center direction (crosshair of the front camera)
            double dist = f_tx.getInteger(0);
            return -dist/50;
        }

        public double getTargetSize() {
            return aSize;
        }

        public double getTargetDist() {
            return XDist;
        }

        public void timedFindIt() { //This starts the clock on the function "timer()" so that the robot turns towards the cube for a short time
            startClock = true;
        }

        public void findIt() { //This toggles the turn function permanently until the driver toggles it back off
            autoTurn = !autoTurn;
            
        }


        @Override
        public void periodic() {

            aSize = f_ta.getDouble(-1.0);
            XDist = f_tx.getDouble(0.0);

            double[] bluePose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]); 
            double[] redPose = table.getEntry("botpose_wpired").getDoubleArray(new double[6]); 

            double id = table.getEntry("tid").getDouble(0);
            double latency = Timer.getFPGATimestamp() - (pipelineLatency.getDouble(0.0) + captureLatency.getDouble(0.0))/1000;
            poseEstimate = (kAllianceColorName.startsWith("b")) ? getEstimate(bluePose, id, latency) : getEstimate(redPose, id, latency);

            double[] bluePoseFront = front.getEntry("botpose_wpiblue").getDoubleArray(new double[6]); 
            double[] redPoseFront = front.getEntry("botpose_wpired").getDoubleArray(new double[6]); 
            double idFront = front.getEntry("tid").getDouble(0);
            double latencyFront = Timer.getFPGATimestamp() - (pipelineLatencyFront.getDouble(0.0) + captureLatencyFront.getDouble(0.0))/1000;
            poseEstimateFront = (kAllianceColorName.startsWith("b")) ? getEstimate(bluePoseFront, idFront, latencyFront) : getEstimate(redPoseFront, idFront, latencyFront);

            if ((f_ta.getDouble(0) > 0.0)) { //This helps the static variable seeCube get uitilized throughout the classes to help if statements like in the CubeSeek command
                seeCube = true;
            } else {
                seeCube = false;
            }

            /*if (autoTurn && kInAuton == false) {
                VisionSubsystem.setFrontLimeLight(false);
            } else {
                VisionSubsystem.setFrontLimeLight(true);
            }*/

            XDist = visionAdjustX()*0.5;
            //timer(1000); //This doesn't do anything unless the startclock function becomes true
            
            SmartDashboard.putNumber("x", bluePose[0]);
            SmartDashboard.putNumber("id", id);
            //SmartDashboard.putNumber("latency", latency);

            if (poseEstimate.isPresent()) { //If you see an apriltag, try and get an estimated positional prediction relative to the field
                addVisionEstimate(poseEstimate.get(), id);
            } else if (poseEstimateFront.isPresent()) {
                addVisionEstimateFront(poseEstimateFront.get(), idFront);
            }


        
        }

    }
        
    
    