package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class LLPoseEstimate {

        public final Pose3d estimatedPose;
        public final double timestampSeconds;
    
        public LLPoseEstimate(Pose3d estimatedPose, double timestampSeconds) {
            this.estimatedPose = estimatedPose;
            this.timestampSeconds = timestampSeconds;
        }
    } 
