package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public PhotonPipelineResult result = new PhotonPipelineResult();
    public Pose3d estimatedPose;
    public double timestampSeconds;
    public PoseStrategy strategy;
    public boolean estimateIsPresent = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
