package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.Vision.CameraConfig;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionIOReal implements VisionIO {
  public final PhotonCamera camera;
  public final PhotonPoseEstimator estimator;

  public VisionIOReal(CameraConfig config) {
    camera = new PhotonCamera(config.name());
    estimator =
        new PhotonPoseEstimator(
            VisionConstants.TAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            config.robotToCam());
    estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.result = camera.getLatestResult();
    var optionalEstimate = estimator.update(inputs.result);
    if (optionalEstimate.isPresent()) {
      inputs.estimatedPose = optionalEstimate.get().estimatedPose;
      inputs.timestampSeconds = optionalEstimate.get().timestampSeconds;
      inputs.strategy = optionalEstimate.get().strategy;
      inputs.estimateIsPresent = true;
    } else {
      inputs.estimateIsPresent = false;
    }
  }
}
