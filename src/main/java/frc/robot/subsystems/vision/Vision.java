package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  public static record CameraConfig(String name, Transform3d robotToCam) {}

  public static record PoseEstimate(EstimatedRobotPose estimatedPose, Matrix<N3, N1> standardDev) {}

  private final VisionIO[] ios;
  private final VisionIOInputsAutoLogged[] inputs;
  private final CameraConfig[] configs;

  public Vision(CameraConfig... configs) {
    ios = new VisionIO[configs.length];
    inputs = new VisionIOInputsAutoLogged[configs.length];
    this.configs = configs;

    for (int i = 0; i < configs.length; i++) {
      switch (Constants.currentMode) {
        case REAL:
          ios[i] = new VisionIOReal(configs[i]);
          break;
        case SIM:
          ios[i] = new VisionIOSim();
          break;
        case REPLAY:
          ios[i] = new VisionIO() {};
          break;
        default:
          ios[i] = new VisionIO() {};
          break;
      }

      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  public PoseEstimate[] getEstimatedGlobalPoses() {
    List<PoseEstimate> estimates = new ArrayList<>();
    List<Pose3d> detectedTagPoses = new ArrayList<>();
    for (int i = 0; i < ios.length; i++) {
      ios[i].updateInputs(inputs[i]);

      // adding detected tags to list to be logged
      for (PhotonTrackedTarget tag : inputs[i].result.getTargets()) {
        VisionConstants.TAG_LAYOUT
            .getTagPose(tag.getFiducialId())
            .ifPresent(
                (tagPose) ->
                    detectedTagPoses.add(tagPose)); // if there's a pose, add it to the list
      }

      boolean addedPose = false;
      if (inputs[i].estimateIsPresent) {
        // don't use if estimate is outside the field
        if (!(inputs[i].estimatedPose.getX() > 0.0
            && inputs[i].estimatedPose.getX() <= Constants.kFieldLengthMeters
            && inputs[i].estimatedPose.getY() > 0.0
            && inputs[i].estimatedPose.getY() <= Constants.kFieldWidthMeters)) continue;
        // don't use if estimate is too high, or too tilted
        if (Math.abs(inputs[i].estimatedPose.getZ()) > VisionConstants.MAX_HEIGHT) continue;
        if (Math.abs(inputs[i].estimatedPose.getRotation().getX()) > VisionConstants.MAX_ANGLE)
          continue;
        if (Math.abs(inputs[i].estimatedPose.getRotation().getY()) > VisionConstants.MAX_ANGLE)
          continue;

        Matrix<N3, N1> stddevs =
            getEstimationStdDevs(inputs[i].estimatedPose.toPose2d(), inputs[i].result);

        addedPose = true;
        Logger.recordOutput(
            "/" + configs[i].name + "/Raw Vision",
            new Pose2d[] {inputs[i].estimatedPose.toPose2d()});
        Logger.recordOutput(
            "/" + configs[i].name + "/Vision Timestamp", inputs[i].timestampSeconds);
        Logger.recordOutput(
            "/" + configs[i].name + "/Vision Std Dev",
            new double[] {stddevs.get(0, 0), stddevs.get(1, 0), stddevs.get(2, 0)});

        estimates.add(
            new PoseEstimate(
                new EstimatedRobotPose(
                    inputs[i].estimatedPose,
                    inputs[i].timestampSeconds,
                    inputs[i].result.targets,
                    inputs[i].strategy),
                stddevs));
      }
      if (!addedPose) {
        Logger.recordOutput("/" + configs[i].name + "/Raw Vision", new Pose2d[] {});
      }
    }
    // logging detected tags
    Logger.recordOutput("Detected Tag Poses", detectedTagPoses.toArray(Pose3d[]::new));

    return estimates.toArray(PoseEstimate[]::new);
  }

  public Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, PhotonPipelineResult pipelineResult) {
    var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
    var targets = pipelineResult.getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      // if nonexistent tag, ignore
      var tagPose = VisionConstants.TAG_LAYOUT.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;

      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;

    avgDist /= numTags;

    // decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.MULTIPLE_TAG_STD_DEVS;
    // increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 7)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    if (VisionConstants.IGNORE_YAW) estStdDevs.set(2, 0, Double.MAX_VALUE);

    return estStdDevs;
  }
}
