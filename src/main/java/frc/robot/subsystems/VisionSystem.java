// See: https://docs.photonvision.org/en/latest/docs/simulation/simulation.html
package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSystem extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera("cameraName");
  private final VisionSystemSim visionSim = new VisionSystemSim("main");
  private final Pose2dSupplier getSimPose;

  @FunctionalInterface
  public interface Pose2dSupplier {
    Pose2d getPose2d();
  }

  public VisionSystem(Pose2dSupplier getSimPose) {
    this.getSimPose = getSimPose;

    // Setup simulated camera properties
    SimCameraProperties props = new SimCameraProperties();
    props.setCalibError(0.25, 0.08);
    props.setFPS(20.0);
    props.setAvgLatencyMs(35.0);
    props.setLatencyStdDevMs(5.0);

    // Setup simulated camera
    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, props);
    // Draw field wireframe in simulated camera view
    cameraSim.enableDrawWireframe(true);

    // Add simulated camera to vision sim
    visionSim.addCamera(cameraSim, Constants.Transforms.robotToCamera);

    // Add AprilTags to vision sim
    try {
      AprilTagFieldLayout tagLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      visionSim.addAprilTags(tagLayout);
    } catch (IOException e) {
      System.err.println(e);
    }
  }

  @Override
  public void simulationPeriodic() {
    // Update the vision system with the simulated robot pose
    visionSim.update(getSimPose.getPose2d());
  }
}
