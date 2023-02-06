package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagTracker {

	private final PhotonCamera camera;
	private AprilTagFieldLayout fieldLayout;
	private final PhotonPoseEstimator photonPoseEstimator;

	public AprilTagTracker() {
		camera = new PhotonCamera(Constants.CAMERA_NAME);
		photonPoseEstimator =
				new PhotonPoseEstimator(
						fieldLayout,
						PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
						camera,
						Constants.ROBOT_TO_CAMERA);

		try {
			fieldLayout =
					AprilTagFieldLayout.loadFromResource(
							AprilTagFields.k2023ChargedUp.m_resourceFile);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	public void setEstimatorStrategy(PhotonPoseEstimator.PoseStrategy strategy) {
		this.photonPoseEstimator.setStrategy(strategy);
	}

	public Optional<PhotonTrackedTarget> getBestTarget() {
		final PhotonPipelineResult result = camera.getLatestResult();

		if (result.hasTargets()) {
			return Optional.of(result.getBestTarget());
		}

		return Optional.empty();
	}

	public Optional<List<PhotonTrackedTarget>> getTargets() {
		final PhotonPipelineResult result = camera.getLatestResult();

		if (result.hasTargets()) {
			return Optional.of(result.getTargets());
		}

		return Optional.empty();
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
		photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
		return photonPoseEstimator.update();
	}

	public PhotonCamera getCamera() {
		return this.camera;
	}
}
