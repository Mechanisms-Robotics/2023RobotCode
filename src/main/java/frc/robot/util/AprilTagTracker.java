package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Wrapper class for PhotonLib's AprilTag support */
public class AprilTagTracker {

	private static final PhotonCamera camera = new PhotonCamera(Constants.CAMERA_NAME);
	private static final AprilTagFieldLayout fieldLayout;
	private static final PhotonPoseEstimator photonPoseEstimator;

	static {
		try {
			fieldLayout =
					AprilTagFieldLayout.loadFromResource(
							AprilTagFields.k2023ChargedUp.m_resourceFile);

		} catch (IOException e) {
			throw new RuntimeException(e);
		}

		photonPoseEstimator =
				new PhotonPoseEstimator(
						fieldLayout,
						PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
						camera,
						Constants.ROBOT_TO_CAMERA);

		photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
	}

	public static void setEstimatorStrategy(PhotonPoseEstimator.PoseStrategy strategy) {
		photonPoseEstimator.setPrimaryStrategy(strategy);
	}

	public static void setAllianceSide(AprilTagFieldLayout.OriginPosition side) {
		fieldLayout.setOrigin(side);
	}

	public static PhotonTrackedTarget getBestTarget() {
		final PhotonPipelineResult result = camera.getLatestResult();

		if (result.hasTargets()) {
			return result.getBestTarget();
		}

		return null;
	}

	public static List<PhotonTrackedTarget> getTargets() {
		final PhotonPipelineResult result = camera.getLatestResult();

		if (result.hasTargets()) {
			return result.getTargets();
		}

		return null;
	}

	public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(
			Pose2d prevEstimatedRobotPose) {
		photonPoseEstimator.setLastPose(prevEstimatedRobotPose);
		photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
		return photonPoseEstimator.update();
	}

	public static PhotonCamera getCamera() {
		return camera;
	}

	public static AprilTagFieldLayout getFieldLayout() {
		return fieldLayout;
	}

	public static class CameraSim {
		SimVisionSystem visionSystemSim =
				new SimVisionSystem(
						Constants.CAMERA_NAME, 170.0, Constants.ROBOT_TO_CAMERA, 20, 640, 480, 10);

		Pose2d cameraPose = new Pose2d();

		public CameraSim() {
			visionSystemSim.addVisionTargets(fieldLayout);
		}

		public void updateSimulation(Pose2d robotPose) {
			this.cameraPose =
					robotPose.plus(
							new Transform2d(
									new Translation2d(
											Constants.ROBOT_TO_CAMERA.getX(),
											Constants.ROBOT_TO_CAMERA.getY()),
									Constants.ROBOT_TO_CAMERA.getRotation().toRotation2d()));
			visionSystemSim.processFrame(robotPose);
		}

		public void putInField(Field2d field) {
			List<Pose2d> tagPoses =
					fieldLayout.getTags().stream()
							.map(tag -> tag.pose.toPose2d())
							.collect(Collectors.toList());
			field.getObject("bCamera").setPose(cameraPose);
			field.getObject("cTags").setPoses(tagPoses);
		}
	}
}
