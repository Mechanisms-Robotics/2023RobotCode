package frc.robot.util;

public class RobotDetection {

	//	private static final String ROBOT_DETECTION_TABLE_NAME = "robotdetect";
	//	private static final double CAMERA_FOCAL_LENGTH = 500.0; // TODO: Find focal length
	//	private static final double WIDTH_OF_AVG_ROBOT = 30.0; // in
	//
	//	private PipelineResult currentResult;
	//	private List<Pair<Pose2d, Pose2d>> estimatedRobotMotion; // Paired positional and velocity
	// data
	//	private WPI_Pigeon2 gyro;
	//
	//	static class ContourData {
	//		public ContourData(double area, double cx, double cy, double w, double h, double solidity) {
	//			this.area = area;
	//			this.cx = cx;
	//			this.cy = cy;
	//			this.w = w;
	//			this.h = h;
	//			this.solidity = solidity;
	//		}
	//
	//		double area;
	//		double cx;
	//		double cy;
	//		double w;
	//		double h;
	//		double solidity;
	//	}
	//
	//	static class PipelineResult {
	//		public PipelineResult(ContourData[] contours, long timestampSeconds) {
	//			this.contours = contours;
	//			this.timestampSeconds = timestampSeconds;
	//		}
	//
	//		ContourData[] contours;
	//		long timestampSeconds;
	//	}
	//
	//	public RobotDetection(WPI_Pigeon2 gyro) {
	//		this.gyro = gyro;
	//
	//		this.estimatedRobotMotion = new ArrayList<>();
	//	}
	//
	//	private long getVisionTimestamp(String key) {
	//		return NetworkTableInstance.getDefault()
	//				.getTable(ROBOT_DETECTION_TABLE_NAME)
	//				.getValue(key)
	//				.getServerTime();
	//	}
	//
	//	private double[] getVisionContourData(String key) {
	//		return NetworkTableInstance.getDefault()
	//				.getTable(ROBOT_DETECTION_TABLE_NAME)
	//				.getValue(key)
	//				.getDoubleArray();
	//	}
	//
	//	public void update() {
	//		double[] area = getVisionContourData("area");
	//		double[] cx = getVisionContourData("cx");
	//		double[] cy = getVisionContourData("cy");
	//		double[] w = getVisionContourData("w");
	//		double[] h = getVisionContourData("h");
	//		double[] solidity = getVisionContourData("solidity");
	//
	//		ContourData[] data = new ContourData[area.length];
	//
	//		for (int i = 0; i < area.length; i++) {
	//			data[i] = new ContourData(area[i], cx[i], cy[i], w[i], h[i], solidity[i]);
	//		}
	//
	//		currentResult = new PipelineResult(data, getVisionTimestamp("area"));
	//
	//		estimatedRobotMotion.addAll(
	//				estimateOtherRobotPoses(WIDTH_OF_AVG_ROBOT, CAMERA_FOCAL_LENGTH, w).stream()
	//						.map(pose -> new Pair<Pose2d, Pose2d>(pose,
	// estimateOtherRobotVelocities().stream().map()))
	//						.collect(Collectors.toList()));
	//
	//
	//	}
	//
	//	//	private List<ContourData> filterContours(List<ContourData> contourData) {
	//	//
	//	//	}
	//
	//	private Pose2d estimateOtherRobotPose(double widthPx) {
	//		double cameraToRobotDistance = (WIDTH_OF_AVG_ROBOT * CAMERA_FOCAL_LENGTH) / widthPx;
	//
	//
	//	}
	//
	//	private Pose2d estimateOtherRobotVelocity(Pose2d prevPose, Pose2d nextPose) {
	//
	//	}
}
