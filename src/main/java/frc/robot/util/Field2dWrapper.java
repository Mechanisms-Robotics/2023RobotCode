package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.sim.BoidsSim;
import frc.robot.sim.Rigidbody2dPhysicsSim;
import frc.robot.sim.RobotSim;

/** Wrapper for Field2d to add and separate various simulation capabilities. */
public class Field2dWrapper extends Field2d {

	private static final int BOIDS_N = 150;
	private static final double BOIDS_VISUAL_RANGE = 0.4;
	private static final double BOIDS_MARGIN = 0.1;
	private static final double BOIDS_TURN_FACTOR = 0.05;
	private static final double BOIDS_CENTERING_FACTOR = 0.005;
	private static final double BOIDS_MIN_DISTANCE = 0.3;
	private static final double BOIDS_AVOID_FACTOR = 0.005;
	private static final double BOIDS_MATCHING_FACTOR = 0.07;
	private static final double BOIDS_SPEED_LIMIT = 0.2;

	private final RobotSim robotSim;
	private final BoidsSim boidsSim;
	private final Rigidbody2dPhysicsSim physicsSim;
	private final AprilTagTracker.CameraSim cameraSim;

	private boolean robotSimActive = false;
	private boolean boidsSimActive = false;
	private boolean cameraSimActive = false;
	private boolean physicsSimActive = false;

	public Field2dWrapper() {
		robotSim = new RobotSim("Robot", this);
		boidsSim =
				new BoidsSim(
						this,
						BOIDS_N,
						BOIDS_VISUAL_RANGE,
						BOIDS_MARGIN,
						BOIDS_TURN_FACTOR,
						BOIDS_CENTERING_FACTOR,
						BOIDS_MIN_DISTANCE,
						BOIDS_AVOID_FACTOR,
						BOIDS_MATCHING_FACTOR,
						BOIDS_SPEED_LIMIT);
		cameraSim = new AprilTagTracker.CameraSim();
		physicsSim = new Rigidbody2dPhysicsSim(this);
	}

	public void updateSims(Pose2d robotPose) {
		if (robotSimActive) robotSim.updateWithRandom();

		if (boidsSimActive) boidsSim.update();

		if (cameraSimActive) {
			cameraSim.updateSimulation(robotPose);
			cameraSim.putInField(this);
		}

		if (physicsSimActive) {
			physicsSim.update();
		}
	}

	public RobotSim getRobotSim() {
		return this.robotSim;
	}

	public BoidsSim getBoidsSim() {
		return this.boidsSim;
	}

	public AprilTagTracker.CameraSim getCameraSim() {
		return cameraSim;
	}
}
