package frc.robot.sim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/** Simulate a differential drive robot with provided joystick or random inputs. */
public class RobotSim {

	private static final double MAX_VEL = 9.5;

	private static final double JITTER_RATE_SEC = 0.5;

	private final DifferentialDrivetrainSim drivetrainSim;
	private final Timer timer = new Timer();

	private String name;
	private DoubleSupplier leftDriveSupplier;
	private DoubleSupplier rightDriveSupplier;
	private Field2d field;

	private Pose2d currentPose = new Pose2d();
	private double inputLeft;
	private double inputRight;

	public RobotSim(
			String name, DoubleSupplier leftDrive, DoubleSupplier rightDrive, Field2d field) {
		this.name = name;
		this.leftDriveSupplier = leftDrive;
		this.rightDriveSupplier = rightDrive;
		this.field = field;

		drivetrainSim =
				DifferentialDrivetrainSim.createKitbotSim(
						DifferentialDrivetrainSim.KitbotMotor.kDoubleFalcon500PerSide,
						DifferentialDrivetrainSim.KitbotGearing.k8p45,
						DifferentialDrivetrainSim.KitbotWheelSize.kSixInch,
						VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

		timer.start();
		field.getObject(name).setPose(0.0, 0.0, Rotation2d.fromDegrees(0.0));
	}

	public RobotSim(String name, Field2d field) {
		this(name, Math::random, Math::random, field);
	}

	private void stayInfield() {
		double y = currentPose.getY();
		double x = currentPose.getX();

		if (y <= 1.0 || x <= 1.0) {
			inputRight = 5;
		}

		if (y >= 7.5 || x >= 15.0) {
			inputLeft = 5;
		}

		// Teleport to middle of field if drifted way too far away.
		if (y <= 0.0 || y >= 8.2 || x <= 0.0 || x >= 15.9)
			drivetrainSim.setPose(new Pose2d(8.0, 3.3, Rotation2d.fromDegrees(0.0)));
	}

	private void updateDrivetrainState() {
		currentPose = drivetrainSim.getPose();
		drivetrainSim.update(Constants.LOOP_TIME);
		field.getObject(name).setPose(currentPose);

		stayInfield();

		inputLeft = leftDriveSupplier.getAsDouble() * MAX_VEL * 12.0;
		inputRight = rightDriveSupplier.getAsDouble() * MAX_VEL * 12.0;
	}

	private void updateRobotInputs() {
		drivetrainSim.setInputs(inputLeft, inputRight);
	}

	public void updateWithJoystick() {
		updateDrivetrainState();
		updateRobotInputs();
	}

	public void updateWithRandom() {
		updateDrivetrainState();
		if (timer.advanceIfElapsed(JITTER_RATE_SEC)) {
			updateRobotInputs();
		}
	}
}
