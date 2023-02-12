package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.sim.impulse.*;
import frc.robot.sim.impulse.Polygon;
import java.util.stream.Collectors;

public class Rigidbody2dPhysicsSim {

	private static final double PHYSICS_TO_FIELD_COORD = 100;

	private final Field2d field;
	private final ImpulseScene scene;

	private final Body FLOOR;
	private final Body ROBOT_COLLIDER;

	private final CommandXboxController controller = new CommandXboxController(0);

	public Rigidbody2dPhysicsSim(Field2d field2d) {
		this.field = field2d;
		this.scene = new ImpulseScene(0.05f, 10);

		ROBOT_COLLIDER = scene.add(new Polygon(70.0f, 100.0f), 0, 0);
		FLOOR = scene.add(new Polygon(700.0f, 10.0f), 800, 75);
		FLOOR.setStatic();
		FLOOR.setOrient(0.0f);

		controller
				.leftBumper()
				.onTrue(
						new InstantCommand(
								() -> {
									scene.add(
											new Circle(10.0f),
											800 + ImpulseMath.random(0, 50),
											150);
								}));
	}

	public void update() {
		scene.step();
		field.getObject("Balls")
				.setPoses(
						scene.bodies.stream()
								.map(
										body ->
												new Pose2d(
														body.position.x / PHYSICS_TO_FIELD_COORD,
														body.position.y / PHYSICS_TO_FIELD_COORD,
														Rotation2d.fromRadians(body.orient)))
								.collect(Collectors.toList()));
		double rx = field.getObject("Robot").getPose().getX();
		double ry = field.getObject("Robot").getPose().getY();
		double rr = field.getObject("Robot").getPose().getRotation().getRadians();
		ROBOT_COLLIDER.position =
				new Vec2(
						(float) (rx * PHYSICS_TO_FIELD_COORD),
						(float) (ry * PHYSICS_TO_FIELD_COORD));
		ROBOT_COLLIDER.setOrient((float) rr);
	}
}
