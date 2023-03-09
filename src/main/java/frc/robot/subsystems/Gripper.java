package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

	// TODO: find positions
	private static final double OPEN_POSITION = -50;
	private static final double CLOSED_POSITION = -3765;
	private static final double CONE_POSITION = -3765;
	private static final double CUBE_POSITION = -3000;

	private static final double GRAB_TIME = 0.5;

	private static final TalonFXConfiguration GRIPPER_MOTOR_CONFIG = new TalonFXConfiguration();

	static {
		GRIPPER_MOTOR_CONFIG.motionAcceleration = 2000;
		GRIPPER_MOTOR_CONFIG.motionCruiseVelocity = 2000;
		GRIPPER_MOTOR_CONFIG.neutralDeadband = 0.001;

		GRIPPER_MOTOR_CONFIG.reverseSoftLimitEnable = true;
		GRIPPER_MOTOR_CONFIG.forwardSoftLimitEnable = true;
		// TODO: Find soft limits
		GRIPPER_MOTOR_CONFIG.reverseSoftLimitThreshold = -3765;
		GRIPPER_MOTOR_CONFIG.forwardSoftLimitThreshold = 0;
	}

	private final WPI_TalonFX gripperMotor = new WPI_TalonFX(60);

	private static final double kP = 0.2;
	private static final double kD = 0.0;

	private final Timer timer = new Timer();

	private boolean isOpen = true;

	public Gripper() {
		gripperMotor.configAllSettings(GRIPPER_MOTOR_CONFIG);

		gripperMotor.config_kP(0, kP);
		gripperMotor.config_kD(0, kD);
	}

	private void setOpenLoop(double percentOutput) {
		gripperMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	private void setClosedLoop(double position) {
		gripperMotor.set(ControlMode.Position, position);
	}

	public void toggle() {
		if (isOpen) {
			isOpen = false;
			close();
		} else {
			isOpen = true;
			open();
		}
	}

	public void open() {
		setClosedLoop(OPEN_POSITION);
	}

	public void close() {
		setClosedLoop(CLOSED_POSITION);
	}

	public void stop() {
		setOpenLoop(0.0);
	}

	public void cone() {
		setClosedLoop(CONE_POSITION);
	}

	public void cube() {
		setClosedLoop(CUBE_POSITION);
	}

	public Command grabCube() {
		return new FunctionalCommand(
				() -> {
					timer.start();
					open();
				},
				() -> {},
				(interrupted) -> cube(),
				() -> timer.hasElapsed(GRAB_TIME));
	}

	public void zeroEncoder() {
		gripperMotor.setSelectedSensorPosition(0.0);
	}
}
