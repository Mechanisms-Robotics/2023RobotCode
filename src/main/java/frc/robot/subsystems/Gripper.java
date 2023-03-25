package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

	private static final double ALLOWABLE_ERROR = 250;
	private static final double CLOSED_THRESHOLD = 1000;

	private static final TalonFXConfiguration GRIPPER_MOTOR_CONFIG = new TalonFXConfiguration();

	private boolean isZeroed = false;

	static {
		GRIPPER_MOTOR_CONFIG.motionAcceleration = 2000;
		GRIPPER_MOTOR_CONFIG.motionCruiseVelocity = 2000;
		GRIPPER_MOTOR_CONFIG.neutralDeadband = 0.001;

		GRIPPER_MOTOR_CONFIG.reverseSoftLimitEnable = true;
		GRIPPER_MOTOR_CONFIG.forwardSoftLimitEnable = true;

		GRIPPER_MOTOR_CONFIG.reverseSoftLimitThreshold = -19375;
		GRIPPER_MOTOR_CONFIG.forwardSoftLimitThreshold = 0;

		GRIPPER_MOTOR_CONFIG.peakOutputReverse = -1.0;
		GRIPPER_MOTOR_CONFIG.peakOutputForward = 0.75;
	}

	private final WPI_TalonFX gripperMotor = new WPI_TalonFX(60);

	private static final double kP = 0.6;
	private static final double kD = 0.0;

	private final Timer timer = new Timer();

	private boolean isOpen = true;

	private double desiredPosition = 0.0;

	public Gripper() {
		gripperMotor.configAllSettings(GRIPPER_MOTOR_CONFIG);

		gripperMotor.config_kP(0, kP);
		gripperMotor.config_kD(0, kD);

		gripperMotor.configAllowableClosedloopError(0, ALLOWABLE_ERROR);
	}

	public void setOpenLoop(double percentOutput) {
		if (!isZeroed) {
			return;
		}

		gripperMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	public void setClosedLoop(double position) {
		if (!isZeroed) {
			return;
		}

		this.desiredPosition = position;

		gripperMotor.set(ControlMode.Position, position);
	}

	public void stop() {
		setOpenLoop(0.0);
	}

	public void zeroEncoder() {
		if (isZeroed) {
			return;
		}

		gripperMotor.setSelectedSensorPosition(0.0);
		isZeroed = true;
	}

	public void init() {
		if (isZeroed) {
			return;
		}

		gripperMotor.setSelectedSensorPosition(-5.0);
		isZeroed = true;
	}

	public boolean atPosition() {
		return Math.abs(desiredPosition - gripperMotor.getSelectedSensorPosition())
				<= ALLOWABLE_ERROR;
	}

	public boolean isOpen() {
		return Math.abs(gripperMotor.getSelectedSensorPosition()) <= CLOSED_THRESHOLD;
	}

	public boolean isClosed() {
		return Math.abs(gripperMotor.getSelectedSensorPosition()) >= CLOSED_THRESHOLD;
	}
}
