package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

	private static final double ALLOWABLE_ERROR = 250;

	private static final double GRAB_TIME = 0.5;

	private static final TalonFXConfiguration GRIPPER_MOTOR_CONFIG = new TalonFXConfiguration();

	private boolean isZeroed = false;

	static {
		GRIPPER_MOTOR_CONFIG.motionAcceleration = 2000;
		GRIPPER_MOTOR_CONFIG.motionCruiseVelocity = 2000;
		GRIPPER_MOTOR_CONFIG.neutralDeadband = 0.001;

		GRIPPER_MOTOR_CONFIG.reverseSoftLimitEnable = true;
		GRIPPER_MOTOR_CONFIG.forwardSoftLimitEnable = true;
		// TODO: Find soft limits
		GRIPPER_MOTOR_CONFIG.reverseSoftLimitThreshold = -19384;
		GRIPPER_MOTOR_CONFIG.forwardSoftLimitThreshold = 0;

		GRIPPER_MOTOR_CONFIG.peakOutputReverse = -1.0;
		GRIPPER_MOTOR_CONFIG.peakOutputForward = 0.75;
	}

	private final WPI_TalonFX gripperMotor = new WPI_TalonFX(60);

	private static final double kP = 0.4;
	private static final double kD = 0.0;

	private final Timer timer = new Timer();

	private boolean isOpen = true;

	private double desiredPosition = 0.0;

	public Gripper() {
		gripperMotor.configAllSettings(GRIPPER_MOTOR_CONFIG);

		gripperMotor.config_kP(0, kP);
		gripperMotor.config_kD(0, kD);
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

	public boolean atPosition() {
		return Math.abs(desiredPosition - gripperMotor.getSelectedSensorPosition())
				<= ALLOWABLE_ERROR;
	}
}
