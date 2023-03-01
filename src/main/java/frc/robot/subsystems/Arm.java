package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

	// TODO: find positions and extension positions
	private enum Position {
		High(65000, 500),
		Middle(57500, 500),
		Low(30000, 500),
		Stowed(17500, 500);

		private final double armPosition;
		private final double extendPosition;

		Position(double armPosition, double extendPosition) {
			this.armPosition = armPosition;
			this.extendPosition = extendPosition;
		}
	}

	private Position m_desiredPosition = Position.Stowed;

	private static final TalonFXConfiguration ARM_MOTOR_CONFIG = new TalonFXConfiguration();
	private static final TalonFXConfiguration ARM_EXTENDER_MOTOR_CONFIG = new TalonFXConfiguration();

	private static final double MAX_GRAVITY_FF = 0.07; // 0.3
	private static final double MAX_EXTENSION_GRAVITY_FF = -0.08;

	private static final double TICKS_PER_DEGREE = (2048.0 / 360.0) * 89.89;

	private static final double ARM_HORIZONTAL_POSITION = 57500;

	private static final double kP = 0.1;
	private static final double kD = 0.0;

	private static final double extenderKP = 1.4; // 0.8

	static {
		ARM_MOTOR_CONFIG.reverseSoftLimitEnable = true;
		ARM_MOTOR_CONFIG.forwardSoftLimitEnable = true;
		ARM_MOTOR_CONFIG.reverseSoftLimitThreshold = 16754;
		ARM_MOTOR_CONFIG.forwardSoftLimitThreshold = 67788;

    ARM_MOTOR_CONFIG.motionCruiseVelocity = 25000;
		ARM_MOTOR_CONFIG.motionAcceleration = 25000;
		ARM_MOTOR_CONFIG.motionCurveStrength = 8;

		ARM_MOTOR_CONFIG.neutralDeadband = 0.001;

		ARM_EXTENDER_MOTOR_CONFIG.reverseSoftLimitEnable = true;
		ARM_EXTENDER_MOTOR_CONFIG.forwardSoftLimitEnable = true;
		ARM_EXTENDER_MOTOR_CONFIG.reverseSoftLimitThreshold = 0;
		ARM_EXTENDER_MOTOR_CONFIG.forwardSoftLimitThreshold = 17500;

		ARM_EXTENDER_MOTOR_CONFIG.motionCruiseVelocity = 9000;
		ARM_EXTENDER_MOTOR_CONFIG.motionAcceleration = 9000;
		ARM_EXTENDER_MOTOR_CONFIG.motionCurveStrength = 2;
	}

	private final WPI_TalonFX armMotor = new WPI_TalonFX(50);
	private final WPI_TalonFX extenderMotor = new WPI_TalonFX(51);

	private boolean zeroed = false;

	public Arm() {
		armMotor.configAllSettings(ARM_MOTOR_CONFIG);

		armMotor.setNeutralMode(NeutralMode.Brake);

		armMotor.config_kP(0, kP);
		armMotor.config_kD(0, kD);

//		armMotor.configAllowableClosedloopError(0, 1.0 * TICKS_PER_DEGREE);

		armMotor.configNeutralDeadband(0.001);

		extenderMotor.configAllSettings(ARM_EXTENDER_MOTOR_CONFIG);

		extenderMotor.setNeutralMode(NeutralMode.Brake);

		extenderMotor.config_kP(0, extenderKP);
		extenderMotor.config_kI(0, 0.0);
		extenderMotor.config_kD(0, 0.0);

		extenderMotor.configNeutralDeadband(0.001);

		extenderMotor.configAllowableClosedloopError(0, 30.0); // 75
	}

	public void setOpenLoop(double percentOutput) {
		if (!zeroed) {
			return;
		}

		armMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	public void setExtensionOpenLoop(double percentOutput) {
		if (!zeroed) {
			return;
		}

		extenderMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	@Override
	public void periodic() {
//		setClosedLoop(m_desiredPosition.armPosition);
//		setExtensionClosedLoop(m_desiredPosition.extendPosition);

		double radians =
				Math.toRadians(
						-(ARM_HORIZONTAL_POSITION - armMotor.getSelectedSensorPosition())
								/ TICKS_PER_DEGREE);
		double cosRadians = Math.cos(radians);
		double demandFF =
				MAX_GRAVITY_FF * cosRadians;
    System.out.println("Deg: " + Math.toDegrees(radians) + "FF: " + demandFF);
    System.out.println("Sensor: " + armMotor.getSelectedSensorPosition() / TICKS_PER_DEGREE);
		SmartDashboard.putNumber("ext FF", demandFF);
	}

	public void setClosedLoop(double position) {
		if (!zeroed) {
			return;
		}

		double radians =
				Math.toRadians(
						-(ARM_HORIZONTAL_POSITION - armMotor.getSelectedSensorPosition())
								/ TICKS_PER_DEGREE);
		double cosRadians = Math.cos(radians);
		double demandFF =
				MAX_GRAVITY_FF * cosRadians;

		armMotor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, 0.07);
	}

	public void setExtensionClosedLoop(double position) {
		if (!zeroed) {
			return;
		}

//		double radians =
//				Math.toRadians(
//						(armMotor.getSelectedSensorPosition())
//								/ TICKS_PER_DEGREE);
//		double cosRadians = Math.cos(radians);
//		double demandFF =
//				MAX_EXTENSION_GRAVITY_FF * cosRadians;

//    SmartDashboard.putNumber("ext FF", demandFF);
//		setExtensionOpenLoop(-0.07);
		extenderMotor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, MAX_EXTENSION_GRAVITY_FF);
	}

	private void setArm() {
		setClosedLoop(m_desiredPosition.armPosition);
		setExtensionClosedLoop(m_desiredPosition.extendPosition);
	}

	public void horizontal() {
		setClosedLoop(ARM_HORIZONTAL_POSITION);
	}

	public void stow() {
		m_desiredPosition = Position.Stowed;
		setArm();
	}

	public void low() {
		m_desiredPosition = Position.Low;
		setArm();
	}

	public void mid() {
		m_desiredPosition = Position.Middle;
		setArm();
	}

	public void high() {
		m_desiredPosition = Position.High;
		setArm();
	}

	public void zeroEncoder() {
		if (zeroed) {
			return;
		}

		armMotor.setSelectedSensorPosition(TICKS_PER_DEGREE * 33.0);
		extenderMotor.setSelectedSensorPosition(0.0);
		zeroed = true;
	}

	public void stop() {
		setOpenLoop(0.0);
		setExtensionOpenLoop(0.0);
	}
}
