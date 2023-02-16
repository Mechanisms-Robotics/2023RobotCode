package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
	private static final double INTAKE_SPEED = 0.5; // percent

	private static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
	private static final TalonFXConfiguration INTAKE_RETRACT_MOTOR_CONFIG = new TalonFXConfiguration();

	private static final double DEPLOYED_SENSOR_POSITION = -40392;
	private static final double RETRACTED_SENSOR_POSITION = -200;

	private static final double RIGHT_MOTOR_HORIZONTAL_POSITION = -34359;
	private static final double TICKS_PER_DEGREE = (2048.0 / 360.0) * 60.6814;

	private static final double MAX_GRAVITY_FF = 0.05;

	static {
		final var intakeCurrentLimit = new SupplyCurrentLimitConfiguration();
		intakeCurrentLimit.currentLimit = 15; // Amps
		intakeCurrentLimit.triggerThresholdCurrent = 18; // Amps
		intakeCurrentLimit.triggerThresholdTime = 0.25; // sec
		intakeCurrentLimit.enable = true;

		INTAKE_MOTOR_CONFIG.supplyCurrLimit = intakeCurrentLimit;
		INTAKE_MOTOR_CONFIG.reverseSoftLimitEnable = false;
		INTAKE_MOTOR_CONFIG.forwardSoftLimitEnable = false;
		INTAKE_MOTOR_CONFIG.motionAcceleration = 2000;
		INTAKE_MOTOR_CONFIG.motionCruiseVelocity = 2000;
	}

	private final WPI_TalonFX intakeMotor = new WPI_TalonFX(20);
	private final WPI_TalonFX intakePivotLeft = new WPI_TalonFX(21);
	private final WPI_TalonFX intakePivotRight = new WPI_TalonFX(22);

	private static final double kP = 0.0; // 0.4
	private static final double kF = 0.0;

	private boolean isBrakeMode = false;

	public Intake() {
		intakeMotor.configAllSettings(INTAKE_MOTOR_CONFIG, 255);
		intakeMotor.setInverted(TalonFXInvertType.Clockwise);
		intakeMotor.setNeutralMode(NeutralMode.Coast);

		intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
		intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

		intakePivotRight.setNeutralMode(NeutralMode.Brake);
		intakePivotLeft.setNeutralMode(NeutralMode.Brake);

		intakePivotRight.setInverted(TalonFXInvertType.CounterClockwise);
		intakePivotLeft.setInverted(InvertType.OpposeMaster);

		intakePivotRight.config_kP(0, kP);
		intakePivotRight.config_kI(0, 0.0);
		intakePivotRight.config_kD(0, 0.0);
//		intakePivotRight.config_kF(0, kF);
//		intakePivotRight.config_IntegralZone(0, );



		intakePivotLeft.config_kP(0, kP);
		intakePivotLeft.config_kI(0, 0.0);
		intakePivotLeft.config_kD(0, 0.0);
//		intakePivotLeft.config_kF(0, kF);
	}


	private void setOpenLoop(double percentOutput) {
		intakeMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	private void setClosedLoop(double position) {
		double radians = Math.toRadians((intakePivotRight.getSelectedSensorPosition() - RIGHT_MOTOR_HORIZONTAL_POSITION) / TICKS_PER_DEGREE);
		double cosRadians = Math.cos(radians);
		double demandFF = MAX_GRAVITY_FF * cosRadians;
		System.out.println(Math.toDegrees(radians));
		intakePivotRight.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, demandFF);
		intakePivotLeft.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, -demandFF);
		intakePivotLeft.follow(intakePivotRight);
	}

	public void intake() {
		setOpenLoop(INTAKE_SPEED);
	}

	public void retract() {
//		intakePivotRight.set(ControlMode.PercentOutput, 0.07);
//		intakePivotLeft.set(ControlMode.PercentOutput, 0.07);
//		intakePivotLeft.follow(intakePivotRight);
		setClosedLoop(intakePivotRight.getSelectedSensorPosition());
	}

	public void deploy() {
		setClosedLoop(DEPLOYED_SENSOR_POSITION);
	}

	public void toggleBrakeMode() {
		isBrakeMode = !isBrakeMode;
		if (isBrakeMode) {
			intakePivotRight.setNeutralMode(NeutralMode.Brake);
			intakePivotLeft.setNeutralMode(NeutralMode.Brake);

		} else {
			intakePivotRight.setNeutralMode(NeutralMode.Coast);
			intakePivotLeft.setNeutralMode(NeutralMode.Coast);

		}
	}

	public void zeroEncoders() {
		intakePivotLeft.setSelectedSensorPosition(0.0);
		intakePivotRight.setSelectedSensorPosition(0.0);
		System.out.println("WEE WOPOO");
	}

	public void stop() {
		setOpenLoop(0.0);
	}
}
