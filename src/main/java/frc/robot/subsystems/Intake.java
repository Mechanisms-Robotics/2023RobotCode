package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

	private enum Position {
		GamePieceStation(-16387, 0.2, 0.7),
		Deploy(-37800, INTAKE_SPEED, OUTTAKE_SPEED),
		Retract(-3000, INTAKE_SPEED, OUTTAKE_SPEED);

		private final double intakeSpeed;
		private final double outtakeSpeed;
		private final double position;

		Position(double position, double intakeSpeed, double outtakeSpeed) {
			this.position = position;
			this.intakeSpeed = intakeSpeed;
			this.outtakeSpeed = outtakeSpeed;
		}
	}

	private static final double INTAKE_SPEED = 0.3; // 0.7 percent
	private static final double OUTTAKE_SPEED = -0.15;

	private static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
	private static final TalonFXConfiguration INTAKE_PIVOT_MOTOR_CONFIG =
			new TalonFXConfiguration();
	private static final TalonFXConfiguration LEFT_PIVOT_CONFIG;
	private static final TalonFXConfiguration RIGHT_PIVOT_CONFIG;

	//	private static final double RETRACTED_SENSOR_POSITION = -3000; // -5000
	//	private static final double DEPLOYED_SENSOR_POSITION = -37000; // -34500
	//	private static final double GAME_PIECE_STATION_POSITION = -16387;

	private static final double RIGHT_MOTOR_HORIZONTAL_POSITION = -37000; // -34359
	private static final double TICKS_PER_DEGREE = (2048.0 / 360.0) * 60.6814;

	private static final double MAX_GRAVITY_FF = 0.03; // 0.07

	static {
		final var intakeCurrentLimit = new SupplyCurrentLimitConfiguration();
		intakeCurrentLimit.currentLimit = 15; // Amps
		intakeCurrentLimit.triggerThresholdCurrent = 18; // Amps
		intakeCurrentLimit.triggerThresholdTime = 0.25; // sec
		intakeCurrentLimit.enable = true;

		INTAKE_MOTOR_CONFIG.supplyCurrLimit = intakeCurrentLimit;
		INTAKE_MOTOR_CONFIG.reverseSoftLimitEnable = false;
		INTAKE_MOTOR_CONFIG.forwardSoftLimitEnable = false;
		INTAKE_MOTOR_CONFIG.voltageCompSaturation = 10;

		// TODO: set amp limits for pivots

		INTAKE_PIVOT_MOTOR_CONFIG.motionAcceleration = 18000;
		INTAKE_PIVOT_MOTOR_CONFIG.motionCruiseVelocity = 18000;
		INTAKE_PIVOT_MOTOR_CONFIG.forwardSoftLimitEnable = true;
		INTAKE_PIVOT_MOTOR_CONFIG.reverseSoftLimitEnable = true;

		INTAKE_PIVOT_MOTOR_CONFIG.neutralDeadband = 0.001;

		LEFT_PIVOT_CONFIG = INTAKE_PIVOT_MOTOR_CONFIG;
		RIGHT_PIVOT_CONFIG = INTAKE_PIVOT_MOTOR_CONFIG;

		LEFT_PIVOT_CONFIG.forwardSoftLimitThreshold = 0;
		LEFT_PIVOT_CONFIG.reverseSoftLimitThreshold = -450000;

		RIGHT_PIVOT_CONFIG.forwardSoftLimitThreshold = 0;
		RIGHT_PIVOT_CONFIG.reverseSoftLimitThreshold = -450000;
	}

	private final WPI_TalonFX pivotRight = new WPI_TalonFX(20);
	private final WPI_TalonFX pivotLeft = new WPI_TalonFX(21);
	private final WPI_TalonFX spinRight = new WPI_TalonFX(22);
	private final WPI_TalonFX spinLeft = new WPI_TalonFX(23);

	private static final double kF = 0.0;
	private static final double kD = 0.0; // 0.04
	private static final double kP = 0.04; // 0.8

	private boolean isBrakeMode = false;
	private boolean zeroed = false;

	private Position currentMode = Position.Retract;

	public Intake() {
		pivotRight.configFactoryDefault();
		pivotLeft.configFactoryDefault();
		spinRight.configFactoryDefault();
		spinLeft.configFactoryDefault();

		// --- Spin right ---
		spinRight.configAllSettings(INTAKE_MOTOR_CONFIG, 255);
		spinRight.setInverted(TalonFXInvertType.Clockwise);
		spinRight.setNeutralMode(NeutralMode.Coast);

		spinRight.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
		spinRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
		spinRight.enableVoltageCompensation(true);
		// ------------

		// --- Spin left ---
		spinLeft.configAllSettings(INTAKE_MOTOR_CONFIG, 255);
		spinLeft.setInverted(InvertType.OpposeMaster);
		spinLeft.setNeutralMode(NeutralMode.Coast);

		spinLeft.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
		spinLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
		spinLeft.enableVoltageCompensation(true);
		// -------------

		// --- Pivot ---
		pivotRight.configAllSettings(RIGHT_PIVOT_CONFIG);
		pivotLeft.configAllSettings(LEFT_PIVOT_CONFIG);
		pivotRight.setNeutralMode(NeutralMode.Brake);
		pivotLeft.setNeutralMode(NeutralMode.Brake);

		pivotRight.setInverted(TalonFXInvertType.CounterClockwise);
		pivotLeft.setInverted(InvertType.OpposeMaster);

		pivotRight.config_kP(0, kP);
		pivotRight.config_kI(0, 0.0);
		pivotRight.config_kD(0, kD);
		pivotRight.config_kF(0, kF);
		//		intakePivotRight.config_IntegralZone(0, );

		pivotLeft.config_kP(0, kP);
		pivotLeft.config_kI(0, 0.0);
		pivotLeft.config_kD(0, kD);
		pivotLeft.config_kF(0, kF);

		pivotRight.selectProfileSlot(0, 0);
		pivotLeft.selectProfileSlot(0, 0);
	}

	private void setOpenLoop(double percentOutput) {
		if (!zeroed) {
			return;
		}

		spinRight.set(ControlMode.PercentOutput, percentOutput);
		spinLeft.follow(spinRight);
	}

	private void setClosedLoop(double position) {
		if (!zeroed) {
			return;
		}

		double radians =
				Math.toRadians(
						(pivotRight.getSelectedSensorPosition() - RIGHT_MOTOR_HORIZONTAL_POSITION)
								/ TICKS_PER_DEGREE);
		double cosRadians = Math.cos(radians);
		double demandFF = MAX_GRAVITY_FF * cosRadians;
		//		System.out.println(Math.toDegrees(radians));
		// NOTE: Use Position with ArbFF instead of MotionMagic
		// For Position, CTRE recommends setting kF to 0 and passing in ArbFF
		// CTRE says that kF needs to be calculated.
		// For MotionMagic: kF is multiplied by the runtime-calculated target and added to output.
		// NOTE: Update motor firmware
		/**
		 * The kF feature and arbitrary feed-forward feature are not the same. Arbitrary
		 * feed-forward is a supplemental term [-1,1] the robot application can provide to add to
		 * the output via the set() routine/VI.
		 */
		pivotRight.set(
				ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, demandFF);
		pivotLeft.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, demandFF);
		pivotLeft.follow(pivotRight);
	}

	public void intake() {
		setOpenLoop(currentMode.intakeSpeed);
	}

	public void outtake() {
		setOpenLoop(currentMode.outtakeSpeed);
	}

	@Override
	public void periodic() {
		double radians =
				Math.toRadians(
						(pivotRight.getSelectedSensorPosition() - RIGHT_MOTOR_HORIZONTAL_POSITION)
								/ TICKS_PER_DEGREE);
		SmartDashboard.putNumber("Right Pivot Pos", pivotRight.getSelectedSensorPosition());
		SmartDashboard.putNumber("Left Pivot Pos", pivotLeft.getSelectedSensorPosition());
		SmartDashboard.putNumber("Intake Angle", Math.toDegrees(radians));

		SmartDashboard.putNumber(
				"Intake Roller Rot/Sec", (spinRight.getSelectedSensorVelocity() / (2048 * 4)) * 10);
	}

	public void retract() {
		currentMode = Position.Retract;
		setClosedLoop(Position.Retract.position);
	}

	public void deploy() {
		currentMode = Position.Deploy;
		setClosedLoop(Position.Deploy.position);
	}

	public void setToGamePieceStation() {
		currentMode = Position.GamePieceStation;
		setClosedLoop(Position.GamePieceStation.position);
	}

	public void toggleBrakeMode() {
		isBrakeMode = !isBrakeMode;
		if (isBrakeMode) {
			pivotRight.setNeutralMode(NeutralMode.Brake);
			pivotLeft.setNeutralMode(NeutralMode.Brake);

		} else {
			pivotRight.setNeutralMode(NeutralMode.Coast);
			pivotLeft.setNeutralMode(NeutralMode.Coast);
		}
	}

	public void zeroEncoders() {
		if (zeroed) {
			return;
		}

		pivotLeft.setSelectedSensorPosition(0.0);
		pivotRight.setSelectedSensorPosition(0.0);
		zeroed = true;
		System.out.println("---Intake encoders zeroed---");
	}

	public void stop() {
		setOpenLoop(0.0);
	}

	public void stopPivot() {
		pivotLeft.set(ControlMode.PercentOutput, 0.0);
		pivotRight.set(ControlMode.PercentOutput, 0.0);
	}

	public static double ticksPer100ms(double degPerSec) {
		return (degPerSec / 10) * TICKS_PER_DEGREE;
	}
}
