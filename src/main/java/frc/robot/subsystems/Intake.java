package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
	private static final double INTAKE_SPEED = 0.7; // percent
	private static final double UNJAM_SPEED = -0.3;

	private static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
	private static final TalonFXConfiguration INTAKE_PIVOT_MOTOR_CONFIG =
			new TalonFXConfiguration();

	private static final double RETRACTED_SENSOR_POSITION = -5000; // -5000
	private static final double DEPLOYED_SENSOR_POSITION = -38500; // -40392

	private static final double RIGHT_MOTOR_HORIZONTAL_POSITION = -37000; // -34359
	private static final double TICKS_PER_DEGREE = (2048.0 / 360.0) * 60.6814;

	private static final double MAX_GRAVITY_FF = 0.07; // 0.07

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

		INTAKE_PIVOT_MOTOR_CONFIG.motionAcceleration = 4000;
		INTAKE_PIVOT_MOTOR_CONFIG.motionCruiseVelocity = 8000;
	}

	private final WPI_TalonFX intakeMotor = new WPI_TalonFX(20);
	private final WPI_TalonFX intakePivotLeft = new WPI_TalonFX(21);
	private final WPI_TalonFX intakePivotRight = new WPI_TalonFX(22);

	private static final double kF = 0.0;
	private static final double kD = 0.0; // 0.04
	private static final double kP = 0.8; // 0.4

	private boolean isBrakeMode = false;

	public Intake() {
		intakePivotRight.configFactoryDefault();
		intakePivotLeft.configFactoryDefault();

		intakeMotor.configAllSettings(INTAKE_MOTOR_CONFIG, 255);
		intakeMotor.setInverted(TalonFXInvertType.Clockwise);
		intakeMotor.setNeutralMode(NeutralMode.Coast);

		intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
		intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
		intakeMotor.enableVoltageCompensation(true);

		intakePivotRight.configAllSettings(INTAKE_PIVOT_MOTOR_CONFIG);
		intakePivotLeft.configAllSettings(INTAKE_PIVOT_MOTOR_CONFIG);
		intakePivotRight.setNeutralMode(NeutralMode.Brake);
		intakePivotLeft.setNeutralMode(NeutralMode.Brake);

		intakePivotRight.setInverted(TalonFXInvertType.CounterClockwise);
		intakePivotLeft.setInverted(InvertType.OpposeMaster);

		intakePivotRight.config_kP(0, kP);
		intakePivotRight.config_kI(0, 0.0);
		intakePivotRight.config_kD(0, kD);
		intakePivotRight.config_kF(0, kF);
		//		intakePivotRight.config_IntegralZone(0, );

		intakePivotLeft.config_kP(0, kP);
		intakePivotLeft.config_kI(0, 0.0);
		intakePivotLeft.config_kD(0, kD);
		intakePivotLeft.config_kF(0, kF);

		intakePivotRight.selectProfileSlot(0, 0);
		intakePivotLeft.selectProfileSlot(0, 0);
	}

	private void setOpenLoop(double percentOutput) {
		intakeMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	private void setClosedLoop(double position) {
		double radians =
				Math.toRadians(
						(intakePivotRight.getSelectedSensorPosition()
										- RIGHT_MOTOR_HORIZONTAL_POSITION)
								/ TICKS_PER_DEGREE);
		double cosRadians = Math.cos(radians);
		double demandFF = MAX_GRAVITY_FF * cosRadians;
		//		System.out.println(Math.toDegrees(radians));
		// NOTE: Use Position with ArbFF instead of MotionMagic
		// For Position, CTRE recommends setting kF to 0 and passing in ArbFF
		// CTRE says that kF needs to be calculated.
		// For MotionMagic: kF is multiplied by the runtime-calculated target and added to output.
		/**
		 * The kF feature and arbitrary feed-forward feature are not the same. Arbitrary
		 * feed-forward is a supplemental term [-1,1] the robot application can provide to add to
		 * the output via the set() routine/VI.
		 */
		intakePivotRight.set(ControlMode.MotionMagic, position);
		intakePivotLeft.set(ControlMode.MotionMagic, position);
		intakePivotLeft.follow(intakePivotRight);
	}

	public void intake() {
		setOpenLoop(INTAKE_SPEED);
	}

	public void unjam() {
		setOpenLoop(UNJAM_SPEED);
	}

	@Override
	public void periodic() {
		double radians =
				Math.toRadians(
						(intakePivotRight.getSelectedSensorPosition()
										- RIGHT_MOTOR_HORIZONTAL_POSITION)
								/ TICKS_PER_DEGREE);
		SmartDashboard.putNumber("Right Pivot Pos", intakePivotRight.getSelectedSensorPosition());
		SmartDashboard.putNumber("Left Pivot Pos", intakePivotLeft.getSelectedSensorPosition());
		SmartDashboard.putNumber("Intake Angle", Math.toDegrees(radians));

		SmartDashboard.putNumber("Intake Roller Rot/Sec", (intakeMotor.getSelectedSensorVelocity() / (2048 * 4)) * 10);
	}

	public void retract() {
		//		intakePivotRight.set(ControlMode.PercentOutput, 0.07);
		//		intakePivotLeft.set(ControlMode.PercentOutput, 0.07);
		//		intakePivotLeft.follow(intakePivotRight);
		setClosedLoop(RETRACTED_SENSOR_POSITION);
		System.out.println("RETRAC");
	}

	public void deploy() {
		setClosedLoop(DEPLOYED_SENSOR_POSITION);
		System.out.println("DEPLOY");
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
		System.out.println("---Intake encoders zeroed---");
	}

	public void stop() {
		setOpenLoop(0.0);
	}
}
