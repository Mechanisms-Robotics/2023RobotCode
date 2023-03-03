package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends ProfiledPIDSubsystem {

	// TODO: find positions and extension positions
	private enum Position {
		High(57500, EXTENDED_POSITION),
		Middle(57500 - 8000, EXTENDED_POSITION / 2),
		Low(30000, STOWED_POSITION),
		Stowed(17500, STOWED_POSITION);

		private final double armPosition;
		private final double extendPosition;

		Position(double armPosition, double extendPosition) {
			this.armPosition = armPosition;
			this.extendPosition = extendPosition;
		}
	}

	private enum ArmState {
		Retracting,
		Pivoting,
		Deploying,
		Idle,
	}

	private Position desiredPosition = Position.Stowed;

	private static final TalonFXConfiguration ARM_MOTOR_CONFIG = new TalonFXConfiguration();
	private static final TalonFXConfiguration ARM_EXTENDER_MOTOR_CONFIG = new TalonFXConfiguration();

	private static final double STOWED_POSITION = 500;
	private static final double EXTENDED_POSITION = 16000;

	private static final double MAX_GRAVITY_FF = 0.079; // 0.3
	private static final double MAX_EXTENSION_GRAVITY_FF = -0.08;

	private static final double TICKS_PER_DEGREE = (2048.0 / 360.0) * 89.89;

	private static final double ARM_HORIZONTAL_POSITION = 57500;
	private static final double ALLOWABLE_PIVOT_ERROR = 1000;
	private static final double ALLOWABLE_EXTENSION_ERROR = 100;

	private static final double kP = 0.2; // 0.1
	private static final double kD = 0.0;
	private static final double kF = 0.0; // 0.01

	private static final double extenderKP = 1.4; // 0.8

	static {
		ARM_MOTOR_CONFIG.reverseSoftLimitEnable = true;
		ARM_MOTOR_CONFIG.forwardSoftLimitEnable = true;
		ARM_MOTOR_CONFIG.reverseSoftLimitThreshold = 16754;
		ARM_MOTOR_CONFIG.forwardSoftLimitThreshold = 75000;

    	ARM_MOTOR_CONFIG.motionCruiseVelocity = 30000;
		ARM_MOTOR_CONFIG.motionAcceleration = 30000;
		ARM_MOTOR_CONFIG.motionCurveStrength = 8;

		ARM_MOTOR_CONFIG.neutralDeadband = 0.001;

		ARM_EXTENDER_MOTOR_CONFIG.reverseSoftLimitEnable = true;
		ARM_EXTENDER_MOTOR_CONFIG.forwardSoftLimitEnable = true;
		ARM_EXTENDER_MOTOR_CONFIG.reverseSoftLimitThreshold = 0;
		ARM_EXTENDER_MOTOR_CONFIG.forwardSoftLimitThreshold = 17500;

		ARM_EXTENDER_MOTOR_CONFIG.motionCruiseVelocity = 10000;
		ARM_EXTENDER_MOTOR_CONFIG.motionAcceleration = 10000;
		ARM_EXTENDER_MOTOR_CONFIG.motionCurveStrength = 2;
	}

	private final WPI_TalonFX armMotor = new WPI_TalonFX(50);
	private final WPI_TalonFX extenderMotor = new WPI_TalonFX(51);

	private final ArmFeedforward armFeedforward = new ArmFeedforward(0.0, 0.86, 1.61, 0.05); // 0.86

	private boolean isArmAtPosition = true;
	private boolean isRetracted = true;

	private ArmState armState = ArmState.Idle;

	private boolean zeroed = false;

	public Arm() {
		super(new ProfiledPIDController(
				kP,
				0.0,
				kD,
				new TrapezoidProfile.Constraints(
					0.5 * Math.PI,
					0.5 * Math.PI
				)
		), Math.toRadians(STOWED_POSITION / TICKS_PER_DEGREE));
		armMotor.configFactoryDefault();
		armMotor.configAllSettings(ARM_MOTOR_CONFIG);

		armMotor.setNeutralMode(NeutralMode.Brake);

		armMotor.config_kP(0, kP);
		armMotor.config_kD(0, kD);
		armMotor.config_kF(0, kF);

//		armMotor.configAllowableClosedloopError(0, 1.0 * TICKS_PER_DEGREE);

		armMotor.configNeutralDeadband(0.001);
//		armMotor.configAllowableClosedloopError(0, 700.0);

		extenderMotor.configAllSettings(ARM_EXTENDER_MOTOR_CONFIG);

		extenderMotor.setNeutralMode(NeutralMode.Brake);

		extenderMotor.config_kP(0, extenderKP);
		extenderMotor.config_kI(0, 0.0);
		extenderMotor.config_kD(0, 0.0);

		extenderMotor.configNeutralDeadband(0.001);

		extenderMotor.configAllowableClosedloopError(0, 30.0); // 75

		armMotor.selectProfileSlot(0, 0);
		extenderMotor.selectProfileSlot(0, 0);

		setGoal(Math.toRadians(STOWED_POSITION / TICKS_PER_DEGREE));
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
		if (armState != ArmState.Pivoting) {
			double radians =
					Math.toRadians(
							-(ARM_HORIZONTAL_POSITION - armMotor.getSelectedSensorPosition())
									/ TICKS_PER_DEGREE);
			double cosRadians = Math.cos(radians);
			double demandFF =
					MAX_GRAVITY_FF * cosRadians;

//			setOpenLoop(demandFF);
		}

		switch (armState) {
			case Idle: idle(); break;
			case Retracting: retract(); break;
			case Pivoting: pivot(); break;
			case Deploying: deploy(); break;
		}
	}

	@Override
	protected void useOutput(double output, TrapezoidProfile.State setpoint) {
		double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);
		System.out.println(feedforward);
		armMotor.setVoltage(output + feedforward);
	}

	@Override
	protected double getMeasurement() {
		return Math.toRadians(armMotor.getSelectedSensorPosition() / TICKS_PER_DEGREE);
//		return armMotor.getSelectedSensorPosition();
	}


	public void setClosedLoop(double position) {
		if (!zeroed) {
			return;
		}

		isArmAtPosition = false;

//		double radians =
//				Math.toRadians(
//						-(ARM_HORIZONTAL_POSITION - armMotor.getSelectedSensorPosition())
//								/ TICKS_PER_DEGREE);
//		double cosRadians = Math.cos(radians);
//		double demandFF =
//				MAX_GRAVITY_FF * cosRadians;
//
//		double demandLOL = armFeedforward.calculate((ARM_HORIZONTAL_POSITION - position) / TICKS_PER_DEGREE,Math.toRadians(armMotor.getActiveTrajectoryVelocity() * 10) / TICKS_PER_DEGREE);
//		System.out.println(demandLOL);
//		armMotor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, demandLOL);

		setGoal(Math.toRadians(position / TICKS_PER_DEGREE));
		enable();
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
		isRetracted = false;
	}

	private void setArm() {
//		setClosedLoop(desiredPosition.armPosition);
		setExtensionClosedLoop(STOWED_POSITION);
		isRetracted = false;
	}

	public void horizontal() {
		setClosedLoop(ARM_HORIZONTAL_POSITION);
	}

	private void retract() {
		SmartDashboard.putString("ArmState", "Retracting");
//		System.out.println("RETRACTING");

		if (armState == ArmState.Retracting) {
			if (Math.abs(extenderMotor.getSelectedSensorPosition() - STOWED_POSITION) <= ALLOWABLE_EXTENSION_ERROR) {
				pivot();
			}

			return;
		}

		armState = ArmState.Retracting;

		setExtensionClosedLoop(STOWED_POSITION);
	}

	private void pivot() {
		SmartDashboard.putString("ArmState", "Pivoting");
//		System.out.println("PIBOTING");

		if (armState == ArmState.Pivoting) {
			if (Math.abs(armMotor.getSelectedSensorPosition() - desiredPosition.armPosition) <= ALLOWABLE_PIVOT_ERROR) {
//				disable();

				if (desiredPosition.extendPosition != STOWED_POSITION) {
					deploy();
				} else {
					idle();
				}
			}

			return;
		}

		armState = ArmState.Pivoting;

		setClosedLoop(desiredPosition.armPosition);
	}

	private void deploy() {
		SmartDashboard.putString("ArmState", "Deploy");
//		System.out.println("DEPLOTIG");

		if (armState == ArmState.Deploying) {
			if (Math.abs(extenderMotor.getSelectedSensorPosition() - desiredPosition.extendPosition) <= ALLOWABLE_EXTENSION_ERROR) {
				idle();
			}

			return;
		}

		armState = ArmState.Deploying;

		setExtensionClosedLoop(desiredPosition.extendPosition);
	}

	private void idle() {
		SmartDashboard.putString("ArmState", "Idling");
//		System.out.println("IDJLING");

		armState = ArmState.Idle;
	}

	public void stow() {
		desiredPosition = Position.Stowed;
		retract();
	}

	public void low() {
		desiredPosition = Position.Low;
		retract();
	}

	public void mid() {
		desiredPosition = Position.Middle;
		retract();
	}

	public void high() {
		desiredPosition = Position.High;
		retract();
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
