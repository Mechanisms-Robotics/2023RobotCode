package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

	private enum Position {
		High(65000, EXTENDED_POSITION),
		Middle(57500, EXTENDED_POSITION / 2),
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
	private static final TalonFXConfiguration ARM_EXTENDER_MOTOR_CONFIG =
			new TalonFXConfiguration();

	private static final double STOWED_POSITION = 500;
	private static final double EXTENDED_POSITION = 17400;

	private static final double MAX_GRAVITY_FF = 0.079; // 0.3
	private static final double MAX_EXTENSION_GRAVITY_FF = -0.08;

	private static final double TICKS_PER_DEGREE = (2048.0 / 360.0) * 89.89;

	private static final double ARM_HORIZONTAL_POSITION = 57500;
	private static final double ALLOWABLE_PIVOT_ERROR = 1000;
	private static final double ALLOWABLE_EXTENSION_ERROR = 100;

	private static final double kP = 0.2; // 0.1
	private static final double kD = 0.0;
	private static final double kF = 0.01; // 0.01

	private static final double extenderKP = 1.4; // 0.8

	static {
		ARM_MOTOR_CONFIG.reverseSoftLimitEnable = true;
		ARM_MOTOR_CONFIG.forwardSoftLimitEnable = true;
		ARM_MOTOR_CONFIG.reverseSoftLimitThreshold = 16875;
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
		ARM_EXTENDER_MOTOR_CONFIG.motionAcceleration = 7000;
		ARM_EXTENDER_MOTOR_CONFIG.motionCurveStrength = 2;
	}

	private final WPI_TalonFX armMotorLeft = new WPI_TalonFX(50);
	private final WPI_TalonFX armMotorRight = new WPI_TalonFX(51);
	private final WPI_TalonFX extenderMotor = new WPI_TalonFX(52);

	private final ArmFeedforward armFeedforward = new ArmFeedforward(0.0, 0.86, 1.61, 0.05); // 0.86

	private ArmState armState = ArmState.Idle;

	private boolean zeroed = false;

	public Arm() {
		armMotorLeft.configFactoryDefault();
		armMotorLeft.configAllSettings(ARM_MOTOR_CONFIG);

		armMotorRight.configFactoryDefault();
		armMotorRight.configAllSettings(ARM_MOTOR_CONFIG);

		armMotorLeft.setNeutralMode(NeutralMode.Brake);
		armMotorRight.setNeutralMode(NeutralMode.Brake);

		armMotorLeft.config_kP(0, kP);
		armMotorLeft.config_kD(0, kD);
		armMotorLeft.config_kF(0, kF);

		armMotorRight.config_kP(0, kP);
		armMotorRight.config_kD(0, kD);
		armMotorRight.config_kF(0, kF);

		armMotorRight.setSensorPhase(true);

		armMotorLeft.configSupplyCurrentLimit(
				new SupplyCurrentLimitConfiguration(true, 15.0, 13.0, 1.0));

		armMotorRight.configSupplyCurrentLimit(
				new SupplyCurrentLimitConfiguration(true, 15.0, 13.0, 1.0));

		armMotorLeft.configNeutralDeadband(0.001);
		armMotorLeft.configAllowableClosedloopError(0, 0.0);

		armMotorRight.configNeutralDeadband(0.001);
		armMotorRight.configAllowableClosedloopError(0, 0.0);

		armMotorRight.setInverted(TalonFXInvertType.OpposeMaster);

		extenderMotor.configAllSettings(ARM_EXTENDER_MOTOR_CONFIG);

		extenderMotor.setNeutralMode(NeutralMode.Brake);

		extenderMotor.config_kP(0, extenderKP);
		extenderMotor.config_kI(0, 0.0);
		extenderMotor.config_kD(0, 0.0);

		extenderMotor.configNeutralDeadband(0.02);

		extenderMotor.configAllowableClosedloopError(0, 0.0); // 75

		extenderMotor.configSupplyCurrentLimit(
				new SupplyCurrentLimitConfiguration(true, 20.0, 18.0, 1.0));

		extenderMotor.configSetParameter(ParamEnum.eContinuousCurrentLimitAmps, 30, 30, 0);

		armMotorLeft.selectProfileSlot(0, 0);
		armMotorRight.selectProfileSlot(0, 0);
		extenderMotor.selectProfileSlot(0, 0);
	}

	public void setOpenLoop(double percentOutput) {
		if (!zeroed) {
			return;
		}

		armMotorLeft.set(ControlMode.PercentOutput, percentOutput);
		armMotorRight.follow(armMotorLeft);
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
							-(ARM_HORIZONTAL_POSITION - armMotorLeft.getSelectedSensorPosition())
									/ TICKS_PER_DEGREE);
			double cosRadians = Math.cos(radians);
			double demandFF = MAX_GRAVITY_FF * cosRadians;

			//			setOpenLoop(demandFF);
		}

		switch (armState) {
			case Idle:
				idle();
				break;
			case Retracting:
				retract();
				break;
			case Pivoting:
				pivot();
				break;
			case Deploying:
				deploy();
				break;
		}
	}

	public void setClosedLoop(double position) {
		if (!zeroed) {
			return;
		}

		armMotorLeft.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, 0.1);
		armMotorRight.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, 0.1);
		armMotorRight.follow(armMotorLeft);
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
		extenderMotor.set(
				ControlMode.MotionMagic,
				position,
				DemandType.ArbitraryFeedForward,
				MAX_EXTENSION_GRAVITY_FF);
	}

	private void setArm() {
		setExtensionClosedLoop(STOWED_POSITION);
	}

	public void horizontal() {
		setClosedLoop(ARM_HORIZONTAL_POSITION);
	}

	private void retract() {
		//		System.out.println("RETRACTING");

		if (armState == ArmState.Retracting) {
			if (Math.abs(extenderMotor.getSelectedSensorPosition() - STOWED_POSITION)
					<= ALLOWABLE_EXTENSION_ERROR) {
				pivot();
			}

			return;
		}

		armState = ArmState.Retracting;

		setExtensionClosedLoop(STOWED_POSITION);
	}

	private void pivot() {
		//		System.out.println("PIBOTING");

		if (armState == ArmState.Pivoting) {
			if (Math.abs(armMotorLeft.getSelectedSensorPosition() - desiredPosition.armPosition)
					<= ALLOWABLE_PIVOT_ERROR) {
				switch (desiredPosition) {
					case Stowed:
						setOpenLoop(0.0);
						break;
					case Low:
						setOpenLoop(0.03);
						break;
					case Middle:
						setOpenLoop(0.04);
						break;
					case High:
						setOpenLoop(0.04);
						break;
				}

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
		//		System.out.println("DEPLOTIG");

		if (armState == ArmState.Deploying) {
			if (Math.abs(extenderMotor.getSelectedSensorPosition() - desiredPosition.extendPosition)
					<= ALLOWABLE_EXTENSION_ERROR) {
				idle();
			}

			return;
		}

		armState = ArmState.Deploying;

		setExtensionClosedLoop(desiredPosition.extendPosition);
	}

	private void idle() {
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

		armMotorLeft.setSelectedSensorPosition(TICKS_PER_DEGREE * 33.0);
		armMotorRight.setSelectedSensorPosition(TICKS_PER_DEGREE * 33.0);
		extenderMotor.setSelectedSensorPosition(0.0);
		zeroed = true;
	}

	public void stop() {
		setOpenLoop(0.0);
		setExtensionOpenLoop(0.0);
	}
}
