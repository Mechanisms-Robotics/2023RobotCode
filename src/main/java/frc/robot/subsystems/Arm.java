package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Arm extends ProfiledPIDSubsystem {
	private enum Positions {
		High(1.97),
		Middle(1.69),
		Low(1.02),
		Stowed(0.60);

		private final double setpoint;

		Positions(double setpoint) {
			this.setpoint = setpoint;
		}
	}

	private static final TalonFXConfiguration ARM_MOTOR_CONFIG = new TalonFXConfiguration();
	private static final TalonFXConfiguration ARM_EXTENDER_MOTOR_CONFIG =
			new TalonFXConfiguration();

	private static final double TICKS_PER_DEGREE = (2048.0 / 360.0) * 89.89;
	private static final double TICKS_PER_RADIAN = (2048.0 / (2 * Math.PI)) * 89.89;

	private static final TrapezoidProfile.Constraints ARM_PROFILE = new TrapezoidProfile.Constraints(
			Math.PI,
			Math.PI
	);

	private static final double kP = 0.2;
	private static final double kI = 0.0;
	private static final double kD = 0.0;

	static {
		ARM_MOTOR_CONFIG.reverseSoftLimitEnable = true;
		ARM_MOTOR_CONFIG.forwardSoftLimitEnable = true;
		ARM_MOTOR_CONFIG.reverseSoftLimitThreshold = 16754;
		ARM_MOTOR_CONFIG.forwardSoftLimitThreshold = 75000;
	}

	private final WPI_TalonFX m_armMotor = new WPI_TalonFX(50);
	private final WPI_TalonFX m_extenderMotor = new WPI_TalonFX(51);

	private final ArmFeedforward m_armFeedforward = new ArmFeedforward(0.0, 0.86, 1.61, 0.05);

	private final SingleJointedArmSim m_armSim =
			new SingleJointedArmSim(
					DCMotor.getFalcon500(1),
					89.89,
					SingleJointedArmSim.estimateMOI(0.762, 6.8),
					0.762,
					0.571,
					2.560,
					true,
					VecBuilder.fill(2 * Math.PI / 2048));

	private final Encoder m_encoder = new Encoder(0, 1);
	private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

	private boolean m_zeroed = false;

	public Arm() {
		super(
				new ProfiledPIDController(
						kP, kI, kD, ARM_PROFILE),
				Positions.Stowed.setpoint);

		m_encoder.setDistancePerPulse(2 * Math.PI / 2048);

		m_armMotor.configFactoryDefault();
		m_armMotor.configAllSettings(ARM_MOTOR_CONFIG);

		m_armMotor.setNeutralMode(NeutralMode.Brake);

		m_extenderMotor.configFactoryDefault();
		m_extenderMotor.configAllSettings(ARM_EXTENDER_MOTOR_CONFIG);

		m_extenderMotor.setNeutralMode(NeutralMode.Brake);

		enable();
	}

	@Override
	public void periodic() {
		super.periodic();

		SmartDashboard.putNumber("Arm Rads", getMeasurement());
	}

	@Override
	public void simulationPeriodic() {
		super.simulationPeriodic();

		m_armSim.setInput(m_armMotor.get() * 12.0);
		m_armSim.update(0.020);

		m_encoderSim.setDistance(m_armSim.getAngleRads());

		SmartDashboard.putNumber("Arm Rads", getMeasurement());
	}

	public void setOpenLoop(double percentOutput) {
		if (!m_zeroed) {
			return;
		}

		m_armMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	@Override
	protected void useOutput(double output, TrapezoidProfile.State setpoint) {
		if (!m_zeroed) {
			return;
		}

		double feedforward = m_armFeedforward.calculate(setpoint.position, setpoint.velocity);

		SmartDashboard.putNumber("PPID Output", output);
		SmartDashboard.putNumber("FF Output", feedforward);

		m_armMotor.setVoltage((output * 12.0) + feedforward);
	}

	@Override
	protected double getMeasurement() {
		if (RobotBase.isReal()) {
			return m_armMotor.getSelectedSensorPosition() / TICKS_PER_RADIAN;
		} else {
			return m_encoderSim.getDistance();
		}
	}

	public void stow() {
		setGoal(Positions.Stowed.setpoint);
	}

	public void low() {
		setGoal(Positions.Low.setpoint);
	}

	public void mid() {
		setGoal(Positions.Middle.setpoint);
	}

	public void high() {
		setGoal(Positions.High.setpoint);
	}

	public void zeroEncoder() {
		if (m_zeroed) {
			return;
		}

		m_armMotor.setSelectedSensorPosition(TICKS_PER_DEGREE * 33.0);
		m_extenderMotor.setSelectedSensorPosition(0.0);
		m_zeroed = true;
	}

	public void stop() {
		setOpenLoop(0.0);
	}
}
