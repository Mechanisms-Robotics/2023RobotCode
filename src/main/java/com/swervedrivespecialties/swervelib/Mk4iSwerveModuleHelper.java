package com.swervedrivespecialties.swervelib;

import com.swervedrivespecialties.swervelib.api.ModuleConfigCallback;
import com.swervedrivespecialties.swervelib.ctre.*;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public final class Mk4iSwerveModuleHelper {
	private Mk4iSwerveModuleHelper() {}

	private static DriveControllerFactory<?, Integer> getFalcon500DriveFactory(
			Mk4ModuleConfiguration configuration, ModuleConfigCallback driveConfig) {
		return new Falcon500DriveControllerFactoryBuilder()
				.withVoltageCompensation(configuration.getNominalVoltage())
				.withCurrentLimit(configuration.getDriveCurrentLimit())
				.withMotorConfigFunction(driveConfig)
				.build();
	}

	private static SteerControllerFactory<
					?, Falcon500SteerConfiguration<CanCoderAbsoluteConfiguration>>
			getFalcon500SteerFactory(
					Mk4ModuleConfiguration configuration, ModuleConfigCallback steerConfig) {
		return new Falcon500SteerControllerFactoryBuilder()
				.withVoltageCompensation(configuration.getNominalVoltage())
				.withCurrentLimit(configuration.getSteerCurrentLimit())
				.withMotorConfigFunction(steerConfig)
				.build(new CanCoderFactoryBuilder().withReadingUpdatePeriod(100).build());
	}

	/**
	 * Creates a Mk4i swerve module that uses Falcon 500s for driving and steering. Module
	 * information is displayed in the specified ShuffleBoard container.
	 *
	 * @param container The container to display module information in.
	 * @param configuration Module configuration parameters to use.
	 * @param gearRatio The gearing configuration the module is in.
	 * @param driveMotorPort The CAN ID of the drive Falcon 500.
	 * @param steerMotorPort The CAN ID of the steer Falcon 500.
	 * @param steerEncoderPort The CAN ID of the steer CANCoder.
	 * @param steerOffset The offset of the CANCoder in radians.
	 * @return The configured swerve module.
	 */
	public static SwerveModule createFalcon500(
			ShuffleboardLayout container,
			Mk4ModuleConfiguration configuration,
			GearRatio gearRatio,
			int driveMotorPort,
			ModuleConfigCallback driveConfig,
			int steerMotorPort,
			ModuleConfigCallback steerConfig,
			int steerEncoderPort,
			double steerOffset) {
		return new SwerveModuleFactory<>(
						gearRatio.getConfiguration(),
						getFalcon500DriveFactory(configuration, driveConfig),
						getFalcon500SteerFactory(configuration, steerConfig))
				.create(
						container,
						driveMotorPort,
						new Falcon500SteerConfiguration<>(
								steerMotorPort,
								new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)));
	}

	/**
	 * Creates a Mk4i swerve module that uses Falcon 500s for driving and steering. Module
	 * information is displayed in the specified ShuffleBoard container.
	 *
	 * @param container The container to display module information in.
	 * @param gearRatio The gearing configuration the module is in.
	 * @param driveMotorPort The CAN ID of the drive Falcon 500.
	 * @param steerMotorPort The CAN ID of the steer Falcon 500.
	 * @param steerEncoderPort The CAN ID of the steer CANCoder.
	 * @param steerOffset The offset of the CANCoder in radians.
	 * @return The configured swerve module.
	 */
	public static SwerveModule createFalcon500(
			ShuffleboardLayout container,
			GearRatio gearRatio,
			int driveMotorPort,
			ModuleConfigCallback driveConfig,
			int steerMotorPort,
			ModuleConfigCallback steerConfig,
			int steerEncoderPort,
			double steerOffset) {
		return createFalcon500(
				container,
				new Mk4ModuleConfiguration(),
				gearRatio,
				driveMotorPort,
				driveConfig,
				steerMotorPort,
				steerConfig,
				steerEncoderPort,
				steerOffset);
	}

	/**
	 * Creates a Mk4i swerve module that uses Falcon 500s for driving and steering.
	 *
	 * @param configuration Module configuration parameters to use.
	 * @param gearRatio The gearing configuration the module is in.
	 * @param driveMotorPort The CAN ID of the drive Falcon 500.
	 * @param steerMotorPort The CAN ID of the steer Falcon 500.
	 * @param steerEncoderPort The CAN ID of the steer CANCoder.
	 * @param steerOffset The offset of the CANCoder in radians.
	 * @return The configured swerve module.
	 */
	public static SwerveModule createFalcon500(
			Mk4ModuleConfiguration configuration,
			GearRatio gearRatio,
			int driveMotorPort,
			ModuleConfigCallback driveConfig,
			int steerMotorPort,
			ModuleConfigCallback steerConfig,
			int steerEncoderPort,
			double steerOffset) {
		return new SwerveModuleFactory<>(
						gearRatio.getConfiguration(),
						getFalcon500DriveFactory(configuration, driveConfig),
						getFalcon500SteerFactory(configuration, steerConfig))
				.create(
						driveMotorPort,
						new Falcon500SteerConfiguration<>(
								steerMotorPort,
								new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)));
	}

	public enum GearRatio {
		L1(SdsModuleConfigurations.MK4I_L1),
		L2(SdsModuleConfigurations.MK4I_L2),
		L3(SdsModuleConfigurations.MK4I_L3);

		private final ModuleConfiguration configuration;

		GearRatio(ModuleConfiguration configuration) {
			this.configuration = configuration;
		}

		public ModuleConfiguration getConfiguration() {
			return configuration;
		}
	}
}
