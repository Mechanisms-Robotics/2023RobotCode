// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure.Element;
import frc.robot.util.AprilTagTracker;
import frc.robot.util.GoalTracker.TrackingMode;
import org.photonvision.common.hardware.VisionLEDMode;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;

	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();
		AprilTagTracker.getCamera().setLED(VisionLEDMode.kOff);

		m_robotContainer.m_swerve.resetModules();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		if (DriverStation.getAlliance() == Alliance.Blue) {
			AprilTagTracker.setAllianceSide(OriginPosition.kBlueAllianceWallRightSide);
		} else {
			AprilTagTracker.setAllianceSide(OriginPosition.kRedAllianceWallRightSide);
		}
	}

	/** This method is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		m_robotContainer.m_intake.stopPivot();
		m_robotContainer.m_arm.stop();
		m_robotContainer.m_gripper.stop();

		m_robotContainer.m_ledWrapper.turnOff();
	}

	@Override
	public void disabledPeriodic() {}

	/**
	 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}

		m_robotContainer.m_swerve.setNeutralMode(NeutralMode.Brake);
		m_robotContainer.m_swerve.zeroGyro();

		m_robotContainer.m_swerve.resetModules();

		m_robotContainer.m_intake.zeroEncoders();
		m_robotContainer.m_arm.init();
		m_robotContainer.m_gripper.zeroEncoder();

		m_robotContainer.m_superstructure.setAutoScore(false);
		m_robotContainer.m_intake.retract();
	}

	/** This method is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.

		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		m_robotContainer.m_swerve.setNeutralMode(NeutralMode.Coast);
		m_robotContainer.m_arm.setArm(0.0, 0.0);
		m_robotContainer.m_superstructure.idle();

		m_robotContainer.m_swerve.resetModules();

		//		m_robotContainer.m_superstructure.setAutoScore(true);
	}

	/** This method is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		if (m_robotContainer.m_superstructure.getAutoScore()
				&& m_robotContainer.m_goalTracker.getTrackingMode() == TrackingMode.BestGoal) {
			m_robotContainer.m_superstructure.setNode(
					m_robotContainer.m_goalTracker.getTargetNode()[0],
					m_robotContainer.m_goalTracker.getTargetNode()[1]);

			if (m_robotContainer.m_goalTracker.getTargetNode()[1] == 1
					|| m_robotContainer.m_goalTracker.getTargetNode()[1] == 4
					|| m_robotContainer.m_goalTracker.getTargetNode()[1] == 7) {
				m_robotContainer.m_superstructure.setElement(Element.Cube);
			} else {
				m_robotContainer.m_superstructure.setElement(Element.Cone);
			}
		}

		if (!m_robotContainer.m_swerve.getClimbMode()) {
			if (m_robotContainer.m_intake.isDeployed()) {
				m_robotContainer.m_swerve.setMaxVelocity(RobotBase.isReal() ? 3.0 : 2.0);
			} else {
				m_robotContainer.m_swerve.setMaxVelocity(RobotBase.isReal() ? 2.5 : 1.5);
			}
		}
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();

		m_robotContainer.m_swerve.resetModules();

		m_robotContainer.m_intake.zeroEncoders();
		m_robotContainer.m_arm.init();
		//		m_robotContainer.m_arm.zeroEncoder();
		m_robotContainer.m_gripper.zeroEncoder();
	}

	/** This method is called periodically during test mode. */
	@Override
	public void testPeriodic() {}

	/** This method is called once when the robot is first started up. */
	@Override
	public void simulationInit() {}

	/** This method is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {}
}
