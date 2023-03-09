package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public interface DriveController {
	Object getDriveMotor();

	void setReferenceVoltage(double voltage);

	double getStateVelocity();

	double getStateDistance();

	void setSimulatedMPS(double mps);

	void setNeutralMode(NeutralMode neutralMode);
}
