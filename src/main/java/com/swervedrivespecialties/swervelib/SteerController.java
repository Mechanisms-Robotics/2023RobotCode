package com.swervedrivespecialties.swervelib;

public interface SteerController {
	Object getSteerMotor();

	AbsoluteEncoder getSteerEncoder();

	double getReferenceAngle();

	void setReferenceAngle(double referenceAngleRadians);

	double getStateAngle();

	void setSimulatedAngle(double angle);

	void reset();
}
