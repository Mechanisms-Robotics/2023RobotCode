package com.swervedrivespecialties.swervelib.api;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.swervedrivespecialties.swervelib.SimpleFeedforwardConstants;
import java.util.function.Consumer;
import java.util.function.Function;

@FunctionalInterface
public interface ModuleConfigCallback
		extends Function<
				TalonFX, Function<TalonFXConfiguration, Consumer<SimpleFeedforwardConstants>>> {}
