package com.swervedrivespecialties.swervelib;

public class GearRatios {
	public enum GearRatio {
		L1(SdsModuleConfigurations.MK4I_L1),
		L2(SdsModuleConfigurations.MK4I_L2),
		L3(SdsModuleConfigurations.MK4I_L3);

		private final MechanicalConfiguration configuration;

		GearRatio(MechanicalConfiguration configuration) {
			this.configuration = configuration;
		}

		public MechanicalConfiguration getConfiguration() {
			return configuration;
		}
	}
}
