package frc.robot.states.arm;

import frc.robot.states.ArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Superstructure.Element;

public class Scoring extends ArmState {
	private static final double[][][] ARM_POSITIONS = {
		{
			{35000, -750}, //   Low  | Cube
			{50000, -3750}, //  Mid  | Cube
			{60000, -17875}, // High | Cube
		},
		{
			{35000, -750}, //   Low  | Cone
			{62500, -3500}, //  Mid  | Cone
			{68750, -16500}, // High | Cone
		}
	};

	private static final double[][] GRIPPER_POSITIONS =
			new double[][] {
				{0, -12500}, // Open, Closed | Cube
				{0, -18000} // Open, Closed | Cone
			};

	public Scoring(Arm arm, Gripper gripper, Element element, int level) {
		super(
				arm,
				gripper,
				ARM_POSITIONS[element.index][level][0],
				ARM_POSITIONS[element.index][level][1],
				GRIPPER_POSITIONS[element.index][0],
				GRIPPER_POSITIONS[element.index][1]);
	}
}
