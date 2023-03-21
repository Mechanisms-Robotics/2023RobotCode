package frc.robot.states.arm;

import frc.robot.states.ArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Superstructure.Element;

public class Stowed extends ArmState {
	private static final double[][] ARM_POSITIONS = {
		{20000, -250}, // Pivot, Extension | Cube
		{17000, -3250} // Pivot, Extension | Cone
	};

	private static final double[][] GRIPPER_POSITIONS =
			new double[][] {
				{0, -12500}, // Open, Closed | Cube
				{0, -20000} // Open, Closed | Cone
			};

	public Stowed(Arm arm, Gripper gripper, Element element) {
		super(
				arm,
				gripper,
				ARM_POSITIONS[element.index][0],
				ARM_POSITIONS[element.index][1],
				GRIPPER_POSITIONS[element.index][0],
				GRIPPER_POSITIONS[element.index][1]);
	}
}
