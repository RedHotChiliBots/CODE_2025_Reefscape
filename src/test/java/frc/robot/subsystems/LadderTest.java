package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants;
import frc.robot.subsystems.Ladder.LadderSP;


public class LadderTest {

	static Ladder ladder = new Ladder();

	@Test
	public void testMaxHeight() {
		for (LadderSP height : LadderSP.values()) {
			assertTrue(height.getValue() <= Constants.Ladder.kMaxLadderHeight);
		}
	}
}
