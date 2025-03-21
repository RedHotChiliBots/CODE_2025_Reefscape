package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.Ladder.LadderSP;


public class LadderTest {

	static Ladder ladder = new Ladder();

	@Test
	public void testMaxHeight() {
		double maxHeight = 93.5;
		for (LadderSP height : LadderSP.values()) {
			assertTrue(height.getValue() <= maxHeight);
		}
	}
}
