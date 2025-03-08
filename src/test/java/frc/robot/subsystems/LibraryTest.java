package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.utils.Library;

import org.junit.jupiter.api.BeforeAll;

public class LibraryTest {

	Library lib = new Library();

	@BeforeAll
	public static void setup() {

	}
	
	@Test
	public void testMovingStop() {
		double sp = 0.0;
		double j = 5.0;
		for (int i = 0; i < 10; i++) {
			System.out.println("MovingStop - j/sp: " + j + "/" + sp);
			lib.isMoving(j, sp);
			assertFalse(lib.isMoving(j, sp));
		}
	}

	@Test
	public void testMovingUp() {
		double sp = 10.0;
		for (int i = 0; i < 10; i++) {
			System.out.println("MovingUp - i/sp: " + i + "/" + sp);
			// isMoving(i, sp);
			assertTrue(lib.isMoving(i, sp));
		}
	}

	@Test
	public void testMovingDn() {
		double sp = 0.0;
		double j = 0;
		for (int i = 0; i < 10; i++) {
			j = 10.0 - i;
			System.out.println("MovingDn - j/sp: " + j + "/" + sp);
			// isMoving(j, sp);
			assertTrue(lib.isMoving(j, sp));
		}
	}
}
