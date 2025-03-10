package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;

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
		lib.isMoving(j, sp);
		for (int i = 0; i < 10; i++) {
			System.out.println("MovingStop - j/sp: " + j + "/" + sp);
			assertFalse(lib.isMoving(j, sp));
		}
	}

	@Test
	public void testMovingUp() {
		double sp = 10.0;
		lib.isMoving(0, sp);
		for (int i = 0; i < 10; i++) {
			System.out.println("MovingUp - i/sp: " + i + "/" + sp);
			assertTrue(lib.isMoving(i, sp));
		}
	}

	@Test
	public void testMovingDn() {
		double sp = 0.0;
		double j = 10.0;
		lib.isMoving(j, sp);
		for (int i = 0; i < 10; i++) {
			j = 10.0 - i;
			System.out.println("MovingDn - j/sp: " + j + "/" + sp);
			assertTrue(lib.isMoving(j, sp));
		}
	}

	@Test
	public void testMovingWidget() {
		double sp = 0.0;
		lib.isMoving(0, sp);
		for (int i = 0; i < 10; i++) {
			System.out.println("MovingDn - i/sp: " + i + "/" + sp);
			// j = 10.0 - i;
			// if (onTarget()) {
			if (false) {
				// sbMovingWidget.withProperties(Map.of("colorWhenTrue", "black"));
				// sbMoving.setBoolean(true);
				System.out.println("Black");
			} else {
				if (lib.isMoving(i, sp)) {
					System.out.println("Yellow");
					// sbMovingWidget.withProperties(Map.of("colorWhenFalse", "yellow"));
					// sbMoving.setString(moving.toHexString());
				} else {
					System.out.println("Red");
					// sbMovingWidget.withProperties(Map.of("colorWhenFalse", "red"));
					// sbMoving.setString(offTgt.toHexString());
				}
				// sbMoving.setBoolean(false);
			}
		}
	}
}
