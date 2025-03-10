package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;

import org.junit.jupiter.api.Test;

import frc.robot.utils.Library;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;

public class LibraryTest {

	static Library lib = new Library();

	// @BeforeEach
	// public static void setup() {
	// 	// lib = new Library();
	// 	// lib.setPrevGap(0.0);
	// }

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
		double j = 0.0;
		lib.isMoving(j, sp);
		for (int i = 1; i < 10; i++) {
			j = i;
			System.out.println("MovingUp - j/sp: " + j + "/" + sp);
			assertTrue(lib.isMoving(j, sp));
		}
	}

	@Test
	public void testMovingDn() {
		double sp = 0.0;
		double j = 10.0;
		lib.isMoving(j, sp);
		for (int i = 1; i < 10; i++) {
			j = 10 - i;
			System.out.println("MovingDn - j/sp: " + j + "/" + sp);
			assertTrue(lib.isMoving(j, sp));
		}
	}

	@Test
	public void testMovingWidgetUp() {
		double sp = 10.0;
		double j = 0.0;
		lib.isMoving(j, sp);
		for (int i = 1; i < 10; i++) {
			j = i;
			System.out.println("MovingWidgetUp - j/sp: " + j + "/" + sp);
			// j = 10.0 - i;
			// if (onTarget()) {
			if (false) {
				// sbMovingWidget.withProperties(Map.of("colorWhenTrue", "black"));
				// sbMoving.setBoolean(true);
				System.out.println("Black");
			} else {
				if (lib.isMoving(j, sp)) {
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

	@Test
	public void testMovingWidgetDn() {
		double sp = 0.0;
		double j = 10.0;
		lib.isMoving(j, sp);
		for (int i = 1; i < 10; i++) {
			j = 10.0 - i;
			System.out.println("MovingWidgetDn - j/sp: " + j + "/" + sp);
			// j = 10.0 - i;
			// if (onTarget()) {
			if (false) {
				// sbMovingWidget.withProperties(Map.of("colorWhenTrue", "black"));
				// sbMoving.setBoolean(true);
				System.out.println("Black");
			} else {
				if (lib.isMoving(j, sp)) {
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
