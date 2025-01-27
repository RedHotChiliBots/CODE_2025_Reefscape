// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class VariableChangeTrigger extends Trigger {

	private final BooleanSupplier condition;
	private boolean previousValue = false;

	public VariableChangeTrigger(BooleanSupplier condition) {
		super(condition);
		this.condition = condition;
	}

	// @Override
	// public boolean get() {
	// 	boolean currentValue = condition.get();
	// 	boolean changed = currentValue != previousValue;
	// 	previousValue = currentValue;
	// 	return changed;
	// }
}