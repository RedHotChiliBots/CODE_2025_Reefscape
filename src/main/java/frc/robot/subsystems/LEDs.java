// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {

  AddressableLED led = new AddressableLED(Constants.PWMId.kLEDs);

  AddressableLEDBuffer buffer = new AddressableLEDBuffer(60);

  LEDPattern red = LEDPattern.solid(Color.kRed);
  LEDPattern blue = LEDPattern.solid(Color.kBlue);
  LEDPattern yellow = LEDPattern.solid(Color.kYellow);

  Optional<Alliance> prevAlly = null;
  Optional<Alliance> currAlly = DriverStation.getAlliance();

  /**
   * Crea
   * tes a new LEDs.
   */
  public LEDs() {

    setAllianceColor().applyTo(buffer);

    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();

    // setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
    setDefaultCommand(runPattern(setAllianceColor()).withName("SetAllianceColor"));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setAlly();
    setAllianceColor().applyTo(buffer);
    led.setData(buffer);
  }

  public Command runPattern(LEDPattern pattern) {
    // Use the single input pattern to drive both sides
    return run(() -> pattern.applyTo(buffer));
  }

  public void setAlly() {
    if (DriverStation.getAlliance() != prevAlly) {
      currAlly = DriverStation.getAlliance();
      setAllianceColor();
    }
  }

  public LEDPattern setAllianceColor() {
    LEDPattern pattern = LEDPattern.solid(Color.kYellow);
    if (currAlly.isPresent()) {
      if (currAlly.get() == Alliance.Red) {
        pattern = LEDPattern.solid(Color.kFirstRed);
        //red.applyTo(buffer);
      }
      if (currAlly.get() == Alliance.Blue) {
        pattern = LEDPattern.solid(Color.kFirstBlue);
        //blue.applyTo(buffer);
      }
    } else {
      DriverStation.reportWarning("No Alloamce set", false);
    }

    return pattern;
  }

  public LEDPattern setClimb() {
    return LEDPattern.solid(Color.kCyan);
  }
}
