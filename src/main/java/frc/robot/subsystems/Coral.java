package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {

//Define Neo
SparkMax coralNeo = new SparkMax(1, MotorType.kBrushless);

  //Creates a new Coral.
    private final ShuffleboardTab coralTab = Shuffleboard.getTab("Coral");
    
  public Coral() {
    System.out.println("+++++ Starting Coral Constructor +++++");
    System.out.println("----- Ending Coral Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

