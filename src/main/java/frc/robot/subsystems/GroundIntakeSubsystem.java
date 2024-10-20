// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeSubsystem extends SubsystemBase {
  public TalonFX motor = new TalonFx(5);

  public GroundIntakeSubsystem() {}
  
  public void output()
  {
    motor.set(0.05);
  }

  public void GroundIntake()
  {
    motor.set(-0.3);
  }

  public void GroundStop()
  {
    motor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
