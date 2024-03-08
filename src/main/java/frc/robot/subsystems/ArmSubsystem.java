// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  
  public TalonFX left = new TalonFX(1);
  public TalonFX right = new TalonFX(0);


  public void up()
  {
    left.set(-0.1);
    right.set(0.1);
  }

  public void down()
  {
    left.set( 0.1);
    right.set(-0.1);
  }

  public void stop()
  {
    left.set( 0);
    right.set(0);
  }


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
}
