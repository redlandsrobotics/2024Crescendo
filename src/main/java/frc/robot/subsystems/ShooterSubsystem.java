// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;
import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  public CANSparkFlex top = new CANSparkFlex(9, MotorType.kBrushless);
  public CANSparkFlex bottom = new CANSparkFlex(10, MotorType.kBrushless);
  public CANSparkMax left = new CANSparkMax(12, MotorType.kBrushless);
  public CANSparkMax right = new CANSparkMax(11, MotorType.kBrushless);

  public ShooterSubsystem() {}

  public void shoot()
  {
    top.set(0.75); // to be tuned later
    bottom.set(-0.75); // to be tuned later
  }

  public void LRshoot()
  {
    left.set(-0.75);
    right.set(0.75);
  }

  public void intake()
  {
    top.set(-0.3); // to be tuned later
    bottom.set(0.3); // to be tuned later
    left.set(0.3);
    right.set(-0.3);
  }

  public void stop()
  {
    top.set(0.0);
    bottom.set(0.0);
    left.set(0.0);
    right.set(0.0);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
