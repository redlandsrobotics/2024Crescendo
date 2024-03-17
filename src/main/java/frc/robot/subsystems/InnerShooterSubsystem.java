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

public class InnerShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  public CANSparkMax left = new CANSparkMax(12, MotorType.kBrushless);
  public CANSparkMax right = new CANSparkMax(11, MotorType.kBrushless);

  public InnerShooterSubsystem() {}



  public void LRshoot()
  {
    left.set(-0.75);
    right.set(0.75);
  }

  public void LRIntake()
  {
    left.set(0.3);
    right.set(-0.3);
  }

  public void LRstop()
  {
    left.set(0.0);
    right.set(0.0);
  }

  

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
