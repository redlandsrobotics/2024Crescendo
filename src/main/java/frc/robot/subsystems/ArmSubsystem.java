// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmSubsystem extends SubsystemBase {
  
  public TalonFX left = new TalonFX(1);
  public TalonFX right = new TalonFX(0);

  DutyCycleEncoder encoder = new DutyCycleEncoder(0);

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

  public void set(double speed)
  {
    left.set(-speed);
    right.set(speed);
  }

  public double getRotation() // returns absolute position relative to reset
  {
    if (encoder.isConnected())
    {
            return encoder.getAbsolutePosition();
    }
    else {
      return 0.0;
    }
  }

  public double getDistance()
  {
    return encoder.getDistance();
  }

  public void resetRotation() // sets current rotation to 0.0
  {
    encoder.reset();
  }

  public double GetPositionOffset() // get the position offset from when the encoder was reset
  {
    return encoder.getPositionOffset();
  }

  public void SetPositionOffset(double dub)
  {
      encoder.setPositionOffset(dub); // set the position offset to double

  }

   


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override 
  public void periodic()
  {
    SmartDashboard.putNumber("Arm Absolute Rotation", getRotation());
  }
}
