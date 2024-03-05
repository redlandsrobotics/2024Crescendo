// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCmd extends Command {
  /** Creates a new ArmCmd. */
  private final ArmSubsystem m_subsystem;
  private Supplier<Double> speedFunction;
 

  public ArmCmd(ArmSubsystem subsystem, Supplier<Double> speedFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    this.speedFunction = speedFunction;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realTimeSpeed = speedFunction.get();

    if(realTimeSpeed<0.05 && realTimeSpeed>-0.05)
    {
      RobotContainer.arm.stop();
    }
    else if(realTimeSpeed<-0.05)
    {
      RobotContainer.arm.down();
    }
      else if(realTimeSpeed>0.05)
    {
      RobotContainer.arm.up();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
