// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.GroundIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class GroundIntakeCommand extends Command {
  /** Creates a new ShootCmd. */
  private final GroundIntakeSubsystem m_subsystem;
    public GroundIntakeCommand(GroundIntakeSubsystem subsystem)
    {
      m_subsystem = subsystem;
      addRequirements(subsystem);
    }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    
          RobotContainer.motor.GroundIntake();
     
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.motor.GroundStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
