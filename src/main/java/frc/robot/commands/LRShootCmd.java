// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.InnerShooterSubsystem;

import java.util.Timer;
import java.util.TimerTask;
import java.util.function.Supplier;

public class LRShootCmd extends Command {
  /** Creates a new ShootCmd. */
  private final InnerShooterSubsystem m_subsystem;
    public LRShootCmd(InnerShooterSubsystem subsystem)
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
    
          RobotContainer.innerShooter.LRshoot();
     
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
