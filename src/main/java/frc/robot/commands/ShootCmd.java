// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Timer;
import java.util.TimerTask;
import java.util.function.Supplier;

public class ShootCmd extends Command {
  /** Creates a new ShootCmd. */
  
  private Supplier<Double> speedFunction;
  private Supplier<Double> speedFunction2;
  private final ShooterSubsystem m_subsystem;

  
  public ShootCmd(ShooterSubsystem subsystem, Supplier<Double> speedFunction, Supplier<Double> speedFunction2) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    this.speedFunction = speedFunction;
    this.speedFunction2 = speedFunction2;
    addRequirements(subsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    double realTimeSpeed = speedFunction.get();  
    double realTimeSpeed2 = speedFunction2.get();
    
    if (realTimeSpeed > 0.05)
    {
      RobotContainer.shooter.shoot();
    }
    else if(realTimeSpeed2 > 0.05)
    {
      RobotContainer.shooter.intake();
    }
    else
    {
      RobotContainer.shooter.stop();
    }
   
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
