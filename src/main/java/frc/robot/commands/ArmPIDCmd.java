// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;;

public class ArmPIDCmd extends Command {
  private final ArmSubsystem armSubsystem;
  private final PIDController pidController;
  
  /** Creates a new ArmPIDCmd. */
  public ArmPIDCmd(ArmSubsystem armSubsystem, double setpoint) {
    this.armSubsystem = armSubsystem;
    this.pidController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    pidController.setSetpoint(setpoint);
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Arm PID running");
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(armSubsystem.getDistance());
    armSubsystem.set(speed);
    System.out.println(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.set(0.0);
    System.out.println("end PID cmd");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
