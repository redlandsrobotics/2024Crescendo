// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.SwerveJoystickCmd;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private static AutoAlignCmd align = new AutoAlignCmd(swerveSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
		// joystick 1
		swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -Math.abs(joystick1.getRawAxis(OIConstants.kDriverXAxis)) * joystick1.getRawAxis(OIConstants.kDriverXAxis),// x and y speed switched up
      () -> -Math.abs(joystick1.getRawAxis(OIConstants.kDriverYAxis)) * joystick1.getRawAxis(OIConstants.kDriverYAxis),
      () -> Math.abs(joystick1.getRawAxis(OIConstants.kDriverRotAxis)) * joystick1.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !joystick1.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // joystick 1
    new JoystickButton(joystick1, 6).whenPressed(() -> swerveSubsystem.zeroHeading());

    // joystick 2
    // new JoystickButton(joystick2, 10).whenPressed(() -> swerveSubsystem.dReset()); // remove this after ONLY FOR AUTO TESTING!!!!
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
