package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.RobotContainer;

public class AutoAlignCmd extends CommandBase{
    
    
   
   
    public AutoAlignCmd(SwerveSubsystem swerveSubsystem){
        addRequirements(swerveSubsystem);

    }

    PIDController yaw = new PIDController(0, 0, 0);
    PIDController distance = new PIDController(0, 0, 0);

    @Override
    public void initialize(){
        //System.out.println("debug");
    }


    @Override
    public void execute(){
        //example PID Controller code, accomplishes same thing as original code
            // double turnSpeed = yaw.calculate(RobotContainer.vision.see()[0], 0);
            // double xSpeed = yaw.calculate(RobotContainer.vision.see()[1], 17);

            // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0, turnSpeed);
            

            // SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            // RobotContainer.swerveSubsystem.setModuleStates(moduleStates);

        
        //BEGIN ORIGINAL CODE
        // double turningSpeed = RobotContainer.vision.see() > 0.5 ? -0.1 : 0.1;
        double turningSpeed;
        if (RobotContainer.vision.see()[0] > 1){
            turningSpeed = -0.1;
            
            if (RobotContainer.vision.see()[0] <1.4){
                turningSpeed = -0.07;
            } 
        }
        else if (RobotContainer.vision.see()[0] < -1) {
            turningSpeed = 0.1;
            if (RobotContainer.vision.see()[0] >-1.4){
                turningSpeed = 0.07;
            } 
        }
        else {
            turningSpeed = 0;
        }

        double xSpeed;
        if(RobotContainer.vision.see()[1] > 20){
            xSpeed = 0.1;
            if (RobotContainer.vision.see()[1] <21.5){
                xSpeed = 0.008;
            } 
        }
        else if (RobotContainer.vision.see()[1] < 16){
            xSpeed = -0.1;
            if (RobotContainer.vision.see()[1] > 14.5){
                xSpeed = -0.008;
            } 
        }
        else {
            xSpeed = 0;
        }

        if(!RobotContainer.vision.hasTargets()){
            xSpeed = 0;
            turningSpeed = 0;
        }


        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0, turningSpeed);
        

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        RobotContainer.swerveSubsystem.setModuleStates(moduleStates);
    }


    @Override
    public void end(boolean interrupted) {
        RobotContainer.swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}