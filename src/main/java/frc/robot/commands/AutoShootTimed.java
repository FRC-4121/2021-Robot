// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Processor2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.NetworkTableQuerier;
import frc.robot.extraClasses.PIDControl;
import frc.robot.extraClasses.Ballistics;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.ShooterConstants.*;


/**
 * AutoShootTimed Command
 * 
 * This command repeatedly drives the robot between a shooting location
 * and a ball loading location.  At the shooting location, the robot shoots
 * three balls at the goal.  At the loading location, the robot
 * waits for three balls to be loaded into the processor before
 * moving back to the shooting location.
 */
public class AutoShootTimed extends CommandBase {

  // Declare class variables
  private final Drivetrain drivetrain;
  private final Shooter shooter;
  private final Processor2 processor;
  private final NetworkTableQuerier ntables;
  private Ballistics ballistics;

  private int robotMode;      // 1: Shooting, 2: Drive to Loading, 3: Loading Wait, 4: Drive to Shooting
  private int ballCount;

  private double driveDistance;
  private double driveDirection;
  private double stopTime;
  private double startTime;
  private double driveSpeedCorrection;
  private double currentGyroAngle;
  private double angleCorrection;
  private double angleDeadband;
  private double shooterSpeedCorrection;
  private double shotPossible;

  private double[] ballisticsData;

  private Timer timer;

  private PIDControl pidDriveAngle;
  private PIDControl pidDriveDistance;
  private PIDControl pidShooterSpeed;


  /** Default constructor */
  public AutoShootTimed(Drivetrain drive, Shooter shoot, Processor2 process, NetworkTableQuerier table, double distance, double deadband, double time) {

    // Set class variables
    drivetrain = drive;
    shooter = shoot;
    processor = process;
    ntables = table;
    driveDistance = distance;
    angleDeadband = deadband;
    stopTime = time;

    // Add subsystem requirements
    addRequirements(drivetrain, shooter, processor);

    // Create PID controllers
    pidDriveAngle = new PIDControl(kP_Turn, kI_Turn, kD_Turn);
    pidDriveDistance = new PIDControl(kP_Straight, kI_Straight, kD_Straight);
    pidShooterSpeed = new PIDControl(kP_Shoot, kI_Shoot, kD_Shoot);

    // Create the ballistics table
    ballistics = new Ballistics(98.25, 22.5, 5, 6380, 6, .227);

  }


  /** Initialize this command (only runs first time command is called) */
  @Override
  public void initialize() {

    // Initialize variables
    driveDirection = -1;
    driveSpeedCorrection = 0;
    ballCount = 0;

    // Initialize shooting mode
    robotMode = 1;

    // Start the timer and get the command start time
    timer = new Timer();
    timer.start();
    startTime = timer.get();
    
    // Zero gyro angle
    drivetrain.zeroGyro();

  }


  /** Main code to be executed every robot cycle */
  @Override
  public void execute() {

    // Determine mode and take action
    switch(robotMode) {

      // Shooting
      case 1:
        break;
      
      // Driving to load area
      case 2:
        break;
      
      // Waiting in load area
      case 3:
        break;
      
      // Driving to shooting location
      case 4:
        break;

    }

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Initialize finished flag
    boolean thereYet = false;

    // Get current time
    double time = timer.get();

    // Check for max time
    if (stopTime <= time - startTime) {

      // Set flag
      thereYet = true;

    } else {
    

    }

    // Return finished flag
    return thereYet;

  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.stopDrive();
    processor.stopProcessor();
    shooter.stopShooter();
    
  }

}
