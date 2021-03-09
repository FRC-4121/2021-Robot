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
import frc.robot.subsystems.Turret;
import frc.robot.extraClasses.NetworkTableQuerier;
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
  private final Drivetrain myDrivetrain;
  private final Shooter myShooter;
  private final Processor2 myProcessor;
  private final Turret myTurret;
  private final NetworkTableQuerier myNTables;
  private Ballistics myBallistics;

  private int robotMode;      // 1: Shooting, 2: Drive to Loading, 3: Loading Wait, 4: Drive to Shooting
  private int ballCount;

  private double driveDistance;
  private double driveDirection;
  private double driveSpeedCorrection;
  private double leftEncoderStart;
  private double rightEncoderStart;
  private double totalRotationsRight;
  private double totalRotationsLeft;
  private double stopTime;
  private double startTime;
  private double currentGyroAngle;
  private double angleCorrection;
  private double angleDeadband;
  private double targetOffset;
  private double turretCorrection;
  private double targetDistance;
  private double targetShooterSpeed;
  private double targetShooterSpeedCorrected;
  private double shooterSpeed;
  private double shooterSpeedCorrection;
  private double shotPossible;//Ballistics value; 0 is false, 1 is true

  private boolean foundTarget;
  private boolean targetLock;
  private boolean runSpeedControl;

  private double[] ballisticsData;

  private Timer timer;

  private PIDControl pidDriveAngle;
  private PIDControl pidDriveDistance;
  private PIDControl pidShooterSpeed;
  private PIDControl pidLock;
  private PIDControl pidTurret;


  /** Default constructor */
  public AutoShootTimed(Drivetrain drive, Shooter shoot, Processor2 process, Turret shootturret, NetworkTableQuerier table, double distance, double deadband, double time) {

    // Set class variables
    myDrivetrain = drive;
    myShooter = shoot;
    myProcessor = process;
    myTurret = shootturret;
    myNTables = table;
    driveDistance = distance;
    angleDeadband = deadband;
    stopTime = time;

    // Add subsystem requirements
    addRequirements(myDrivetrain, myShooter, myProcessor, myTurret);

    // Create PID controllers
    pidDriveAngle = new PIDControl(kP_Turn, kI_Turn, kD_Turn);
    pidDriveDistance = new PIDControl(kP_Straight, kI_Straight, kD_Straight);
    pidShooterSpeed = new PIDControl(kP_Shoot, kI_Shoot, kD_Shoot);
    pidLock = new PIDControl(kP_TurretLock, kI_TurretLock, kD_TurretLock);
    pidTurret = new PIDControl(kP_Turret, kI_Turret, kD_Turret);

    // Create the ballistics table
    myBallistics = new Ballistics(98.25, 22.5, 5, 6380, 6, .227);

  }


  /** Initialize this command (only runs first time command is called) */
  @Override
  public void initialize() {

    // Initialize variables
    driveDirection = -1;
    driveDistance = 0.0;
    driveSpeedCorrection = 1;
    totalRotationsRight = 0;
    totalRotationsLeft = 0;
    ballCount = 3;
    turretCorrection = 0;
    shooterSpeed = .75;
    shooterSpeedCorrection = 0;
    targetShooterSpeed = 1.0;
    targetDistance = 60;

    // Initialize flags
    runSpeedControl = true;

    // Initialize shooting mode
    robotMode = 1;

    // Start the timer and get the command start time
    timer = new Timer();
    timer.start();
    startTime = timer.get();
    
    // Zero gyro angle
    myDrivetrain.zeroGyro();

    // Get starting encoder positions
    leftEncoderStart = myDrivetrain.getMasterLeftEncoderPosition();
    rightEncoderStart = myDrivetrain.getMasterRightEncoderPosition();

  }


  /** Main code to be executed every robot cycle */
  @Override
  public void execute() {

    // Get status of flags/NT data
    foundTarget = myNTables.getFoundTapeFlag();
    targetLock = myNTables.getTargetLockFlag();
    targetOffset = myNTables.getTapeOffset();
    targetDistance = myNTables.getTapeDistance();


    // Aim turret
    double turretSpeed = 0;
    double turretAngle = myTurret.getTurretAngle();
    if (foundTarget) {

      //If the target is not centered in the screen
      if (!targetLock) {

        //If the turret is in a safe operating range for the physical constraints of the robot
        turretSpeed = -kTurretSpeedAuto * pidLock.run(targetOffset, -5.0);
        SmartDashboard.putNumber("TurretSpeed", turretSpeed);

        myTurret.rotateTurret(turretSpeed);
      
      } else {

        //If target is locked, stop the motor
        myTurret.stopTurret();

      }

    } else {

      //If the camera does not see a target, we need to figure out how to write the code for this
      turretSpeed = -kTurretSpeedAuto * pidTurret.run(turretAngle, 0.0);
      SmartDashboard.putNumber("TurretSpeed", turretSpeed);
      myTurret.rotateTurret(turretSpeed);

    }


    // Control shooter speed
    if(runSpeedControl) {

      targetLock = myNTables.getTargetLockFlag();

      if(targetLock) {
        
        // distance = 153;
        ballisticsData = myBallistics.queryBallisticsTable(targetDistance);
        shotPossible = ballisticsData[0];

        if(shotPossible == 0){

          SmartDashboard.putBoolean("Shot Possible", false);
          targetShooterSpeed = shooterSpeed;

        } else {

          SmartDashboard.putBoolean("Shot Possible", true);
          targetShooterSpeed = ballisticsData[2];

        }

        targetShooterSpeedCorrected = targetShooterSpeed * kSpeedCorrectionFactor;
        SmartDashboard.putNumber("Ballistics Speed", targetShooterSpeed);

        myShooter.shoot(-targetShooterSpeedCorrected);
        //I have battery concerns about this implementation.  If we notice that battery draw during a match is problematic for speed control, we
        //will need to revert to a pid for RPM in some way.  This would be sufficiently complicated that it is a low priority, however.

      } else {

        myShooter.shoot(-shooterSpeed);

      }

    } else {

      myShooter.shoot(-shooterSpeed);  

    } 


    // Determine mode and take action
    switch(robotMode) {

      // Shooting
      case 1:

        if (targetLock) {

          if (myShooter.getShooterSpeed() > kSpeedThreshold) {

          } else {

          }

        }

        break;
      
      // Driving to load area
      case 2:

        // Calculate angle correction based on gyro reading
        currentGyroAngle = myDrivetrain.getGyroAngle();
        angleCorrection = pidDriveAngle.run(currentGyroAngle, 0);

        // Calculate speed correction based on distance to target
        totalRotationsRight = Math.abs((myDrivetrain.getMasterRightEncoderPosition() - rightEncoderStart));
        totalRotationsLeft = Math.abs((myDrivetrain.getMasterLeftEncoderPosition() - leftEncoderStart));
        driveDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / AUTO_ENCODER_REVOLUTION_FACTOR;
        driveSpeedCorrection = pidDriveDistance.run(driveDistance, targetDistance);

        // Run the drive
        driveDirection = -1;
        myDrivetrain.autoDrive(driveDirection * driveSpeedCorrection *  kAutoDriveSpeed + angleCorrection, driveDirection * driveSpeedCorrection * kAutoDriveSpeed - angleCorrection);

        break;
      
      // Waiting in load area
      case 3:

        //Run the processor continually
        myProcessor.autoRunProcessor(false);

        break;
      
      // Driving to shooting location
      case 4:

        // Calculate angle correction based on gyro reading
        currentGyroAngle = myDrivetrain.getGyroAngle();
        angleCorrection = pidDriveAngle.run(currentGyroAngle, 0);

        // Calculate speed correction based on distance to target
        totalRotationsRight = Math.abs((myDrivetrain.getMasterRightEncoderPosition() - rightEncoderStart));
        totalRotationsLeft = Math.abs((myDrivetrain.getMasterLeftEncoderPosition() - leftEncoderStart));
        driveDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / AUTO_ENCODER_REVOLUTION_FACTOR;
        driveSpeedCorrection = pidDriveDistance.run(driveDistance, targetDistance);

        // Run the drive
        driveDirection = 1;
        myDrivetrain.autoDrive(driveDirection * driveSpeedCorrection *  kAutoDriveSpeed + angleCorrection, driveDirection * driveSpeedCorrection * kAutoDriveSpeed - angleCorrection);

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

    } 
    else
    {
    
      // Determine mode and make checks
      switch(robotMode) {

        // Shooting
        case 1:

          // Check number of balls shot
          if (ballCount == 0) {
            
            // Set next mode
            robotMode = 2;

            // Reset starting encoder positions
            leftEncoderStart = myDrivetrain.getMasterLeftEncoderPosition();
            rightEncoderStart = myDrivetrain.getMasterRightEncoderPosition();

            // Turn off shooter speed control
            runSpeedControl = false;

          }

          break;
      
        // Driving to load area
        case 2:

          // Calculate distance traveled
          totalRotationsRight = Math.abs((myDrivetrain.getMasterRightEncoderPosition() - rightEncoderStart));
          totalRotationsLeft = Math.abs((myDrivetrain.getMasterLeftEncoderPosition() - leftEncoderStart));
          driveDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / AUTO_ENCODER_REVOLUTION_FACTOR;

          // Check distance against target
          if (Math.abs(driveDistance - targetDistance) < kDriveDistanceTolerance) {
            
            // Set next mode
            robotMode = 3;

          }

          break;
      
        // Waiting in load area
        case 3:

          // Check light sensor for ball loading
          if(myDrivetrain.getProcessorEntry() == false) {
            ballCount++;
          }

          // Check number of balls loaded
          if (ballCount == 3) {
            
            // Set next mode
            robotMode = 4;

            // Reset starting encoder positions
            leftEncoderStart = myDrivetrain.getMasterLeftEncoderPosition();
            rightEncoderStart = myDrivetrain.getMasterRightEncoderPosition();

            // Turn on shooter speed control
            runSpeedControl = true;

          }
          
          break;
      
        // Driving to shooting location
        case 4:

          // Calculate distance traveled
          totalRotationsRight = Math.abs((myDrivetrain.getMasterRightEncoderPosition() - rightEncoderStart));
          totalRotationsLeft = Math.abs((myDrivetrain.getMasterLeftEncoderPosition() - leftEncoderStart));
          driveDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / AUTO_ENCODER_REVOLUTION_FACTOR;

          // Check distance against target
          if (Math.abs(driveDistance - targetDistance) < kDriveDistanceTolerance) {
            
            // Set next mode
            robotMode = 1;

          }

          break;
          
      }

    }


    // Return finished flag
    return thereYet;

  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    myDrivetrain.stopDrive();
    myProcessor.stopProcessor();
    myShooter.stopShooter();
    
  }

}
