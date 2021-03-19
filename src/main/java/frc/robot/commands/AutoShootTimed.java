// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Processor2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.extraClasses.NetworkTableQuerier;
import frc.robot.extraClasses.PIDControl;
import frc.robot.Constants.DrivetrainConstants;
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
  private final Pneumatics myPneumatics;
  private final NetworkTableQuerier myNTables;
  private Ballistics myBallistics;

  private int robotMode;      // 1: Shooting, 2: Drive to Loading, 3: Loading Wait, 4: Drive to Shooting
  private int ballCount;

  private double driveDistance;
  private double targetDriveDistance;
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
  private double shotPossible;//Ballistics value; 0 is false, 1 is 
  private double shotWaitTime;
  private double shotTime;
  private double loopCount = 0;
  private boolean shooting = false;
  private boolean ballEntering;
  private boolean turretLocked;
  private boolean timeSet;
  private double allShotsTime;
  private double lastShotWaitTime;
  private double targetAngle;

  private boolean foundTarget;
  private boolean targetLock;
  private boolean runSpeedControl;

  private double[] ballisticsData;

  private Timer runTimer;
  private Timer shotTimer;

  private PIDControl pidDriveAngle;
  private PIDControl pidDriveDistance;
  private PIDControl pidShooterSpeed;
  private PIDControl pidLock;
  private PIDControl pidTurret;


  /** Default constructor */
  public AutoShootTimed(Drivetrain drive, Shooter shoot, Pneumatics pneumatics, Processor2 process, Turret shootturret, NetworkTableQuerier table, double time) {

    // Set class variables
    myDrivetrain = drive;
    myShooter = shoot;
    myProcessor = process;
    myTurret = shootturret;
    myPneumatics = pneumatics;
    myNTables = table;
    // driveDistance = distance;
    // angleDeadband = deadband;
    stopTime = time;

    // Add subsystem requirements
    addRequirements(myDrivetrain, myShooter, myProcessor, myTurret);

    // Create PID controllers
    pidDriveAngle = new PIDControl(kP_DriveAngle, kI_DriveAngle, kD_DriveAngle);
    pidDriveDistance = new PIDControl(kP_Straight, kI_Straight, kD_Straight);
    pidShooterSpeed = new PIDControl(kP_Shoot, kI_Shoot, kD_Shoot);
    pidLock = new PIDControl(kP_TurretLock, kI_TurretLock, kD_TurretLock);
    pidTurret = new PIDControl(kP_Turret, kI_Turret, kD_Turret);

    // Create the ballistics table
    myBallistics = new Ballistics(98.25, 22.5, 5, 6050, 6, .25);

  }


  /** Initialize this command (only runs first time command is called) */
  @Override
  public void initialize() {

    // Initialize variables
    driveDirection = -1;
    driveDistance = 0;
    targetDriveDistance = 90;
    driveSpeedCorrection = 1;
    totalRotationsRight = 0;
    totalRotationsLeft = 0;
    ballCount = 3;
    turretCorrection = 0;
    shooterSpeed = .75;
    shooterSpeedCorrection = 0;
    targetShooterSpeed = kShooterMaxRPM;
    targetDistance = 0;
    shotWaitTime = .5;
    ballEntering = false;
    turretLocked = false;
    targetAngle = 10.4;
    
    timeSet = false;
    lastShotWaitTime = .25;

    // Initialize flags
    runSpeedControl = true;

    // Initialize shooting mode
    robotMode = 1;

    // Start the timers and get the command start time
    runTimer = new Timer();
    runTimer.start();
    startTime = runTimer.get();

    shotTimer = new Timer();
    shotTimer.start();
    shotTime = shotTimer.get();
    
    // Zero gyro angle
    myDrivetrain.zeroGyro();

    // Get starting encoder positions
    leftEncoderStart = Math.abs(myDrivetrain.getMasterLeftEncoderPosition());
    rightEncoderStart = Math.abs(myDrivetrain.getMasterRightEncoderPosition());

    myPneumatics.retractIntake();

  }


  /** Main code to be executed every robot cycle */
  @Override
  public void execute() {

    // Get status of flags/NT data
    foundTarget = myNTables.getFoundTapeFlag();
    targetLock = myNTables.getTargetLockFlag();
    targetOffset = myNTables.getTapeOffset();
    targetDistance = myNTables.getTapeDistance();


    // Aim turret and configure hood position
    double turretSpeed = 0;
    double turretAngle = myTurret.getTurretAngle();
    if (foundTarget) {

      if (robotMode == 1){
      
        //If the target is not centered in the screen
        if (!targetLock && turretLocked == false) {

          //If the turret is in a safe operating range for the physical constraints of the robot
          turretSpeed = -kTurretSpeedAuto * pidLock.run(targetOffset, -5.0);
          SmartDashboard.putNumber("TurretSpeed", turretSpeed);

          myTurret.rotateTurret(turretSpeed);
        
        } else {

          //If target is locked, stop the motor
          myTurret.stopTurret();
          turretLocked = true;
          targetAngle = myTurret.getTurretAngle();

        }
      
      } else {
        turretSpeed = -kTurretSpeedAuto * pidTurret.run(turretAngle, targetAngle);
        SmartDashboard.putNumber("TurretSpeed", turretSpeed);
        myTurret.rotateTurret(turretSpeed);
      }
      
      //This probably shouldn't be implemented here because we don't need it, but 
      //it will be useful for the other version of this challenge
      if (targetDistance < 85) {
        //set high angle (55)
      
      } else {
        //set low angle (45)
      }

    } else {

      //If the camera does not see a target, we need to figure out how to write the code for this
      turretSpeed = -kTurretSpeedAuto * pidTurret.run(turretAngle, 0.0);
      SmartDashboard.putNumber("TurretSpeed", turretSpeed);
      myTurret.rotateTurret(turretSpeed);

    }

    targetShooterSpeedCorrected = 0;

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
        SmartDashboard.putNumber("Ballistics Speed", targetShooterSpeedCorrected);

        myShooter.shoot(targetShooterSpeedCorrected);
        //I have battery concerns about this implementation.  If we notice that battery draw during a match is problematic for speed control, we
        //will need to revert to a pid for RPM in some way.  This would be sufficiently complicated that it is a low priority, however.

      } else {

        myShooter.shoot(shooterSpeed);

      }

    } else {

      myShooter.shoot(shooterSpeed);  

    } 


    // Determine mode and take action
    switch(robotMode) {

      // Shooting
      case 1:

        //Run processor normally regardless of position conditions
        myProcessor.lockProcessor();
        myProcessor.autoRunProcessor(false, true);

        if (targetLock) {

          //Ensure wheel is moving fast enough to accurately make shot
          double l_targetSpeed = targetShooterSpeed * kShooterMaxRPM;
          SmartDashboard.putNumber("l_targetSpeed", l_targetSpeed);
          SmartDashboard.putNumber("Shooter RPM", myShooter.getShooterRPM());
          SmartDashboard.putNumber("tolerance", kRPMTolerance);
          
          if (ballCount > 0) {
            double time = shotTimer.get();
            SmartDashboard.putNumber("Shot Timer", time - shotTime);
            if(time - shotTime >= shotWaitTime){
              if (Math.abs(Math.abs(myShooter.getShooterRPM()) - targetShooterSpeed * kShooterMaxRPM) < kRPMTolerance || shooting) {
                myProcessor.unlockProcessor();
                shooting = true;
                loopCount++;
                if (loopCount == 10) {
                  ballCount--;
                  shotTime = time;
                  loopCount = 0;
                  shooting = false;
                }
              }  
            }
          } 

        }

        break;
      
      // Driving to load area
      case 2:

        // Calculate angle correction based on gyro reading
        currentGyroAngle = myDrivetrain.getGyroAngle();
        angleCorrection = pidDriveAngle.run(currentGyroAngle, 0);

        // Calculate speed correction based on distance to target
        totalRotationsRight = Math.abs((Math.abs(myDrivetrain.getMasterRightEncoderPosition()) - rightEncoderStart));
        totalRotationsLeft = Math.abs((Math.abs(myDrivetrain.getMasterLeftEncoderPosition()) - leftEncoderStart));
        driveDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / (DrivetrainConstants.kTalonFXPPR * kGearRatio);
        driveSpeedCorrection = pidDriveDistance.run(driveDistance, targetDriveDistance);
        SmartDashboard.putNumber("DrSpCorrection", driveSpeedCorrection);
        if(driveSpeedCorrection > 1){
          driveSpeedCorrection = 1;
        } else if (driveSpeedCorrection < -1) {
          driveSpeedCorrection = -1;
        }
        // Run the drive
        driveDirection = 1;
        myDrivetrain.autoDrive(driveDirection * driveSpeedCorrection *  kAutoShootDriveSpeed + angleCorrection, driveDirection * driveSpeedCorrection * kAutoShootDriveSpeed - angleCorrection);

        break;
      
      // Waiting in load area
      case 3:

        //Run the processor continually
        myProcessor.autoRunProcessor(false, true);

        break;
      
      // Driving to shooting location
      case 4:

        // Calculate angle correction based on gyro reading
        currentGyroAngle = myDrivetrain.getGyroAngle();
        angleCorrection = pidDriveAngle.run(currentGyroAngle, 0);

        // Calculate speed correction based on distance to target
        totalRotationsRight = Math.abs((Math.abs(myDrivetrain.getMasterRightEncoderPosition()) - rightEncoderStart));
        totalRotationsLeft = Math.abs((Math.abs(myDrivetrain.getMasterLeftEncoderPosition()) - leftEncoderStart));
        driveDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / (DrivetrainConstants.kTalonFXPPR * kGearRatio);
        driveSpeedCorrection = pidDriveDistance.run(driveDistance, targetDriveDistance);

        // Run the drive
        driveDirection = -1;
        myDrivetrain.autoDrive(driveDirection * driveSpeedCorrection *  kAutoShootDriveSpeed + angleCorrection, driveDirection * driveSpeedCorrection * kAutoShootDriveSpeed - angleCorrection);

        break;

    }

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("BallCount", ballCount);
    // Initialize finished flag
    boolean thereYet = false;


    // Get current time
    double time = runTimer.get();


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

            if (!timeSet) {

              allShotsTime = runTimer.get();
              timeSet = true;

            } 
            else if (runTimer.get() - allShotsTime > shotWaitTime) {

              // Reset flag
              timeSet = false;

              // Set next mode
              robotMode = 2;

              // Reset starting encoder positions
              leftEncoderStart = Math.abs(myDrivetrain.getMasterLeftEncoderPosition());
              rightEncoderStart = Math.abs(myDrivetrain.getMasterRightEncoderPosition());

              // Turn off shooter speed control
              runSpeedControl = false;
              turretLocked = false;

            }

          }

          break;
      
        // Driving to load area
        case 2:

          myProcessor.stopProcessor();
          // Calculate distance traveled
          totalRotationsRight = Math.abs((Math.abs(myDrivetrain.getMasterRightEncoderPosition()) - rightEncoderStart));
          totalRotationsLeft = Math.abs((Math.abs(myDrivetrain.getMasterLeftEncoderPosition()) - leftEncoderStart));
          driveDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / (DrivetrainConstants.kTalonFXPPR * kGearRatio);
          SmartDashboard.putNumber("DistanceTraveled", driveDistance);
          // Check distance against target
          SmartDashboard.putNumber("Distance error", Math.abs(driveDistance - targetDriveDistance));
          if (Math.abs(driveDistance - targetDriveDistance) < kDriveDistanceTolerance) {
            
            // Set next mode
            robotMode = 3;

          }

          break;
      
        // Waiting in load area
        case 3:

          // Check light sensor for ball loading
          if (!ballEntering) {
            if(myDrivetrain.getProcessorEntry() == false) {
             ballEntering = true;
            }       
          }
          else {
            if(myDrivetrain.getProcessorEntry() == true && ballEntering == true){
              ballCount++;
              ballEntering = false;
            }
          }
          // if (myDrivetrain.getProcessorEntry() == false){
          //   ballCount++;
          // }

          // Check number of balls loaded
          if (ballCount == 3) {
            
            // Set next mode
            robotMode = 4;

            // Reset starting encoder positions
            leftEncoderStart = Math.abs(myDrivetrain.getMasterLeftEncoderPosition());
            rightEncoderStart = Math.abs(myDrivetrain.getMasterRightEncoderPosition());

            // Turn on shooter speed control
            runSpeedControl = true;

          }
          
          break;
      
        // Driving to shooting location
        case 4:

          myProcessor.stopProcessor();
          // Calculate distance traveled
          totalRotationsRight = Math.abs((Math.abs(myDrivetrain.getMasterRightEncoderPosition()) - rightEncoderStart));
          totalRotationsLeft = Math.abs((Math.abs(myDrivetrain.getMasterLeftEncoderPosition()) - leftEncoderStart));
          driveDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / (DrivetrainConstants.kTalonFXPPR * kGearRatio);
          SmartDashboard.putNumber("Distance Traveled", driveDistance);
          // Check distance against target
          if (Math.abs(driveDistance - targetDriveDistance) < kDriveDistanceTolerance) {
            
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
    myTurret.stopHood();
    myTurret.stopTurret();
    
  }

}
