// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.DrivetrainConstants.*;
import frc.robot.extraClasses.PIDControl;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Processor2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.extraClasses.NetworkTableQuerier;
import frc.robot.extraClasses.BallData;


/** AutoGetAllBalls Class
 * 
 * This class picks up 3 balls placed in a random pattern on the field
 * 
 */
public class AutoGetAllBalls extends CommandBase {

  // Declare class variables
  private final Drivetrain drivetrain;
  private final Pneumatics shifter;
  private final Processor2 processor;
  private final NetworkTableQuerier ntables;
  private final BallData balldata;

  private int ballCount;

  private boolean endRun;
  private boolean executeTurn;
  private boolean holdAngle;

  private double ballDistance;
  private double ballAngle;
  private double ballOffset;
  private double nextBallAngle;
  private double direction;
  private double stopTime;
  private double startTime;
  private double angleCorrection;
  private double speedCorrection;
  private double currentGyroAngle;
  private double targetGyroAngle;
  private double angleDeadband;

  private double distanceTraveled;
  private double targetDistance;
  private double leftEncoderStart;
  private double rightEncoderStart;

  private Timer timer;

  private PIDControl pidAngle; 


  /** Default constructor */
  public AutoGetAllBalls(Drivetrain drive, Pneumatics shift, Processor2 process, NetworkTableQuerier table, BallData data, double deadband, double time) {

    // Set class variables
    drivetrain = drive;
    shifter = shift;
    processor = process;
    ntables = table;
    balldata = data;
    angleDeadband = deadband;
    stopTime = time;

    // Add subsystem requirements
    addRequirements(drivetrain, shifter, processor);
    
    // Create the PID controller
    pidAngle = new PIDControl(kP_Turn, kI_Turn, kD_Turn);

  }


  /** Initialize this command (only runs first time command is called) */
  @Override
  public void initialize() {

    // Start the timer and get the command start time
    timer = new Timer();
    timer.start();
    startTime = timer.get();

    // Initialize class variables
    direction = -1;
    angleCorrection = 0;
    speedCorrection = 1;
    ballCount = 0;
    targetGyroAngle = 0;
    distanceTraveled = 0;
    targetDistance = 150;//in inches; test value, possibly needs slight adjustment
    leftEncoderStart = 0;
    rightEncoderStart = 0;

    // Shift into high gear
    shifter.shiftUp();

    // Get the initial ball positions
    getInitialBallPositions();

    // Initialize flags
    executeTurn = false;
    holdAngle = false;
    endRun = false;

    // Zero gyro angle
    drivetrain.zeroGyro();

  }


  /** Main code to be executed every robot cycle */
  @Override
  public void execute() {

    // Get guro angle
    currentGyroAngle = drivetrain.getGyroAngle();
    
    SmartDashboard.putNumber("BallAngle1", balldata.getBallToBallAngle(0));
    SmartDashboard.putNumber("BallAngle2", balldata.getBallToBallAngle(1));
    SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
    // Check if executing turn or driving to ball
    if (executeTurn) {

      // Get angle to the next ball
      nextBallAngle = balldata.getBallToBallAngle(ballCount - 1);
      // nextBallAngle = 30;
      SmartDashboard.putNumber("NextBallAngle", nextBallAngle);

      // Calculate angle correction
      angleCorrection = pidAngle.run(currentGyroAngle, nextBallAngle);

      // Fix speed correction and run drivetrain
      if (Math.abs(nextBallAngle) > 45){
        speedCorrection = .75;
      } else {
        speedCorrection = .8;
      }

      drivetrain.autoDrive(direction * speedCorrection *  kAutoDriveSpeed + angleCorrection, direction * speedCorrection * kAutoDriveSpeed - angleCorrection);

    }
    //If we are finishing the run (because we have three balls)
    else if (endRun){

      //Orient to 0 degrees
      double targetAngle = 0;
      angleCorrection = pidAngle.run(currentGyroAngle, targetAngle);
      speedCorrection = 1.0;

      //Drive
      drivetrain.autoDrive(direction * speedCorrection *  kAutoDriveSpeed + angleCorrection, direction * speedCorrection * kAutoDriveSpeed - angleCorrection);
      
      //Count encoder revs until such distance that we definitely get past the line
      double totalRotationsRight = Math.abs((drivetrain.getMasterRightEncoderPosition() - rightEncoderStart));
      double totalRotationsLeft = Math.abs((drivetrain.getMasterLeftEncoderPosition() - leftEncoderStart));
      distanceTraveled = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / AUTO_ENCODER_REVOLUTION_FACTOR;
      SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
    
    } else {

      // Get current values from vision and gyro
      ballDistance = ntables.getVisionDouble("BallDistance0");
      ballOffset = ntables.getVisionDouble("BallOffset0");

      // Check if we are close enough and centered enough to hold the angle
      if (holdAngle == false) {

        if (ballDistance < 35 && Math.abs(ballOffset) < 4) {

          holdAngle = true;
          targetGyroAngle = currentGyroAngle;

        }
  
      }
    
      // Calculate angle correction for driving
      if (holdAngle == false) {

        angleCorrection = pidAngle.run(ballOffset, 0);

      }
      else {

        angleCorrection = pidAngle.run(currentGyroAngle, targetGyroAngle);

      }
      SmartDashboard.putNumber("Angle Correction", angleCorrection);
      SmartDashboard.putBoolean("Hold Angle", holdAngle);

      // Determine speed correction based on distance
      if (ballDistance > 40) {

        speedCorrection = 1;

      } else {

        speedCorrection = 1;

      }

      // Run the drivetrain
      drivetrain.autoDrive(direction * speedCorrection *  kAutoDriveSpeed + angleCorrection, direction * speedCorrection * kAutoDriveSpeed - angleCorrection);

      //Run the processor
      processor.autoRunProcessor(false);
    }

  }


  /** Check to see if command should finish executing */
  @Override
  public boolean isFinished() {

    SmartDashboard.putNumber("Ball Count", ballCount);
    SmartDashboard.putBoolean("ExecuteTurn", executeTurn);
    SmartDashboard.putBoolean("EndRun", endRun);
    // Initialize stopping flag
    boolean thereYet = false;
  
    SmartDashboard.putBoolean("ThereYet", thereYet);

    // Get current time
    double time = timer.get();

    // Check for max time
    if (stopTime <= time - startTime) {

      // Set flag
      thereYet = true;

    } else {

      // Determine if turning or driving
      if (executeTurn) {

        // Get angles
        double currentAngle = drivetrain.getGyroAngle();
        double targetAngle = balldata.getBallToBallAngle(ballCount - 1);
        SmartDashboard.putNumber("TargetAngle", targetAngle);

        // Check for same angle within deadband
        if (Math.abs(targetAngle - currentAngle) <= angleDeadband) {

          executeTurn = false;

        }

      //If we are finishing the run
      } else if (endRun) {

        //Check if distance is far enough yet, then stop
        // if (distanceTraveled > targetDistance){
        //   thereYet = true;
        // }

      } else {

        if(drivetrain.getProcessorEntry() == false) {

          // Increment ball count
          ballCount++;

          // Check if we have picked up last ball
          if (ballCount == 3) {

            // Move to ending the run
            endRun = true;
            executeTurn = false;
            
            //Reset the encoders for the distance-driving part of the program
            drivetrain.zeroEncoders();
            leftEncoderStart = drivetrain.getMasterLeftEncoderPosition();
            rightEncoderStart = drivetrain.getMasterRightEncoderPosition();
            SmartDashboard.putNumber("LeftEncStart", leftEncoderStart);
            SmartDashboard.putNumber("RightEncStart", rightEncoderStart);
            

          } else {

            // Set flags
            executeTurn = true;
            holdAngle = false;

          }
        }
      }

    }

    // Return stopping flag
    return thereYet;

  }


  /** Code to run when this command has finished execution */
  @Override
  public void end(boolean interrupted) {

    drivetrain.stopDrive();
    processor.stopProcessor();
  }


  /**
   *  Gets the initial positions of all three balls
   *  before the robot moves to get them
   */
  private void getInitialBallPositions() {

    // Loop over three balls
    for (int i = 0; i < 3; i++)
    {

      // Distance
      String distanceKey = "BallDistance" + Integer.toString(i);
      Double distance = ntables.getVisionDouble(distanceKey);
      balldata.setBallDistance(i, distance);

      // Angle
      String angleKey = "BallAngle" + Integer.toString(i);
      Double angle = ntables.getVisionDouble(angleKey);
      balldata.setBallAngle(i, angle);

      // Offset
      String offsetKey = "BallOffset" + Integer.toString(i);
      Double offset = ntables.getVisionDouble(offsetKey);
      balldata.setBallOffset(i, offset);

    }

    // Calculate ball to ball angles
    balldata.calcInterBallAngles();
    
  }

}
