// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Processor;
import frc.robot.subsystems.NetworkTableQuerier;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DrivetrainConstants.*;
import frc.robot.extraClasses.PIDControl;


public class AutoDriveToBall extends CommandBase {

  /**
   * Declare variables
   */
  private final Drivetrain drivetrain;
  private final NetworkTableQuerier ntables;
  private final Processor processor;
  private Timer timer;
  private double startTime;
  private double speedCorrection;
  private double angleCorrection;
  private double speedMultiplier;
  private PIDControl pidAngle;
  private PIDControl pidSpeed;
  private double stopTime;
  
  /** 
   * Constructs a new AutoDriveToBall command
   */
  public AutoDriveToBall(Drivetrain drive, NetworkTableQuerier tables, Processor tunnel, double time) {
    
    //Declare required subsystems
    drivetrain = drive;
    processor = tunnel;
    ntables = tables;
    addRequirements(drivetrain, processor);

    //Create new timer and controllers
    timer = new Timer();
    pidAngle = new PIDControl(kP_Turn, kI_Turn, kD_Turn);
    pidSpeed = new PIDControl(kP_Straight, kI_Straight, kD_Straight);
    stopTime = time;


  }


  /**
   * Initialize command
   */
  @Override
  public void initialize() {

    //Start timer and save init time.
    timer.start();
    startTime = timer.get();
    
    

  }


  /**
   * Execute the functions of the command
   */
  @Override
  public void execute() {
    // checks if the robot is far from the ball goes faster if its far away
    if(ntables.getVisionDouble("BallScreenPercent0") < 3) {
      
      speedMultiplier = 0.8; 
    }
    else
    {
      speedMultiplier = 0.6; 
    }
    
    double ballOffset = ntables.getVisionDouble("BallOffset0");
    angleCorrection = pidAngle.run(ballOffset, 0);
    SmartDashboard.putNumber("AutoBallAngleCorr", angleCorrection);

    drivetrain.autoDrive(speedMultiplier * kAutoDriveSpeed + angleCorrection, speedMultiplier * kAutoDriveSpeed - angleCorrection);


  }


  /**
   * Perform cleanup tasks when command ends
   */
  @Override
  public void end(boolean interrupted) {
    processor.runProcessor(false);
  }


  /**
   * Check if the command should end
   */
  @Override
  public boolean isFinished() {

    boolean thereYet = false;
    
    if(ntables.getVisionDouble("BallScreenPercent") > 7) {
    thereYet = true;

    } 
    else if (stopTime<=timer.get()-startTime){
      thereYet = true;

    }
    SmartDashboard.putBoolean("There Yet", thereYet);



    
    return false;
  }

}
