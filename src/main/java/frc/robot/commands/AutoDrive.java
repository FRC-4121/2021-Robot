/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.PneumaticsConstants.*;
import frc.robot.extraClasses.PIDControl;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Drivetrain;

public class AutoDrive extends CommandBase {
  
  private final Drivetrain drivetrain;
  private final Pneumatics shifter;

  private double targetDistance;
  private double targetAngle;
  private double direction;
  private double stopTime;

  private double angleCorrection, angleError, speedCorrection;
  private double startTime;
  private double distanceTraveled;

  private double leftEncoderStart;
  private double rightEncoderStart;


  private Timer timer = new Timer();
  private PIDControl pidAngle;
  private PIDControl pidSpeed;
  private PIDControl pidSpeedHigh; 


  public AutoDrive(Drivetrain drive, Pneumatics shift, double dis, double ang, double dir, double time) {

    drivetrain = drive;
    shifter = shift;
    addRequirements(drivetrain, shifter);

    targetDistance = dis;
    targetAngle = ang;
    direction = dir;
    stopTime = time;

    pidAngle = new PIDControl(kP_Turn, kI_Turn, kD_Turn);
    pidSpeed = new PIDControl(kP_Straight, kI_Straight, kD_Straight);
    //pidSpeedHigh = new PIDControl(kP_Speed_High, kI_Speed_High, kD_Speed_High);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    distanceTraveled = 0.0;
    timer.start();
    startTime = timer.get();

    leftEncoderStart = drivetrain.getMasterLeftEncoderPosition();
    rightEncoderStart = drivetrain.getMasterRightEncoderPosition();

    angleCorrection = 0;
    angleError = 0;
    speedCorrection = 1;

    shifter.shiftUp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    angleCorrection = pidAngle.run(drivetrain.getGyroAngle(), targetAngle);
    
    speedCorrection = pidSpeed.run(distanceTraveled, targetDistance);
    
    if (speedCorrection > 1.0) {
      speedCorrection = 1.0;
    } else if (speedCorrection < -1.0) {
      speedCorrection = -1.0;
    }
    drivetrain.autoDrive(speedCorrection * direction * kAutoDriveSpeed + angleCorrection, speedCorrection * direction*kAutoDriveSpeed - angleCorrection);
    SmartDashboard.putNumber("Angle Correction", angleCorrection);
    SmartDashboard.putNumber("Speed Correction", speedCorrection);

    double totalRotationsRight = Math.abs((drivetrain.getMasterRightEncoderPosition() - rightEncoderStart));
    double totalRotationsLeft = Math.abs((drivetrain.getMasterLeftEncoderPosition() - leftEncoderStart));

    distanceTraveled = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / AUTO_ENCODER_REVOLUTION_FACTOR;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean thereYet = false;

    double time = timer.get();

    
    if (Math.abs(targetDistance - distanceTraveled) <= 4){

      thereYet = true;

      // else if(Math.abs(targetDistance - distanceTraveled) <= 24){

        //shifter.shiftDown();
      
      //}

      

    } else if (stopTime <= time - startTime){

      thereYet = true;
    }
    SmartDashboard.putNumber("Distance Traveled", distanceTraveled);

    return thereYet;

  }
}
