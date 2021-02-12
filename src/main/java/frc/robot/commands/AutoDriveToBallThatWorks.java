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
import frc.robot.subsystems.Processor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NetworkTableQuerier;

public class AutoDriveToBallThatWorks extends CommandBase {
  
  private final Drivetrain drivetrain;
  private final Pneumatics shifter;
  private final NetworkTableQuerier ntables;
  // private final Processor processor;

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


  public AutoDriveToBallThatWorks(Drivetrain drive, Pneumatics shift, NetworkTableQuerier table, double time) {
    // processor = process;
    drivetrain = drive;
    shifter = shift;
    ntables = table;
    addRequirements(drivetrain, shifter);

    stopTime = time;

    pidAngle = new PIDControl(kP_Turn, kI_Turn, kD_Turn);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.start();
    startTime = timer.get();

    angleCorrection = 0;
    angleError = 0;
    speedCorrection = 1;

    shifter.shiftUp();
    // shifter.retractIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double ballOffset = ntables.getVisionDouble("BallOffset0");
    angleCorrection = pidAngle.run(ballOffset, 0);
    
    speedCorrection = 1;
    direction = -1;
    drivetrain.autoDrive(speedCorrection * direction * kAutoDriveSpeed + angleCorrection, speedCorrection * direction*kAutoDriveSpeed - angleCorrection);
    SmartDashboard.putNumber("Angle Correction", angleCorrection);
    // processor.runProcessor(false);
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

    /*if(processor.getProcessorEntry() == false) {
      thereYet = true;
    }
    else 
    */if (stopTime <= time - startTime){

      thereYet = true;
    }
   
    return thereYet;

  }
}
