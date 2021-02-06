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
import frc.robot.Constants.*;
import frc.robot.Constants.DrivetrainConstants.*;
import frc.robot.extraClasses.PIDControl;


public class AutoDriveToBall extends CommandBase {

  /**
   * Declare variables
   */
  private final Drivetrain drivetrain;
  private final Intake intake;
  private final Processor processor;
  private Timer timer;
  private PIDControl pidAngle;
  private PIDControl pidSpeed;


  /** 
   * Constructs a new AutoDriveToBall command
   */
  public AutoDriveToBall(Drivetrain drive, Intake rollers, Processor tunnel) {
    
    //Declare required subsystems
    drivetrain = drive;
    intake = rollers;
    processor = tunnel;
    addRequirements(drivetrain, intake, processor);

    //Create new timer and controllers
    timer = new Timer();
    pidAngle = new PIDControl(kP_Turn, kI_Turn, kD_Turn);
    pidSpeed = new PIDControl(kP_Straight, kI_Straight, kD_Straight);


  }


  /**
   * Initialize command
   */
  @Override
  public void initialize() {}


  /**
   * Execute the functions of the command
   */
  @Override
  public void execute() {}


  /**
   * Perform cleanup tasks when command ends
   */
  @Override
  public void end(boolean interrupted) {}


  /**
   * Check if the command should end
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
