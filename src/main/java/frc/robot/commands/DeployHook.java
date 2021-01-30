/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class DeployHook extends CommandBase {
  
  private Climber climber;
  private double startHeight;
  private double targetHeight;
  private double startTime;
  private double stopTime;

  private Timer timer;
  
  //In progress 'auto hook' commands
  public DeployHook(Climber climb, double height, double time) {

    climber = climb;
    targetHeight = height;
    stopTime = time;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer = new Timer();
    startTime = timer.get();

    startHeight = climber.getHeight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    climber.runHook();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    boolean thereYet = false;
    double currentTime = timer.get();
    double timeChange = currentTime - startTime;

    double currentHeight = climber.getHeight();
    double heightChange = Math.abs(currentHeight - startHeight);
    double heightErr = Math.abs(targetHeight - heightChange);

    if(heightErr < kHeightTolerance)
    {
      thereYet = true;
    }
    else if(timeChange > stopTime)
    {
      thereYet = true;
    }

    return thereYet;
  }
}
