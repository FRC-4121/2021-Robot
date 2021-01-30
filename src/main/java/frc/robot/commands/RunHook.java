/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RunHook extends CommandBase {
  
  private Climber climber;
  private double motorSpeed;
  
  public RunHook(Climber climb, double speed) {

    climber = climb;
    motorSpeed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //If we are raising the hook... (this comparision may need to be changed based on mech)
    if(motorSpeed > 0) {
      //If the climber is at the max height, stop
      if(Math.abs(climber.getHeight() - kMaxHookHeight) < kHeightTolerance)
        
        climber.stopHook();

      else //otherwise run normally

        climber.runHook();
    }
    else //if we are lowering the hook
    {
      //If the climber is near the bottom, stop
      if(Math.abs(climber.getHeight() - 0) < kHeightTolerance)
        
        climber.stopHook();

      else //otherwise run normally

        climber.runHook();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
