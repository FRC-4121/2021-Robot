/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Processor;

public class RunProcessor extends CommandBase {
  
  private final Processor processor;
  private boolean invert;
  private Timer timer = new Timer();
  double startTime;
  double time;
  
  public RunProcessor(Processor process, boolean invertDirect) {

    invert = invertDirect;
    processor = process;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(processor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    startTime = timer.get();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    processor.runProcessor(invert);
    //processor.lockProcessor();
    time = timer.get();
    SmartDashboard.putNumber("Processor Time", time);
    SmartDashboard.putNumber("Processor Start Time", startTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    processor.stopProcessor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean thereYet = false;
    
    if (5 <= time - startTime){
      thereYet = true;
    }
    SmartDashboard.putBoolean("Processor TY", thereYet);

    return thereYet;
  }
}
