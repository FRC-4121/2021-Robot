// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Processor;
import frc.robot.subsystems.Processor2;

public class AutoRunProcessor extends CommandBase {
  /** Creates a new AutoRunProcessor. */
  private Processor2 processor;
  private boolean invert;
  private Timer timer;
  private double startTime;
  private double stopTime;
  
  public AutoRunProcessor(Processor2 process, boolean direction, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    processor = process;
    addRequirements(processor);
    stopTime = time;

    timer = new Timer();
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
    processor.autoRunProcessor(invert);
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
    double time = timer.get();

    if (stopTime <= time - startTime){
      thereYet = true;
    }

    return thereYet;
  }
}
