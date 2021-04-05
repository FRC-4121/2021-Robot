// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Processor2;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.ShooterConstants.*;

public class ShootWithSpeedConfirm extends CommandBase {
  /** Creates a new ShootWithSpeedConfirm. */
  private Shooter myShooter;
  private Processor2 myProcess;
  boolean shooting = false;
  boolean targetLock;
  double targetShooterSpeed;
  int loopCount = 0;

  public ShootWithSpeedConfirm(Shooter shooter, Processor2 process) {
    myProcess = process;
    myShooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myProcess, myShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooting = false;
    targetShooterSpeed = 0;
    loopCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Run processor normally regardless of position conditions
    myProcess.lockProcessor();
    myProcess.autoRunProcessor(false, true);

    //Check target lock (tossed up to SmartDash in ControlShooterSpeed)
    targetLock = SmartDashboard.getBoolean("TargetLock", false);
    targetShooterSpeed = SmartDashboard.getNumber("Ballistics Speed", 0);

    if (targetLock) {

      //Ensure wheel is moving fast enough to accurately make shot
      double l_targetSpeed = targetShooterSpeed * kShooterMaxRPM;
      SmartDashboard.putNumber("l_targetSpeed", l_targetSpeed);
      SmartDashboard.putNumber("Shooter RPM", myShooter.getShooterRPM());
      SmartDashboard.putNumber("tolerance", kRPMTolerance);
      
      if (Math.abs(Math.abs(myShooter.getShooterRPM()) - targetShooterSpeed * kShooterMaxRPM) < kRPMTolerance || shooting) {
        myProcess.unlockProcessor();
        shooting = true;
        loopCount++;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myProcess.stopProcessor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooting && loopCount > 8){
      return true;
    }
    return false;
  }
}