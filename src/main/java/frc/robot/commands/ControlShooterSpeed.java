/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.extraClasses.Ballistics;
import frc.robot.extraClasses.PIDControl;
import frc.robot.extraClasses.NetworkTableQuerier;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.ShooterConstants.*;

public class ControlShooterSpeed extends CommandBase {
  
  private Shooter shooter;
  private NetworkTableQuerier ntQuerier;
  private Ballistics ballistics;

  private PIDControl myPID;
  private double distance;
  private boolean targetLock;
  private double targetSpeed;
  private double targetSpeedCorrected;
  private double speed;
  private double speedCorrection;
  private double shotPossible;//Ballistics value; 0 is false, 1 is true

  private boolean runSpeedControl = true;

  private double[] ballisticsData;
  
  public ControlShooterSpeed(Shooter shoot, NetworkTableQuerier querier) {

    shooter = shoot;
    ntQuerier = querier;
    ballistics = new Ballistics(98.25, 22.5, 5, 6050, 6, .25);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    myPID = new PIDControl(kP_Shoot, kI_Shoot, kD_Shoot);
    speed = .75;
    speedCorrection = 0;

    targetSpeed = 1.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(runSpeedControl){

      targetLock = ntQuerier.getTargetLockFlag();

      if(targetLock)
      {
        
        distance = ntQuerier.getTapeDistance();
        // distance = 153;
        ballisticsData = ballistics.queryBallisticsTable(distance);
        shotPossible = ballisticsData[0];

        if(shotPossible == 0)
        {
          SmartDashboard.putBoolean("Shot Possible", false);
          targetSpeed = speed;
        }
        else
        {
          SmartDashboard.putBoolean("Shot Possible", true);
          targetSpeed = ballisticsData[2];
        }

        targetSpeedCorrected = targetSpeed * kSpeedCorrectionFactor;
        SmartDashboard.putNumber("Ballistics Speed", targetSpeed);

        shooter.shoot(-targetSpeedCorrected);
        //I have battery concerns about this implementation.  If we notice that battery draw during a match is problematic for speed control, we
        //will need to revert to a pid for RPM in some way.  This would be sufficiently complicated that it is a low priority, however.
      }
      else 
      {

        shooter.shoot(-speed);
      }

    }

    SmartDashboard.putNumber("Shooter Speed", shooter.getShooterSpeed());
    SmartDashboard.putNumber("Shooter RPM", shooter.getShooterRPM());
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

  public void stopAutoSpeed(){


  }
}
