/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Turret extends SubsystemBase {

  //Declare turret motor and encoder
  private final CANSparkMax turret = new CANSparkMax(TURRET, MotorType.kBrushless);
  private final CANEncoder turretEncoder = turret.getEncoder();

  //Declare class variables
  private boolean resetEncoder = false;
  private double turretAngle;


  /**
   * Creates a new Turret.
   */
  public Turret() {

    //Configured to calculate the angle around the turret from the limit switch
    //theta (radians) = arclength / radius
    turretEncoder.setPosition(0);
    //turretEncoder.setPositionConversionFactor(-kTurretSprocketDia * 360 / (kTurretEncoderPPR * kTurretGearReduction * kTurretDiskDia/2));
  }


  /**
   * Actions to take on a periodic basis
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    turretAngle = turretEncoder.getPosition() * 140 / 3.04;
    SmartDashboard.putNumber("Turret Angle", turretAngle);
  }


  /**
   * Rotate the turret
   * @param speed
   */
  public void rotateTurret(double speed){

    turret.set(speed);
  }


  /**
   * Stop the turret
   */
  public void stopTurret(){

    turret.set(0);

  }


  /**
   * Get the current angle of the turret
   * @return
   */
  public double getTurretAngle(){

    return turretAngle;
  }

}
