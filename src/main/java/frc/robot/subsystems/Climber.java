/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//In progress -> currently testing in 2020-Playground
public class Climber extends SubsystemBase {
  
  private CANSparkMax hookMotor = new CANSparkMax(HOOK, MotorType.kBrushless);
  private CANEncoder hookEncoder = hookMotor.getEncoder();
  
  public Climber() {

    hookEncoder.setPosition(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hook Height", getHeight());
  }

  public void runHook(){
  
    hookMotor.set(kHookSpeed);
  }

  public void downHook(){
    hookMotor.set(-kHookSpeed);
  }

  public void stopHook(){

    hookMotor.set(0);
  }

  public double getHeight(){
    
    return hookEncoder.getPosition() * Math.PI * kShaftSize / (kClimberGearReduction * kClimbEncoderPPR);
  }

  //May be worthwhile to implement motion magic/PID, a la 2018 elevator
}
