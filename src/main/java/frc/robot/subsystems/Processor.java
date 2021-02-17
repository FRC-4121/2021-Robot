/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ProcessorConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extraClasses.PIDControl;

public class Processor extends SubsystemBase {
  
  private WPI_TalonSRX intake = new WPI_TalonSRX(INTAKE);
  private CANSparkMax processorMain = new CANSparkMax(PROCESSOR_MAIN, MotorType.kBrushless);
  private WPI_TalonSRX processorLock = new WPI_TalonSRX(PROCESSOR_END);
  
  
  public Processor() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Processor Current", processorMain.getOutputCurrent());

    //Index via photoelectric sensor
    
    //boolean logic tree for default setup
  }

  //invertDirection = false: normal intake->shooter direction
  public void runProcessor(boolean invertDirection){

    if(!invertDirection)
    {
      processorMain.set(kProcessorSpeed);
      intake.set(kIntakeSpeed);
      lockProcessor();
      SmartDashboard.putBoolean("Running Processor", true);
    }
    else 
    {
      processorMain.set(-kProcessorSpeed);
      intake.set(kOuttakeSpeed);
    }
  }
  
  public void autoRunProcessor(boolean invertDirection){

    if(!invertDirection)
    {
      processorMain.set(kProcessorSpeed);
      // intake.set(kIntakeSpeed);
      // lockProcessor();
      SmartDashboard.putBoolean("Running Processor", true);
    }
    else 
    {
      processorMain.set(-kProcessorSpeed);
      intake.set(kOuttakeSpeed);
    }
  }

  public void unlockProcessor(){

    processorLock.set(kUnlockSpeed);
  }

  public void lockProcessor(){

    processorLock.set(kLockSpeed);
  }

  public void stopProcessor(){

    intake.set(0);
    processorMain.set(0);
    processorLock.set(0);
    SmartDashboard.putBoolean("Running Processor", false);
  }

  public void deflectBalls(){}

  public void vomitBalls(){}

 
}
