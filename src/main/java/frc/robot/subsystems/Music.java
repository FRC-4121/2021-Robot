// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collection;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {
  /** Creates a new Music. */

  private Shooter shoot;
  private TalonFX falcon1;
  private TalonFX falcon2;

  private Orchestra orchestra;
  private Collection<TalonFX> talonCollect;

  public Music(Shooter shooter) {

    shoot = shooter;
    falcon1 = shoot.getShooterMaster();
    falcon2 = shoot.getShooterSlave();

    // boolean falcon1added = talonCollect.add(falcon1);
    // boolean falcon2added = talonCollect.add(falcon2);

    orchestra = new Orchestra();
    orchestra.addInstrument(falcon1);
    orchestra.addInstrument(falcon2);
    orchestra.loadMusic("hooked_on_a_feeling.chrp");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void play(){
    orchestra.play();
  }

  public void pause(){
    orchestra.pause();
  }

  public void stop(){
    orchestra.stop();
  }
}
