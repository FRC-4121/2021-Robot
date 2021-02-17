// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extraClasses;

/** Add your docs here. */
public class BallData {

    private double ballOffset2;
    private double ballOffset3;


    public BallData(){
        ballOffset2 = 0;
        ballOffset3 = 0;
    }

    public void SetBallOffset2(double offset){
        ballOffset2 = offset;
    }

    public void SetBallOffset3(double offset){
        ballOffset3 = offset;
    }

    public double GetBallOffset2(){
        return ballOffset2;
    }

    public double GetBallOffset3(){
        return ballOffset3;
    }

}
