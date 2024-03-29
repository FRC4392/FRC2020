/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  VictorSPX mRollerMotor;
  Solenoid mSolenoid;

  
  public Intake(){
    mRollerMotor = new VictorSPX(51);
    mRollerMotor.setInverted(true);

    mSolenoid = new Solenoid(2);
  }

  public void setSpeed(double speed) {
    mRollerMotor.set(ControlMode.PercentOutput, speed);
  }

  public void lift(){
    mSolenoid.set(false);
  }

  public void lower(){
    mSolenoid.set(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}