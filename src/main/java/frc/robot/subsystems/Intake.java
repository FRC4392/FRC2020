/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  VictorSPX mRollerMotor;
  Solenoid mSolenoid;

  /**
   * Creates a new Intake.
   */
  public Intake(){
    mRollerMotor = new VictorSPX(51);
    mSolenoid = new Solenoid(1);
  }

  public void setSpeed(double speed) {
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
