/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  CANSparkMax mRollerMotor;
  Solenoid mSolenoid;

  /**
   * Creates a new Intake.
   */
  public Intake(){
    mRollerMotor = new CANSparkMax(51, MotorType.kBrushless);
    mSolenoid = new Solenoid(1);
  }

  private void setSpeed() {
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
