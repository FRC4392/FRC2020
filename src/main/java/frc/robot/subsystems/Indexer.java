/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  CANSparkMax mIndexerMotor;

  public Indexer() {
    mIndexerMotor = new CANSparkMax(41, MotorType.kBrushless);

  }

  public void setSpeed(double speed) {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}