/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  CANSparkMax mIndexerMotor;
  CANifier mCanifier;

  public Indexer() {
    mIndexerMotor = new CANSparkMax(41, MotorType.kBrushless);

    mCanifier = new CANifier(42);

    mIndexerMotor.burnFlash();

    mIndexerMotor.setInverted(true);

    mIndexerMotor.setSmartCurrentLimit(15);
  }

  public void setSpeed(double speed) {
    mIndexerMotor.set(speed);
  }

  public boolean getStartEye(){
    return mCanifier.getGeneralInput(CANifier.GeneralPin.SPI_MISO_PWM2P);
  }

  public boolean getEndEye() {
    return mCanifier.getGeneralInput(CANifier.GeneralPin.SPI_MOSI_PWM1P);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
