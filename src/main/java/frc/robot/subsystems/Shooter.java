/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  CANSparkMax mWheelMotor;
  CANSparkMax mWheelMotor2;
  CANPIDController mPidController;
  CANEncoder mEncoder;
  Joystick mJoystick;

  public Double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    mWheelMotor = new CANSparkMax(61, MotorType.kBrushless);
    mWheelMotor2 = new CANSparkMax(62, MotorType.kBrushless);
  }
    public void setVelocity() {

    }

    public void ShooterInit() {
      mJoystick = new Joystick(0);

      mWheelMotor.restoreFactoryDefaults();
      mWheelMotor2.restoreFactoryDefaults();

      mPidController = mWheelMotor.getPIDController();
      mPidController = mWheelMotor2.getPIDController();

      mEncoder = mWheelMotor.getEncoder();
      mEncoder = mWheelMotor2.getEncoder();

      kP = 5e-5;
      kI = 1e-6;
      kD = 0.0;
      kIz = 0.0;
      kFF = 0.0;
      kMaxOutput = 1.0;
      kMinOutput = -1.0;
      maxRPM = 5700.0;

      mPidController.setP(kP);
      mPidController.setI(kI);
      mPidController.setD(kD);
      mPidController.setIZone(kIz);
      mPidController.setFF(kFF);
      mPidController.setOutputRange(kMinOutput, kMaxOutput);

      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIz);
      SmartDashboard.putNumber("Feed Forward", kFF);
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);

    }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Read PID coefficients from SmartDashboard
    Double p = SmartDashboard.getNumber("P Gain", 0);
    Double i = SmartDashboard.getNumber("I Gain", 0);
    Double d = SmartDashboard.getNumber("D Gain", 0);
    Double iz = SmartDashboard.getNumber("I Zone", 0);
    Double ff = SmartDashboard.getNumber("Feed Forward", 0);
    Double max = SmartDashboard.getNumber("Max Output", 0);
    Double min = SmartDashboard.getNumber("Min Output", 0);

    //If PID coeffiecients on SmartDashboard have changed, write new values to controller
    if ((p != kP)) {mPidController.setP(p); kP = p;}
    if ((i != kI)) {mPidController.setI(i); kI = i;}
    if ((d != kD)) {mPidController.setD(d); kD = d;}
    if ((iz != kIz)) {mPidController.setIZone(iz); kIz = iz;}
    if ((ff != kFF)) {mPidController.setFF(ff); kFF = ff;}
    if ((max != kMaxOutput) || (min != kMinOutput)) {mPidController.setOutputRange(min, max); kMinOutput = min; kMaxOutput = max;}

    Double setPoint = mJoystick.getY()*maxRPM;
    mPidController.setReference(setPoint, ControlType.kVelocity);

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", mEncoder.getVelocity());
    
  }
}
