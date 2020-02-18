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

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  CANSparkMax mLiftMotor1;
  CANSparkMax mLiftMotor2;
  CANSparkMax mHookMotor;

  CANEncoder mCanEncoder;
  CANPIDController mPidController;

  Preferences mRobotPreferences = Preferences.getInstance();

  public Double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  public enum ClimberPosition{
    Open(),
    Closed();

    private final double Height;

    ClimberPosition(double state){
      Height = state;
    }

    private double getHeight() {
      return Height;
    }

  }

  /**
   * Creates a new Climber.
   */
  public Climber() {
    mLiftMotor1 = new CANSparkMax(31, MotorType.kBrushless);
    mLiftMotor2 = new CANSparkMax(32, MotorType.kBrushless);
    mHookMotor = new CANSparkMax(33, MotorType.kBrushless);

    mCanEncoder = mLiftMotor1.getEncoder();
    mCanEncoder = mLiftMotor2.getEncoder();
    mPidController = mLiftMotor1.getPIDController();
    mPidController = mLiftMotor2.getPIDController();

    mLiftMotor2.follow(mLiftMotor1);

    kP = mRobotPreferences.getDouble("ClimberKP", 5e-5);
    kI = mRobotPreferences.getDouble("ClimberKI", 1e-6);
    kD = mRobotPreferences.getDouble("ClimberKD", 0.0);
    kIz = mRobotPreferences.getDouble("ClimberKIz", 0.0);
    kFF = mRobotPreferences.getDouble("ClimberKFF", 0.0);
    kMaxOutput = mRobotPreferences.getDouble("ClimberKMaxOutput", 1.0);
    kMinOutput = mRobotPreferences.getDouble("ClimberKMinOutput", -1.0);
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

  public void setOpenLoop(double speed) {
    mLiftMotor1.set(speed);
    
  }

  public void setPosition(double position) {
   mPidController.setReference(position, ControlType.kPosition);
    
  }

  public void setStrafe(double speed) {
    mHookMotor.set(speed);
    
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

  }
}
