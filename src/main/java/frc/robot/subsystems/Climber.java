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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private CANSparkMax mLiftMotor1;
    private CANSparkMax mLiftMotor2;
    private CANSparkMax mHookMotor;

    private CANEncoder mCanEncoder;
    private CANPIDController mPidController;

    private Solenoid mSolenoid = new Solenoid(2);

    private Preferences mRobotPreferences = Preferences.getInstance();

    private Double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, targetPosition, targetSpeed;

    private ClimberState mState;

    public enum ClimberState {
      ManualMove,
      ManualStop,
      Positioning,
      PositionHold,
      Stopped;
    }

    public enum ClimberPosition {
        Down(0.0),
        BarHeight(0.0),
        ClimbHeight(0.0),
        WheelOfFortune(0.0);

        private final double Height;

        ClimberPosition(double state) {
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

        mLiftMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mLiftMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        mLiftMotor1.burnFlash();
        mLiftMotor2.burnFlash();

    }

    public void setOpenLoop(double speed) {
        mLiftMotor1.set(speed);
        mSolenoid.set(true);
    }

    public void setManual(double speed){

    }

    public void setHeight(double height) {
        targetSpeed = 0.0;
        targetPosition = height;
        mPidController.setReference(height, ControlType.kPosition, 0, 0.0, CANPIDController.ArbFFUnits.kVoltage);
    }

    public void setPostion(ClimberPosition position) {
        setHeight(position.getHeight());
    }

    public void setStrafe(double speed) {
        mHookMotor.set(speed);
    }

    @Override
    public void periodic(){
      /**double currPos = mCanEncoder.getPosition();
      double error = currPos - targetPosition;
      double currVelocity = mCanEncoder.getVelocity();

      switch (mState){
        case Stopped:
          if (targetSpeed > 0.0){
            mSolenoid.set(false);
            mState = ClimberState.ManualStop;
          }
        case ManualMove:
        case ManualStop:
        case Positioning:
        case PositionHold:**/
      }
    }
