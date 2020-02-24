/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwervePod;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax mDriveMotor1 = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax mDriveMotor2 = new CANSparkMax(12, MotorType.kBrushless);
  CANSparkMax mDriveMotor3 = new CANSparkMax(13, MotorType.kBrushless);
  CANSparkMax mDriveMotor4 = new CANSparkMax(14, MotorType.kBrushless);

  CANSparkMax mAzimuth1 = new CANSparkMax(21, MotorType.kBrushless);
  CANSparkMax mAzimuth2 = new CANSparkMax(22, MotorType.kBrushless);
  CANSparkMax mAzimuth3 = new CANSparkMax(23, MotorType.kBrushless);
  CANSparkMax mAzimuth4 = new CANSparkMax(24, MotorType.kBrushless);

  CANCoder mCanCoder1 = new CANCoder(11);
  CANCoder mCanCoder2 = new CANCoder(12);
  CANCoder mCanCoder3 = new CANCoder(13);
  CANCoder mCanCoder4 = new CANCoder(14);

  PigeonIMU pidgey = new PigeonIMU(10);

  private final double mTrackWidth = 30.0;
  private final double mWheelBase = 30.0;

  SwervePod pod1 = new SwervePod(mDriveMotor1, mAzimuth1, mCanCoder1);
  SwervePod pod2 = new SwervePod(mDriveMotor2, mAzimuth2, mCanCoder2);
  SwervePod pod3 = new SwervePod(mDriveMotor3, mAzimuth3, mCanCoder3);
  SwervePod pod4 = new SwervePod(mDriveMotor4, mAzimuth4, mCanCoder4);

  SwerveDrive swerveDrive = new SwerveDrive(pidgey, mTrackWidth, mWheelBase, new SwervePod[]{pod2, pod1, pod4, pod3}, true);

  SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(new Translation2d(-15,15), new Translation2d(15, 15), new Translation2d(-15, -15), new Translation2d(15,-15));
  SwerveDriveOdometry driveOdometry = new SwerveDriveOdometry(driveKinematics, Rotation2d.fromDegrees(pidgey.getFusedHeading()));

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {

  }

  public void drive(double forward, double strafe, double azimuth){
    swerveDrive.drive(forward, strafe, azimuth);
  }

  public void stop(){
    swerveDrive.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wheel1Position", pod1.getIncrementalPosition());
    SmartDashboard.putNumber("Wheel2Position", pod2.getIncrementalPosition());
    SmartDashboard.putNumber("Wheel3Position", pod3.getIncrementalPosition());
    SmartDashboard.putNumber("Wheel4Position", pod4.getIncrementalPosition());

    SmartDashboard.putNumber("Wheel1Setpoint",pod1.getSetpoint());
    SmartDashboard.putNumber("Wheel2Setpoint",pod2.getSetpoint());
    SmartDashboard.putNumber("Wheel3Setpoint",pod3.getSetpoint());
    SmartDashboard.putNumber("Wheel4Setpoint",pod4.getSetpoint());

    /*driveOdometry.update(swerveDrive.getHeading(), pod1.getState(), pod2.getState(), pod3.getState(), pod4.getState());

    SmartDashboard.putNumber("Swerve Y", driveOdometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Swerve X", driveOdometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Swerve Rotation", driveOdometry.getPoseMeters().getRotation().getDegrees());*/
  }
}
