/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
  public final Drivetrain mDrivetrain;
  public final DoubleSupplier mForward;
  public final DoubleSupplier mStrafe;
  public final DoubleSupplier mAzimuth;

  public Drive(Drivetrain Drivetrain, DoubleSupplier Forward, DoubleSupplier Strafe, DoubleSupplier Azimuth) {
    mDrivetrain = Drivetrain;
    mForward = Forward;
    mStrafe = Strafe;
    mAzimuth = Azimuth;
    addRequirements(mDrivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrivetrain.drive(mForward.getAsDouble(), mStrafe.getAsDouble(), mAzimuth.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
