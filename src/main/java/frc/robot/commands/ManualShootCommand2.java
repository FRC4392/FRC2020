/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.HoodPosition;

public class ManualShootCommand2 extends CommandBase {
 public final Shooter mShooter;

  public ManualShootCommand2(Shooter Shooter) {
    mShooter = Shooter;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShooter.setHood(HoodPosition.Open);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.setPIDVelocity(3600);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
   mShooter.setVelocity(0.0);
   mShooter.setHood(HoodPosition.Closed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
