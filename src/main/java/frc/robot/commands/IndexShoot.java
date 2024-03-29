/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class IndexShoot extends CommandBase {
  Indexer mIndexer;
  Shooter mShooter;
  /**
   * Creates a new IndexShoot.
   */
  public IndexShoot(Indexer indexer, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIndexer = indexer;
    mShooter = shooter;

    addRequirements(mIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mShooter.isAtSpeed()){
      mIndexer.setSpeed(-1);
    } else {
      mIndexer.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIndexer.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
