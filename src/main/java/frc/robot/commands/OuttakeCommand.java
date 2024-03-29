/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;

public class OuttakeCommand extends CommandBase {
  public final Intake mOuttake;
  public final Indexer mIndexer;

  public OuttakeCommand(Intake Outtake, Indexer Indexer) {
    mOuttake = Outtake;
    addRequirements(mOuttake);

    mIndexer = Indexer;
    addRequirements(mIndexer);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mOuttake.setSpeed(-1.0);
    mIndexer.setSpeed(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mOuttake.setSpeed(0.0);
    mIndexer.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
