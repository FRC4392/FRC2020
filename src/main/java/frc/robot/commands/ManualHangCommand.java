/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ManualHangCommand extends CommandBase {
  public final Climber mClimber;
  public XboxController mController;
  
  public ManualHangCommand(Climber Climber, XboxController Controller) {
    mClimber = Climber;
    mController = Controller;
    addRequirements(mClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0;
    if (Math.abs(mController.getY(Hand.kLeft)) > 0.1) {
      speed = Math.pow(mController.getY(Hand.kLeft), 2);
      if (mController.getY(Hand.kLeft) < 0) {
        speed *= -1;
      }
    }
    mClimber.setOpenLoop(speed * -1);

    double strafe = 0;
    if (Math.abs(mController.getY(Hand.kRight)) > 0.1) {
      strafe = Math.pow(mController.getY(Hand.kRight), 2);
      if (mController.getY(Hand.kLeft) < 0) {
        strafe *= -1;
      }
    }
    mClimber.setStrafe(strafe);
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
