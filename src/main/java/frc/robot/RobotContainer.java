/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Limelight mLimelight = new Limelight();
  Drivetrain mDrivetrain = new Drivetrain();
  Climber mClimber = new Climber();
  Intake mIntake = new Intake();
  Shooter mShooter = new Shooter();

  XboxController mDriverController = new XboxController(0);
  XboxController mOperatorController = new XboxController(1);


    
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton ManualShootButton = new JoystickButton(mOperatorController,6);
    JoystickButton ManualShootButton2 = new JoystickButton(mOperatorController, 0);
    JoystickButton ManualShootButton3 = new JoystickButton(mOperatorController, 0);
    JoystickButton IntakeButton = new JoystickButton(mOperatorController, 2);
    JoystickButton OuttakeButton = new JoystickButton(mOperatorController, 5);

    mDrivetrain.setDefaultCommand(new DriveCommand(mDrivetrain, mDriverController));
    mClimber.setDefaultCommand(new ManualHangCommand(mClimber, mOperatorController));
    ManualShootButton.whileHeld(new ManualShootCommand(mShooter));
    ManualShootButton2.whileHeld(new ManualShootCommand2(mShooter));
    ManualShootButton3.whileHeld( new ManualShootCommand3(mShooter));
    IntakeButton.whileHeld(new IntakeCommand(mIntake));
    OuttakeButton.whileHeld(new OuttakeCommand(mIntake));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
