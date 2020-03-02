/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.BetterElevator;
import frc.robot.subsystems.BetterShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.Constants.*;
import frc.robot.commands.Autonomous;

public class RobotContainer 
{
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Magazine magazine = new Magazine();
  private final Feeder feeder = new Feeder();
  private final BetterShooter shooter = new BetterShooter();
  private final BetterElevator elevator = new BetterElevator();

  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  public RobotContainer() 
  {
    configureDefaultCommands();
    configureButtonBindings();
    configureLimelight();
    configureShuffleboard();
  }

  private void configureDefaultCommands()
  {
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(driver.getRawAxis(1)*SpeedConstants.driveSpeed, -driver.getRawAxis(4)*SpeedConstants.driveSpeed), drivetrain));
  }

  private void configureButtonBindings() 
  {
    //intake on operator?
    JoystickButton dA, dB,
                   oA, oB, oY, oLB, oRB, oBACK;

    dA = new JoystickButton(driver, 1);
    dB = new JoystickButton(driver, 2);

    oA = new JoystickButton(operator, 1);
    oB = new JoystickButton(operator, 2);
    oY = new JoystickButton(operator, 4);
    oLB = new JoystickButton(operator, 5);
    oRB = new JoystickButton(operator, 6);
    oBACK = new JoystickButton(operator, 7);

    //driver:
    //a - vision turn and shoot
    //b - intake out and suck

    //operator:
    //back - kill all motors
    //rb - manually run intake
    //lb - outtake
    //a - manual shoot
    //b - bleh ball
    //y - brakes
  }

  private void configureLimelight()
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  private void configureShuffleboard()
  {
    
  }

  public Command getAutonomousCommand() 
  {
    return new Autonomous(drivetrain, shooter, magazine, feeder);
  }
}
