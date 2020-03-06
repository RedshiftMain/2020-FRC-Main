/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Auto extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;
  /**
   * @apiNote constructor for the auto class
   */
  public Auto(Drivetrain drive) 
  {
    drivetrain = drive;
    drivetrain.setRamp(SpeedConstants.autoDriveRampSpeed);
    addRequirements(drivetrain);
  }

  /**
   * @apiNote resets the drivetrain
   */
  @Override
  public void initialize() 
  {
    drivetrain.reset();
  }

  /**ihoji
   * @apiNote drives the robot backwards
   * @throws us into a problem because it doesn't do that
   */
  @Override
  public void execute() 
  {
    drivetrain.drive(-SpeedConstants.autoDriveSpeed, 0);
    //System.out.println(drivetrain.leftEncoder());
  }
  /**
   * @param boolean
   */
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.setRamp(SpeedConstants.driveRampSpeed);
    drivetrain.drive(0, 0);
  }
  /**
   * checks if the robot distance exceeded the drive distance
   * @return boolean
   */
  @Override
  public boolean isFinished() 
  {
    return Math.abs(drivetrain.leftEncoder()) > AutoConstants.driveDistance;
  }
}