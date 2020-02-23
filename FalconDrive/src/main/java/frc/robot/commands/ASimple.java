/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

//PROBABLY DEPRECATED
public class ASimple extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;

  private double speed = 0;
  private double error = 0;
  private double sum = 0;
  private final double desiredDist;

  public ASimple(Drivetrain drive, double dist) 
  {
    drivetrain = drive;
    desiredDist = dist;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() 
  {
    drivetrain.reset();
    
  }

  @Override
  public void execute() 
  {
    error = desiredDist-drivetrain.leftEncoder();
    speed = .00006*error+sum;

    sum += .000000 * error;

    SmartDashboard.putNumber("Speed", speed);
    SmartDashboard.putNumber("Sum", sum);

    drivetrain.drive(speed, 0);
  }

  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.drive(0, 0);
  }

  @Override
  public boolean isFinished() 
  {
    return Math.abs(error) < AutoConstants.driveThreshold;
  }
}
