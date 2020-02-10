/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VSimpleTurn extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;

  private double tx;
  private double tv;
  private double speed;
  private double steer;
  private double sum;

  public VSimpleTurn(Drivetrain drive) 
  {
    drivetrain = drive;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() 
  {
      sum = 0;
      speed = 0;
      steer = 0;
  }

  @Override
  public void execute() 
  {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    if(tv < 1.0)
    {
        drivetrain.drive(0, 0);
    }
    else
    {
        //P-Loop
        steer = tx*VisionConstants.kP+sum;
        //I-Loop
        sum += tx*VisionConstants.kI;

        drivetrain.drive(speed, Math.min(steer, VisionConstants.maxSteer));
    }
  }

  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.drive(0, 0);
  }

  @Override
  public boolean isFinished() 
  {
    return Math.abs(tx) < VisionConstants.minThreshold;
  }
}
