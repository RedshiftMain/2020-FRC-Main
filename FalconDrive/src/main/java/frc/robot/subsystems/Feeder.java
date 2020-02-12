/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SpeedConstants;

public class Feeder extends SubsystemBase 
{
  private final WPI_VictorSPX lFeeder = new WPI_VictorSPX(PortConstants.feeder1);
  private final WPI_VictorSPX rFeeder = new WPI_VictorSPX(PortConstants.feeder2);

  private final DigitalInput beamBreak = new DigitalInput(PortConstants.beamSensor);

  private boolean holdingBall = beamBreak.get();

  public Feeder()
  {
    rFeeder.follow(lFeeder);
    rFeeder.setInverted(InvertType.OpposeMaster);
  }

  public void feed()
  {
    lFeeder.set(ControlMode.PercentOutput, SpeedConstants.feederSpeed);
  }

  public void stop()
  {
    lFeeder.set(ControlMode.PercentOutput, 0);
  }

  public boolean hasBall()
  {
    return holdingBall;
  }

  @Override
  public void periodic() 
  {
    holdingBall = beamBreak.get();
  }
}
