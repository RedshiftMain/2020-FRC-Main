/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MotionMagicConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SpeedConstants;

public class AutoDrivetrain extends SubsystemBase 
{  
  private final WPI_TalonFX lMainFalcon = new WPI_TalonFX(PortConstants.lMainFalcon);
  private final WPI_TalonFX rMainFalcon = new WPI_TalonFX(PortConstants.rMainFalcon);
  private final WPI_TalonFX lSubFalcon = new WPI_TalonFX(PortConstants.lSubFalcon);
  private final WPI_TalonFX rSubFalcon = new WPI_TalonFX(PortConstants.rSubFalcon);

  private final AHRS gyro = new AHRS(Port.kMXP);

  public AutoDrivetrain()
  {
    lMainFalcon.configFactoryDefault();

    lMainFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    lMainFalcon.setNeutralMode(NeutralMode.Brake);
    rMainFalcon.setNeutralMode(NeutralMode.Brake);

    rMainFalcon.follow(lMainFalcon);
    rMainFalcon.setInverted(TalonFXInvertType.FollowMaster);
    lSubFalcon.follow(lMainFalcon);
    lSubFalcon.setInverted(TalonFXInvertType.FollowMaster);
    rSubFalcon.follow(rMainFalcon);
    rSubFalcon.setInverted(TalonFXInvertType.FollowMaster);

    lMainFalcon.configOpenloopRamp(SpeedConstants.driveRampSpeed);
    lMainFalcon.configClosedloopRamp(SpeedConstants.autoDriveRampSpeed);

    lMainFalcon.configMotionCruiseVelocity(MotionMagicConstants.kCruiseVelocity);
    lMainFalcon.configMotionAcceleration(MotionMagicConstants.kMaxAcceleration);
    lMainFalcon.config_kP(0, MotionMagicConstants.kP);
    lMainFalcon.config_kI(0, MotionMagicConstants.kI);
    lMainFalcon.config_kD(0, MotionMagicConstants.kD);
    lMainFalcon.config_kF(0, MotionMagicConstants.kF);

    reset();
  }

  public void goTo(double dist)
  {
      lMainFalcon.set(ControlMode.MotionMagic, dist);
  }

  @Override
  public void periodic() 
  {
    
  }

  public void reset() 
  {
    gyro.reset();
    lMainFalcon.setSelectedSensorPosition(0, 0, 10);
  }
}
