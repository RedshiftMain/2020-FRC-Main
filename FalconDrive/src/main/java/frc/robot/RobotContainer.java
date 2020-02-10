/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.Shoot;
import frc.robot.commands.VSimpleTurn;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SpinState;
import frc.robot.subsystems.Spinner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer 
{
  private final Drivetrain drivetrain = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final Feeder feeder = new Feeder();
  private final Intake intake = new Intake();
  private final Magazine magazine = new Magazine();
  private final Shooter shooter = new Shooter();
  private final Spinner spinner = new Spinner();
  
  private final Joystick stick = new Joystick(0);

  public RobotContainer() 
  {
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(stick.getRawAxis(1)*SpeedConstants.driveSpeed, stick.getRawAxis(4)*SpeedConstants.driveSpeed), drivetrain));
    configureButtonBindings();
  }

  private void configureButtonBindings() 
  {
    JoystickButton a, b, x, y, start, back, lb, rb, lt, rt;

    a = new JoystickButton(stick, 1);
    b = new JoystickButton(stick, 2);
    x = new JoystickButton(stick, 3);
    y = new JoystickButton(stick, 4);
    lb = new JoystickButton(stick, 5);
    rb = new JoystickButton(stick, 6);
    back = new JoystickButton(stick, 7);
    start = new JoystickButton(stick, 8);

    a.toggleWhenPressed(new VSimpleTurn(drivetrain));

    b.toggleWhenPressed(new PerpetualCommand(new InstantCommand(() -> shooter.shoot(1.00), shooter)));

    //back.whenPressed(new ConditionalCommand(null, null, spinner.getState() == Color));
  }

  public Command getAutonomousCommand() 
  {
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.AutoConstants.ksVolts,
                                   Constants.AutoConstants.kvVoltSecondsPerMeter,
                                   Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
                                   Constants.AutoConstants.kDriveKinematics,
                                   10);

    TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.AutoConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    RamseteCommand forward = createRamsete(TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(0.1, 0)),
      new Pose2d(0.2, 0, new Rotation2d(0)),
      config));
  
    return forward.andThen(() -> drivetrain.tankDriveVolts(0, 0)).andThen(() -> drivetrain.drive(0, 0));
  }

  private RamseteCommand createRamsete(Trajectory trajectory)
  {
    return new RamseteCommand(
              trajectory,
              drivetrain::getPose,
              new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
              new SimpleMotorFeedforward(Constants.AutoConstants.ksVolts,
                  Constants.AutoConstants.kvVoltSecondsPerMeter,
                  Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
                  Constants.AutoConstants.kDriveKinematics,
              drivetrain::getWheelSpeeds,
              new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
              new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
              drivetrain::tankDriveVolts,
              drivetrain
              );
  }
}
