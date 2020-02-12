/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;

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
import frc.robot.commands.ASimple;
import frc.robot.commands.VSimpleTurn;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer 
{
  private final Drivetrain drivetrain = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final Feeder feeder = new Feeder();
  private final Intake intake = new Intake();
  private final Magazine magazine = new Magazine();
  private final Shooter shooter = new Shooter();
  private final Spinner spinner = new Spinner();
  
  private final Joystick stick1 = new Joystick(0);
  private final Joystick stick2 = new Joystick(1);

  public RobotContainer() 
  {
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(stick1.getRawAxis(1)*SpeedConstants.driveSpeed, stick1.getRawAxis(4)*SpeedConstants.driveSpeed), drivetrain));
    configureButtonBindings();
  }

  private void configureButtonBindings() 
  {
    JoystickButton a1, b1, x1, y1, start1, back1, lb1, rb1, lt1, rt1,
                   a2, b2, x2, y2, start2, back2, lb2, rb2, lt2, rt2;

    a1 = new JoystickButton(stick1, 1);
    b1 = new JoystickButton(stick1, 2);
    x1 = new JoystickButton(stick1, 3);
    y1 = new JoystickButton(stick1, 4);
    lb1 = new JoystickButton(stick1, 5);
    rb1 = new JoystickButton(stick1, 6);
    back1 = new JoystickButton(stick1, 7);
    start1 = new JoystickButton(stick1, 8);
    rt1 = new JoystickButton(stick1, 9);

    a2 = new JoystickButton(stick2, 1);
    b2 = new JoystickButton(stick2, 2);
    x2 = new JoystickButton(stick2, 3);
    y2 = new JoystickButton(stick2, 4);
    lb2 = new JoystickButton(stick2, 5);
    rb2 = new JoystickButton(stick2, 6);
    back2 = new JoystickButton(stick2, 7);
    start2 = new JoystickButton(stick2, 8);
    
    Command shoot = new RunCommand(() -> shooter.shoot(SpeedConstants.shootSpeed), shooter).andThen(() -> shooter.stop());
    Command runMag = new RunCommand(() -> magazine.load(), magazine).andThen(() -> magazine.stop());
    Command runFeeder = new RunCommand(() -> feeder.feed(), feeder).andThen(() -> feeder.stop());
    Command transferBall = runMag.alongWith(runFeeder);
    Command shootBall = shoot.alongWith(transferBall);
    Command loadSequence = new ConditionalCommand(runMag, transferBall, feeder::hasBall);

    a1.toggleWhenPressed(new VSimpleTurn(drivetrain));
    rb1.toggleWhenPressed(new RunCommand(() -> intake.intake()));
    rt1.toggleWhenPressed(new RunCommand(() -> intake.outtake()));

    back2.toggleWhenPressed(loadSequence);
    start2.toggleWhenPressed(shootBall);

    rb2.toggleWhenPressed(new RunCommand(() -> elevator.up()));
    lb2.toggleWhenPressed(new RunCommand(() -> elevator.down()));
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
  
    //return forward.andThen(() -> drivetrain.tankDriveVolts(0, 0)).andThen(() -> drivetrain.drive(0, 0));
    return new ASimple(drivetrain, 100);
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
