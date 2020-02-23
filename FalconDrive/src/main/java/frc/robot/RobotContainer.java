/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer 
{
  private final Drivetrain drivetrain = new Drivetrain();
  //needs to be implemented
  //private final Elevator elevator = new Elevator();
  //private final Feeder feeder = new Feeder();
  //private final Intake intake = new Intake();
  //private final Magazine magazine = new Magazine();
  private final Shooter shooter = new Shooter();
  //might need to be implemented
  //private final Spinner spinner = new Spinner();
  
  private final Joystick stick1 = new Joystick(0);
  private final Joystick stick2 = new Joystick(1);

  //shoots the ball from line
  /*Command shootSequence = new InstantCommand(() -> shooter.shoot(), shooter)
                            .andThen(new WaitUntilCommand(() -> shooter.atSpeed()))
                            .andThen(() -> feeder.feed(), feeder)
                            .andThen(() -> magazine.load(), magazine);

  //shoots the ball from trench
  Command shootFastSequence = new InstantCommand(() -> shooter.shootFast(), shooter)
                            .andThen(new WaitUntilCommand(() -> shooter.atSpeed()))
                            .andThen(() -> feeder.feed(), feeder)
                            .andThen(() -> magazine.load(), magazine);

  //stops all motors except drivetrain
  Command kill = new InstantCommand(() -> intake.stop(), intake)
                  .andThen(() -> shooter.stop(), shooter)
                  .andThen(() -> magazine.stop(), magazine)
                  .andThen(() -> feeder.stop(), feeder);*/

  //centers on vision target
  //Command visionTrack = new VSimpleTurn(drivetrain);

  public RobotContainer() 
  {
    //drive command
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(stick1.getRawAxis(4)*SpeedConstants.driveSpeed, -stick1.getRawAxis(1)*SpeedConstants.driveSpeed), drivetrain));
    configureButtonBindings();
  }

  private void configureButtonBindings() 
  {
    JoystickButton a1, b1, x1, y1, start1, back1, lb1, rb1, lt1,
                   a2, b2, x2, y2, start2, back2, lb2, rb2, lt2;

    a1 = new JoystickButton(stick1, 1);
    b1 = new JoystickButton(stick1, 2);
    x1 = new JoystickButton(stick1, 3);
    y1 = new JoystickButton(stick1, 4);
    lb1 = new JoystickButton(stick1, 5);
    rb1 = new JoystickButton(stick1, 6);
    back1 = new JoystickButton(stick1, 7);
    start1 = new JoystickButton(stick1, 8);

    a2 = new JoystickButton(stick2, 1);
    b2 = new JoystickButton(stick2, 2);
    x2 = new JoystickButton(stick2, 3);
    y2 = new JoystickButton(stick2, 4);
    lb2 = new JoystickButton(stick2, 5);
    rb2 = new JoystickButton(stick2, 6);
    back2 = new JoystickButton(stick2, 7);
    start2 = new JoystickButton(stick2, 8);

    /*x1.whenPressed(shootFastSequence).whenReleased(kill);
    lb1.whenPressed(shootSequence).whenReleased(kill);
    rb1.toggleWhenPressed(new StartEndCommand(() -> intake.outtake(), () -> intake.stop(), intake));
    a1.toggleWhenPressed(new VSimpleTurn(drivetrain));
    back1.toggleWhenPressed(new StartEndCommand(() -> intake.unploy(), () -> intake.deploy(), intake));
    start1.whenPressed(kill);*/

    //needs to be tested
    //shoots the ball from trench
    
    
    //testing commands only
    b1.toggleWhenPressed(new StartEndCommand(() -> shooter.shootFast(), () -> shooter.stop(), shooter));
    //y1.toggleWhenPressed(new StartEndCommand(() -> magazine.load(), () -> magazine.stop(), magazine));
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

    //simple forward command?
    Trajectory forward = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(0.25, 0)),
      new Pose2d(0.5, 0, new Rotation2d(0)),
      config);

    //translates trajectory to current robot pose
    RamseteCommand aForward = createRamsete(forward.relativeTo(drivetrain.getPose()));
  
    //runs trajectory, stops robot, aims with vision, shoots
    return aForward.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    //return aForward.andThen(aBackward).andThen(() -> drivetrain.tankDriveVolts(0, 0));
    //return auto.andThen(() -> drivetrain.tankDriveVolts(0, 0)).andt;//.andThen(new VSimpleTurn(drivetrain));//.andThen(shootSequence);
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
