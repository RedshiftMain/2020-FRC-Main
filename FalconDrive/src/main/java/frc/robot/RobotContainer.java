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
import frc.robot.commands.Shoot;
import frc.robot.commands.SimpleVisionTurn;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
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


  private final SimpleVisionTurn aim = new SimpleVisionTurn(drivetrain);
  
  private final Joystick stick = new Joystick(0);

  public RobotContainer() 
  {
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(stick.getRawAxis(1), stick.getRawAxis(4)), drivetrain));
    configureButtonBindings();
  }

  private void configureButtonBindings() 
  {
    JoystickButton a, b, x, y;

    a = new JoystickButton(stick, 1);

    b = new JoystickButton(stick, 2);
    x = new JoystickButton(stick, 3);
    y = new JoystickButton(stick, 4);

    a.toggleWhenPressed(aim);
    //b.whenPressed(() -> shooter.shoot(1.0)).whenReleased(() -> shooter.shoot(0.0));
    //x.whenPressed(() -> shooter.shoot(0.75)).whenReleased(() -> shooter.shoot(0.0));
    //y.whenPressed(() -> shooter.shoot(0.5)).whenReleased(() -> shooter.shoot(0.0));
    b.toggleWhenPressed(new Shoot(shooter, 0.9));
    x.toggleWhenPressed(new Shoot(shooter, 0.8));
    y.toggleWhenPressed(new Shoot(shooter, 0.7));

    b.toggleWhenPressed(new PerpetualCommand(new InstantCommand(() -> shooter.shoot(0.9), shooter)));
    x.toggleWhenPressed(new PerpetualCommand(new InstantCommand(() -> shooter.shoot(0.9), shooter)));
    y.toggleWhenPressed(new PerpetualCommand(new InstantCommand(() -> shooter.shoot(0.9), shooter)));

    //start.whenPressed(() -> drivetrain.reset());
  }

  public Command getAutonomousCommand() 
  {
    
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.AutoConstants.ksVolts,
                                       Constants.AutoConstants.kvVoltSecondsPerMeter,
                                       Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
                                       Constants.AutoConstants.kDriveKinematics,
                                       10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                             Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.AutoConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(.01, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drivetrain::getPose,
        new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.AutoConstants.ksVolts,
                                   Constants.AutoConstants.kvVoltSecondsPerMeter,
                                   Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
        Constants.AutoConstants.kDriveKinematics,
        drivetrain::getWheelSpeeds,
        new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
        new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        drivetrain::tankDriveVolts,
        drivetrain
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }
}
