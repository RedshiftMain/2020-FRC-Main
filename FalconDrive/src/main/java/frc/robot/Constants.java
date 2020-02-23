/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    public static final class PortConstants
    {
        public static final int lMainFalcon = 1;
        public static final int rMainFalcon = 3;
        public static final int lSubFalcon  = 2;
        public static final int rSubFalcon  = 4;

        public static final int intake    = 5;
        public static final int magazine  = 6;
        public static final int rFeeder   = 7;
        public static final int lFeeder   = 8;
        public static final int elevator1 = 9;
        public static final int elevator2 = 10;
        public static final int lShooter  = 11;
        public static final int rShooter  = 12;
        public static final int spinner   = 13;

        public static final int beamSensor   = 14;
        public static final int fPiston  = 0;
        public static final int rPiston  = 1;
    }
    public static class VisionConstants
    {
        public static final double kP = 0.05;
        public static final double kI = 0.001;

        public static final double minThreshold = .01;
        public static final double maxSteer     = .5;
    }
    public static class AutoConstants
    {
        public static final double ksVolts = .193;
        public static final double kvVoltSecondsPerMeter = 2.93;
        public static final double kaVoltSecondsSquaredPerMeter = 0.245;
        public static final double kPDriveVel = .00293;
        //possible 42, 0.00293, 0.492, 0.25
        public static final double kTrackwidthMeters = 0.641241342;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        //revolutions per count * wheel diameter (meters) / gear ratio
        public static final double distancePerPulse = (1.0 / 2048) * (.1524 * Math.PI) / 12.92;;

        //deprecated maybe
        public static final double driveThreshold = 100.0;
    }

    public static class SpeedConstants
    {
        public static final double driveSpeed = 0.8;

        public static final double minShootSpeed = 0.9;
        public static final double maxShootSpeed = 1.0;
        //needs to be experimentally found
        public static final double minArea = 0;
        public static final double maxArea = 1.0;

        public static final double magazineSpeed = 0.5;
        public static final double intakeSpeed = 0.8;
        public static final double feederSpeed = -1.0;
        public static final double elevatorSpeed = 0.5;
        public static final double spinnerSpeed = 0.5;

        public static final double rampSpeed = 0.5;
        public static final double driveRampSpeed = 0.25;
        public static final double autoDriveRampSpeed = 5.0;
    }
}
