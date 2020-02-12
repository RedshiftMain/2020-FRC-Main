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
        public static final int lSubFalcon = 2;
        public static final int rSubFalcon = 4;

        public static final int intake = 5;
        public static final int magazine = 6;
        public static final int feeder1 = 7;
        public static final int feeder2 = 8;
        public static final int elevator1 = 9;
        public static final int elevator2 = 10;
        public static final int lShooter = 11;
        public static final int rShooter = 12;
        public static final int spinner = 13;

        public static final int beamSensor = 14;
        public static final int fLeftPiston = 15;
        public static final int rLeftPiston = 16;
        public static final int fRightPiston = 17;
        public static final int rRightPiston = 18;
    }
    public static class VisionConstants
    {
        public static final double kP = 0.05;
        public static final double kI = 0.001;

        public static final double minThreshold = .05;
        public static final double maxSteer = .5;
    }
    public static class AutoConstants
    {
        public static final double ksVolts = .332;
        public static final double kvVoltSecondsPerMeter = 0.0116;
        public static final double kaVoltSecondsSquaredPerMeter = 0.000561;
        public static final double kPDriveVel = .0553;
        public static final double kTrackwidthMeters = 174.70918916202316;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 3.56;
        public static final double kMaxAccelerationMetersPerSecondSquared = 9.1;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double distancePerPulse = 0.0002337786720;
    }

    public static class SpeedConstants
    {
        public static final double driveSpeed = 1.0;
        public static final double shootSpeed = 1.0;
        public static final double magazineSpeed = 1.0;
        public static final double intakeSpeed = 1.0;
        public static final double feederSpeed = 1.0;
        public static final double elevatorSpeed = 1.0;
        public static final double spinnerSpeed = 1.0;
    }
}
