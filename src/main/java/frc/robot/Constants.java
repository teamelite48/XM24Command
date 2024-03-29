// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

public final class Constants {
    public static final class DriveConstants {
        public static final class MotorChannels {
            public static final int LEFT_FRONT = 3;
            public static final int LEFT_REAR = 2;
            public static final int RIGHT_FRONT = 0;
            public static final int RIGHT_REAR = 1;
        }

        public static final class EncoderChannels {
            public static final int LEFT_A = 0;
            public static final int LEFT_B = 1;
            public static final int RIGHT_A = 2;
            public static final int RIGHT_B = 3;
        }

        public static final class FeedForwardGains {
            public static final double ksVolts = 0.22;
            public static final double kvVoltSecondsPerMeter = 1.98;
            public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        }

        public static final class FeedbackGains {
            public static final double kPDriveVel = 8.5;
        }

        public static final double TrackWidthMeters = 0.69;
        public static final DifferentialDriveKinematics DriveKinematics = new DifferentialDriveKinematics(TrackWidthMeters);

        public static final double MaxSpeedMetersPerSecond = 3;
        public static final double MaxAccelerationMetersPerSecondSquared = 3;

        public static final double RamseteB = 2;
        public static final double RamseteZeta = 0.7;

        public static final double SIMULATION_REFRESH_RATE = 0.02;
        public static final int ENCODER_RESOLUTION = -4096;
        public static final int GYRO_CHANNEL = 0;

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);

        public static final double SLOW_SPEED = 0.5;
        public static final double FAST_SPEED = 1.0;
    }

    public static final class OIConstants {
        public static final int PILOT_JOYSTICK_PORT = 0;
    }
}
