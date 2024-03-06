/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125;

import com.revrobotics.CANSparkLowLevel.MotorType;

public final class Constants {
    public static final class HID {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class DriveTrain {
        public enum ControlType {
            ARCADE,
            ARCADE_ROCKETLEAGUE,
            CURVE,
            CURVE_ROCKETLEAGUE;
        }

        public static final MotorType kMotorType = MotorType.kBrushed;
        public static final int kFrontLeftID = 1;
        public static final int kFrontRightID = 2;
        public static final int kBackLeftID = 3;
        public static final int kBackRightID = 4;
    }

    public static final class Intake {
        public static final MotorType kMotorType = MotorType.kBrushless;
        public static final int kIntakeID = 5;

        public static final boolean kInverted = false;

        public static final double kIntakeSpeed = 0.5;
        public static final double kFeedSpeed = 1D;
        public static final double kMaxOutakeSpeed = -0.25;
    }

    public static final class Shooter {
        public static final MotorType kMotorType = MotorType.kBrushless;
        public static final int kLeftID = 6;
        public static final int kRightID = 7;

        public static final boolean kInverted = true;

        public static final double kP = 0.0004;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0.00017;
        public static final double kPIDTolerance = 300; // rpm

        public static final double kShootRPM = 5700;
    }
}
