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
}
