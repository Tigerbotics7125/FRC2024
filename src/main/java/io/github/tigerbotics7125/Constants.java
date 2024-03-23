/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

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

    public static final int kCurrentLimit = 60;

    public static final int kFrontLeftID = 1;
    public static final int kFrontRightID = 2;
    public static final int kBackLeftID = 3;
    public static final int kBackRightID = 4;

    public static final int kLeftTalonID = 1;
    public static final int kRightTalonID = 2;

    public static final boolean kLeftInverted = true;

    public static final double kMaxLinearVelocity = Units.feetToMeters(12.98); // meters per second

    public static final double kPositionConversionFactor =
        1D / 4096D * 2D * Math.PI * Units.inchesToMeters(3D);
    public static final double kVelocityConversionFactor =
        1D / 100D * 1000D * kPositionConversionFactor;

    public static final double kTrackWidth = Units.inchesToMeters(21);

    public static final double kP = 1.75;
    public static final double kI = 0D;
    public static final double kD = 0D;
  }

  public static final class Intake {
    public static final MotorType kMotorType = MotorType.kBrushless;
    public static final int kIntakeID = 5;

    public static final int kCurrentLimit = 25;

    public static final boolean kInverted = false;

    public static final double kIntakeSpeed = 0.5;
    public static final double kFeedSpeed = 1D;
    public static final double kMaxOutakeSpeed = -0.25;
  }

  public static final class Shooter {
    public static final MotorType kMotorType = MotorType.kBrushless;
    public static final int kLeftID = 6;
    public static final int kRightID = 7;

    public static final int kCurrentLimit = 90;

    public static final boolean kInvertedFollower = true;

    public static final double kP = 0.0004;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0.00017;
    public static final double kPIDTolerance = 300; // rpm

    public static final double kShootRPM = 5700;
  }

  public static final class Arm {
    public static final MotorType kMotorType = MotorType.kBrushless;
    public static final int kLeftID = 8;
    public static final int kRightID = 9;

    public static final int kCurrentLimit = 50;

    public static final boolean kFollowerInverted = true;

    public static final double kP = .01;
    public static final double kI = 0; // Integral term should be done with FF instead.
    public static final double kD = 0; // .5;
    public static final PIDController kPID = new PIDController(kP, kI, kD);
    // TODO look into sysid to characterize this.
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final ArmFeedforward kFF = new ArmFeedforward(kS, kG, kV);

    public static final double kGearRatio = 1D / 48D;
    public static final double kChainRatio = 10D / 60D;

    public static final double kPositionConversionFactor = kGearRatio * kChainRatio * 360;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60D;

    public enum ArmState {
      AMP(-82D),
      SPEAKER(-10D),
      SPEAKERAUTO(-10D),
      INTAKE(0D);

      public final double kPosition;

      ArmState(double position) {
        kPosition = position;
      }
    }
  }
}
