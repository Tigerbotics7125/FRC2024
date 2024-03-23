/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.subsystems;

import static io.github.tigerbotics7125.Constants.DriveTrain.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.Constants;
import io.github.tigerbotics7125.lib.REVUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {
  // Drive Motors
  private CANSparkMax m_frontLeft = new CANSparkMax(kFrontLeftID, kMotorType);
  private CANSparkMax m_frontRight = new CANSparkMax(kFrontRightID, kMotorType);
  private CANSparkMax m_backLeft = new CANSparkMax(kBackLeftID, kMotorType);
  private CANSparkMax m_backRight = new CANSparkMax(kBackRightID, kMotorType);

  // Encoder Talons
  private WPI_TalonSRX m_leftEncoder = new WPI_TalonSRX(kLeftTalonID);
  private WPI_TalonSRX m_rightEncoder = new WPI_TalonSRX(kRightTalonID);

  // NavX Gyroscope
  public AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry / Kinematics
  private DifferentialDriveOdometry m_odometry =
      new DifferentialDriveOdometry(
          m_gyro.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());
  private DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(Constants.DriveTrain.kTrackWidth);

  // Path following PID Controllers
  private PIDController m_leftPID = new PIDController(kP, kI, kD);
  private PIDController m_rightPID = new PIDController(kP, kI, kD);

  // Field2d visualization
  private Field2d m_field = new Field2d();

  public Drivetrain() {
    configureMotor(m_frontLeft);
    configureMotor(m_frontRight);
    configureMotor(m_backLeft);
    configureMotor(m_backRight);

    // Make sure encoders are selected.
    m_leftEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_rightEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // Set followers, using retry since they've failed before.
    REVUtil.retryFailable(5, () -> m_backLeft.follow(m_frontLeft));
    REVUtil.retryFailable(5, () -> m_backRight.follow(m_frontRight));

    // Set inversions.
    m_frontLeft.setInverted(kLeftInverted);
    m_leftEncoder.setInverted(kLeftInverted);

    // Default PP AutoBuilder
    AutoBuilder.configureRamsete(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::setChassisSpeeds,
        new ReplanningConfig(),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  private void configureMotor(CANSparkMax motor) {
    motor.restoreFactoryDefaults();
    Timer.delay(.02);

    motor.setSmartCurrentLimit(Constants.DriveTrain.kCurrentLimit);
    motor.setIdleMode(IdleMode.kCoast);
    motor.burnFlash();
    Timer.delay(.02);
  }

  public Command arcadeDrive(
      DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier squareInputs) {
    return run(
        () -> {
          WheelSpeeds ws =
              DifferentialDrive.arcadeDriveIK(
                  xSpeed.getAsDouble(), zRotation.getAsDouble(), squareInputs.getAsBoolean());
          m_frontLeft.set(ws.left);
          m_frontRight.set(ws.right);
        });
  }

  public Command curvatureDrive(
      DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier allowTurnInPlace) {
    return run(
        () -> {
          WheelSpeeds ws =
              DifferentialDrive.curvatureDriveIK(
                  xSpeed.getAsDouble(), zRotation.getAsDouble(), allowTurnInPlace.getAsBoolean());
          m_frontLeft.set(ws.left);
          m_frontRight.set(ws.right);
        });
  }

  public Command setIdleMode(IdleMode idleMode) {
    return runOnce(
            () -> {
              m_frontLeft.setIdleMode(idleMode);
              m_frontRight.setIdleMode(idleMode);
              m_backLeft.setIdleMode(idleMode);
              m_backRight.setIdleMode(idleMode);
            })
        .ignoringDisable(true);
  }

  private double getLeftPositionMeters() {
    return m_leftEncoder.getSelectedSensorPosition()
        * Constants.DriveTrain.kPositionConversionFactor;
  }

  private double getRightPositionMeters() {
    return m_rightEncoder.getSelectedSensorPosition()
        * Constants.DriveTrain.kPositionConversionFactor;
  }

  private double getLeftVelocityMetersPerSecond() {
    return m_leftEncoder.getSelectedSensorVelocity()
        * Constants.DriveTrain.kVelocityConversionFactor;
  }

  private double getRightVelocityMetersPerSecond() {
    return m_rightEncoder.getSelectedSensorVelocity()
        * Constants.DriveTrain.kVelocityConversionFactor;
  }

  private Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  private ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond()));
  }

  private void setPose(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters(), pose);
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    DifferentialDriveWheelSpeeds ws = m_kinematics.toWheelSpeeds(chassisSpeeds);

    double leftPID = m_leftPID.calculate(getLeftVelocityMetersPerSecond(), ws.leftMetersPerSecond);
    double rightPID =
        m_rightPID.calculate(getRightVelocityMetersPerSecond(), ws.rightMetersPerSecond);

    m_frontLeft.set(leftPID);
    m_frontRight.set(rightPID);

    SmartDashboard.putNumber("Left Velocity Setpoint m/s", ws.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Velocity Setpoint m/s", ws.rightMetersPerSecond);
    SmartDashboard.putNumber("Omega Velocity Setpoint rad/s", chassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Left PID Contribution", leftPID);
    SmartDashboard.putNumber("Right PID Contribution", rightPID);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Velocity m/s", getLeftVelocityMetersPerSecond());
    SmartDashboard.putNumber("Right Velocity m/s", getRightVelocityMetersPerSecond());
    SmartDashboard.putNumber("Omega Velocity rad/s", Units.degreesToRadians(-m_gyro.getRate()));
    SmartDashboard.putNumber("Left Position m", getLeftPositionMeters());
    SmartDashboard.putNumber("Right Position m", getRightPositionMeters());

    m_odometry.update(m_gyro.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());
    m_field.setRobotPose(getPose());
  }
}
