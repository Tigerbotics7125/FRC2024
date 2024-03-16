/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.Constants;

public class Drivetrain extends SubsystemBase {
    private CANSparkMax frontLeft =
            new CANSparkMax(Constants.DriveTrain.kFrontLeftID, Constants.DriveTrain.kMotorType);
    private CANSparkMax frontRight =
            new CANSparkMax(Constants.DriveTrain.kFrontRightID, Constants.DriveTrain.kMotorType);
    private CANSparkMax backLeft =
            new CANSparkMax(Constants.DriveTrain.kBackLeftID, Constants.DriveTrain.kMotorType);
    private CANSparkMax backRight =
            new CANSparkMax(Constants.DriveTrain.kBackRightID, Constants.DriveTrain.kMotorType);
    private WPI_TalonSRX m_leftEncoder = new WPI_TalonSRX(1);
    private WPI_TalonSRX m_rightEncoder = new WPI_TalonSRX(2);
    private AHRS m_gyro = new AHRS(SerialPort.Port.kMXP);
    private DifferentialDriveOdometry m_odometry =
            new DifferentialDriveOdometry(
                                    m_gyro.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());
    private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21));
    private PIDController m_leftPID = new PIDController(1, 0, 0);
    private PIDController m_rightPID = new PIDController(1, 0, 0);



    public Drivetrain() {
        configureMotor(frontLeft);
        configureMotor(frontRight);
        configureMotor(backLeft);
        configureMotor(backRight);

        m_leftEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_rightEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        frontRight.setInverted(true);
        // No need to tell backRight to invert, it's a follower.

        AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // Current ChassisSpeeds supplier
            this::driveRelative, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
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
                                    xSpeed.getAsDouble(),
                                    zRotation.getAsDouble(),
                                    squareInputs.getAsBoolean());
                    frontLeft.set(ws.left);
                    frontRight.set(ws.right);
                });
    }

    public Command curvatureDrive(
            DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier allowTurnInPlace) {
        return run(
                () -> {
                    WheelSpeeds ws =
                            DifferentialDrive.curvatureDriveIK(
                                    xSpeed.getAsDouble(),
                                    zRotation.getAsDouble(),
                                    allowTurnInPlace.getAsBoolean());
                    frontLeft.set(ws.left);
                    frontRight.set(ws.right);
                });
    }

    public Command setIdleMode(IdleMode idleMode) {
        return runOnce(
                        () -> {
                            frontLeft.setIdleMode(idleMode);
                            frontRight.setIdleMode(idleMode);
                            backLeft.setIdleMode(idleMode);
                            backRight.setIdleMode(idleMode);
                        })
                .ignoringDisable(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("/DT/Left", frontLeft.get());
        SmartDashboard.putNumber("/DT/Right", frontRight.get());

        m_odometry.update(
                m_gyro.getRotation2d(),
                m_leftEncoder.getSelectedSensorPosition(),
                m_rightEncoder.getSelectedSensorPosition());
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

    private void resetPose(Pose2d pose) {
            m_odometry.resetPosition(m_gyro.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters(), pose);
    }

    private ChassisSpeeds getCurrentSpeeds() {
            return m_kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSecond(),
                            getRightVelocityMetersPerSecond()));
    }

    private void driveRelative(ChassisSpeeds chassisSpeeds) {
            DifferentialDriveWheelSpeeds ws = m_kinematics.toWheelSpeeds(chassisSpeeds);

            m_leftPID.setSetpoint(ws.leftMetersPerSecond);
            m_rightPID.setSetpoint(ws.rightMetersPerSecond);
            frontLeft.set(m_leftPID.calculate(getLeftVelocityMetersPerSecond()));
            frontRight.set(m_rightPID.calculate(getRightVelocityMetersPerSecond()));
    }
}
