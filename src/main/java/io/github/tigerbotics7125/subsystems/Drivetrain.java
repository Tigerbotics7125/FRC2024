/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {
    private CANSparkMax frontLeft =
            new CANSparkMax(Constants.DriveTrain.kFrontLeftID, Constants.DriveTrain.kMotorType);
    private CANSparkMax frontRight =
            new CANSparkMax(Constants.DriveTrain.kFrontRightID, Constants.DriveTrain.kMotorType);
    private CANSparkMax backLeft =
            new CANSparkMax(Constants.DriveTrain.kBackLeftID, Constants.DriveTrain.kMotorType);
    private CANSparkMax backRight =
            new CANSparkMax(Constants.DriveTrain.kBackRightID, Constants.DriveTrain.kMotorType);

    public Drivetrain() {
        configureMotor(frontLeft);
        configureMotor(frontRight);
        configureMotor(backLeft);
        configureMotor(backRight);

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        frontRight.setInverted(true);
        // No need to tell backRight to invert, it's a follower.
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
                                    WheelSpeeds ws = DifferentialDrive.curvatureDriveIK(
                                                    xSpeed.getAsDouble(),
                                                    zRotation.getAsDouble(),
                                                    allowTurnInPlace.getAsBoolean());
                                    frontLeft.set(ws.left);
                                    frontRight.set(ws.right);
                            });
    }
     public Command setIdleMode(IdleMode idleMode) {
        return runOnce(()->{
            frontLeft.setIdleMode(idleMode);
            frontRight.setIdleMode(idleMode);
            backLeft.setIdleMode(idleMode);
            backRight.setIdleMode(idleMode);
        }).ignoringDisable(true);


    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("/DT/Left", frontLeft.get());
        SmartDashboard.putNumber("/DT/Right", frontRight.get());
    }
}
