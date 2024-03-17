/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.tigerbotics7125.Constants;

// TODO contemplate using a BangBangController, especially if the shooter becomes more massive.

public class Shooter extends SubsystemBase {

    private CANSparkMax m_left =
            new CANSparkMax(Constants.Shooter.kLeftID, Constants.Shooter.kMotorType);
    private CANSparkMax m_right =
            new CANSparkMax(Constants.Shooter.kRightID, Constants.Shooter.kMotorType);

    private PIDController m_PID =
            new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD);

    private RelativeEncoder m_encoder = m_right.getEncoder();

    public Shooter() {
        configureMotor(m_left);
        configureMotor(m_right);

        m_left.setInverted(Constants.Shooter.kInvertedFollower);
        m_right.follow(m_left);

        m_PID.setTolerance(0, Constants.Shooter.kPIDTolerance);
    }

    private void configureMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        Timer.delay(.02);

        motor.setSmartCurrentLimit(Constants.Shooter.kCurrentLimit);

        motor.burnFlash();
        Timer.delay(.02);
    }

    public Trigger isShooterReady() {
        Trigger atSetpoint = new Trigger(m_PID::atSetpoint);
        Trigger setpointValid = new Trigger(() -> m_PID.getSetpoint() != 0);
        return setpointValid.and(atSetpoint);
    }

    public Command disable() {
        return run(m_left::stopMotor);
    }

    public Command prepShooter() {
        return runOnce(() -> m_PID.setSetpoint(Constants.Shooter.kShootRPM))
        .andThen(pidControl());
    }

    public Command shootNote(Intake intake) {
        return prepShooter()
                .andThen(Commands.waitUntil(isShooterReady()))
                .andThen(intake.feedShooter().withTimeout(1));
    }

    public Command pidControl() {
        return run(
                () -> {
                    double pidContribution = m_PID.calculate(m_encoder.getVelocity());
                    m_left.set(pidContribution);
                });
    }

    /**
     * TODO seth: create a method to keep the shooter at a constant but slightly less velocity than
     * our shooter velocity. This should be the new default method. Eventually this should consider
     * our distance from the speaker so that it automatically ramps up to shooting speed when we get
     * closer. Not entirely relevant but also should look into using the InterpolatingMap or
     * whatever its called, and some data we can create to find values for shooting from a distance.
     */
    @Override
    public void periodic() {}
}
