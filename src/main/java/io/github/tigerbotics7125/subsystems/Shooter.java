/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.subsystems;

import static io.github.tigerbotics7125.Constants.Shooter.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.tigerbotics7125.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax m_left = new CANSparkMax(kLeftID, kMotorType);
  private CANSparkMax m_right = new CANSparkMax(kRightID, kMotorType);

  private PIDController m_PID = new PIDController(kP, kI, kD);

  private RelativeEncoder m_encoder = m_right.getEncoder();

  public Shooter() {
    configureMotor(m_left);
    configureMotor(m_right);

    m_left.setInverted(kInvertedFollower);
    m_right.follow(m_left);

    m_PID.setTolerance(0, kPIDTolerance);
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
    return runOnce(() -> m_PID.setSetpoint(Constants.Shooter.kShootRPM)).andThen(pidControl());
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

  @Override
  public void periodic() {}
}
