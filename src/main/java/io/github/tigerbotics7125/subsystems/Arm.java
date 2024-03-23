/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.subsystems;

import static io.github.tigerbotics7125.Constants.Arm.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.tigerbotics7125.Constants.Arm.ArmState;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {

  private CANSparkMax m_left = new CANSparkMax(kLeftID, kMotorType);
  private CANSparkMax m_right = new CANSparkMax(kRightID, kMotorType);

  private RelativeEncoder m_encoder = m_left.getEncoder();

  private PIDController m_PID = kPID;
  // private ArmFeedforward m_feedforward = Constants.Arm.kFF;

  public Arm() {
    configureMotor(m_left);
    configureMotor(m_right);
    m_right.follow(m_left, kFollowerInverted);

    m_encoder.setPositionConversionFactor(kPositionConversionFactor);
    m_encoder.setVelocityConversionFactor(kVelocityConversionFactor);
    m_encoder.setPosition(0);
  }

  private void configureMotor(CANSparkMax motor) {
    motor.restoreFactoryDefaults();
    Timer.delay(.02);

    motor.setSmartCurrentLimit(kCurrentLimit);
    motor.setIdleMode(IdleMode.kCoast);
    motor.burnFlash();
    Timer.delay(.02);
  }

  public Command disable() {
    return run(m_left::stopMotor);
  }

  public Command voltageControl(DoubleSupplier input) {
    return run(() -> m_left.setVoltage(input.getAsDouble()));
  }

  public Command pidControl(ArmState state) {
    return runOnce(() -> m_PID.setSetpoint(state.kPosition))
        .andThen(run(() -> m_left.set(m_PID.calculate(m_encoder.getPosition()))));
  }

  public Command resetEncoder() {
    return runOnce(() -> m_encoder.setPosition(0));
  }

  public Command setIdleMode(IdleMode idleMode) {
    return runOnce(
            () -> {
              m_left.setIdleMode(idleMode);
              m_right.setIdleMode(idleMode);
            })
        .ignoringDisable(true);
  }

  public Trigger atState() {
    return new Trigger(m_PID::atSetpoint);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Position", m_encoder.getPosition());
  }
}
