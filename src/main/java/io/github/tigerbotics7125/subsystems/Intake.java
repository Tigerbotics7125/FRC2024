/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.subsystems;

import static io.github.tigerbotics7125.Constants.Intake.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.Constants;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {

  private CANSparkMax m_intake = new CANSparkMax(kIntakeID, kMotorType);

  public Intake() {
    configureMotor(m_intake);
  }

  private void configureMotor(CANSparkMax motor) {
    motor.restoreFactoryDefaults();
    Timer.delay(.02);

    motor.setSmartCurrentLimit(kCurrentLimit);

    m_intake.setInverted(kInverted);

    motor.burnFlash();
    Timer.delay(.02);
  }

  public Command disable() {
    return run(m_intake::disable);
  }

  public Command intake() {
    return run(() -> m_intake.set(kIntakeSpeed));
  }

  public Command outtake(DoubleSupplier axis) {
    return run(
        () ->
            m_intake.set(
                MathUtil.interpolate(0, Constants.Intake.kMaxOutakeSpeed, axis.getAsDouble())));
  }

  public Command feedShooter() {
    return run(() -> m_intake.set(Constants.Intake.kFeedSpeed));
  }
}
