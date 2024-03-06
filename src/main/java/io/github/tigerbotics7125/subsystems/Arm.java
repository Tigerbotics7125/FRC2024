/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.Constants;
import io.github.tigerbotics7125.Constants.Arm.ArmState;

// TODO consider using ProfiledPIDController instead for smoother motion.

public class Arm extends SubsystemBase {

    private CANSparkMax m_left = new CANSparkMax(Constants.Arm.kLeftID, Constants.Arm.kMotorType);
    private CANSparkMax m_right = new CANSparkMax(Constants.Arm.kRightID, Constants.Arm.kMotorType);

    private RelativeEncoder m_encoder = m_left.getEncoder();

    private PIDController m_PID = Constants.Arm.kPID;
    private ArmFeedforward m_feedforward = Constants.Arm.kFF;

    public Arm() {
        configureMotor(m_left);
        configureMotor(m_right);
        m_right.follow(m_left, Constants.Arm.kFollowerInverted);
    }

    private void configureMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        Timer.delay(.02);

        // TODO motor configs, we need to do this but we can later.

        motor.burnFlash();
        Timer.delay(.02);
    }

    public Command disable() {
        return run(m_left::stopMotor);
    }

    public Command setState(ArmState state) {
        return runOnce(() -> m_PID.setSetpoint(state.kPosition));
    }

    public Command pidControl() {
        return run(
                () -> {
                    double pidContribution = 12D * m_PID.calculate(m_encoder.getPosition());
                    double ffContribution = m_feedforward.calculate(m_PID.getSetpoint(), 0);
                    m_left.setVoltage(pidContribution + ffContribution);
                });
    }

    public Command resetEncoder() {
        return runOnce(() -> m_encoder.setPosition(0));
    }

    @Override
    public void periodic() {}
}
