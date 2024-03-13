/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.tigerbotics7125.Constants.Arm.ArmState;
import io.github.tigerbotics7125.subsystems.Arm;
import io.github.tigerbotics7125.subsystems.Intake;
import io.github.tigerbotics7125.subsystems.Shooter;
import java.util.Optional;

public class ShootNoteNoDrive implements Auto {

    private Arm m_arm;
    private Intake m_intake;
    private Shooter m_shooter;

    public ShootNoteNoDrive(Arm arm, Intake intake, Shooter shooter) {
        m_arm = arm;
        m_intake = intake;
        m_shooter = shooter;
    }

    @Override
    public Optional<Command> autoCommand() {
        Command cmd =
                Commands.sequence(
                                m_arm.setState(ArmState.SPEAKERAUTO),
                                Commands.waitUntil(m_arm.atState()),
                                m_shooter.shootNote(m_intake))
                        .withTimeout(5);

        return Optional.of(cmd);
    }
}
