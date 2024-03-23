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
import io.github.tigerbotics7125.subsystems.Drivetrain;
import io.github.tigerbotics7125.subsystems.Intake;
import io.github.tigerbotics7125.subsystems.Shooter;
import java.util.Optional;

public class LeftShootThenDrive implements Auto {

  private Drivetrain m_drivetrain;
  private Arm m_arm;
  private Intake m_intake;
  private Shooter m_shooter;

  public LeftShootThenDrive(Drivetrain drivetrain, Arm arm, Intake intake, Shooter shooter) {
    m_drivetrain = drivetrain;
    m_arm = arm;
    m_intake = intake;
    m_shooter = shooter;
  }

  @Override
  public Optional<Command> autoCommand() {
    Command cmd =
        // Lower arm, then shoot note.
        // TODO arm.atState doesn't have a tolerance, may never return true atm.
        Commands.sequence(
                m_arm.pidControl(ArmState.SPEAKERAUTO).until(m_arm.atState()),
                m_shooter.shootNote(m_intake))
            .withTimeout(5)
            // Drive backwards for 1 second.
            .andThen(m_drivetrain.arcadeDrive(() -> -.5, () -> 0, () -> false).withTimeout(1))
            // Turn left for 2 seconds.
            .andThen(m_drivetrain.arcadeDrive(() -> -.5, () -> .5, () -> false).withTimeout(2))
            // Drive backwards for 2 seconds.
            .andThen(m_drivetrain.arcadeDrive(() -> -.5, () -> 0, () -> false).withTimeout(2));

    return Optional.of(cmd);
  }
}
