/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;

public interface Auto {

  /** @return The command to run before running this auto. */
  default Command preCommand() {
    return Commands.none();
  }

  /** @return An Optional containing the auto command, or empty if should be pathplanner. */
  Optional<Command> autoCommand();

  // TOOD uncomment these as they are part of pathplanner

  // List<PathPlannerTrajectory> getPath();

  // Map<String, Command> getEventMap();

  /** @return The command to run after running this auto. */
  default Command postCommand() {
    return Commands.none();
  }
}
