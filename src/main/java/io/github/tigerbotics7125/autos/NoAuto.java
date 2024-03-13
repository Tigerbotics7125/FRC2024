/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;

public class NoAuto implements Auto {

    @Override
    public Optional<Command> autoCommand() {
        return Optional.of(Commands.none());
    }
}
