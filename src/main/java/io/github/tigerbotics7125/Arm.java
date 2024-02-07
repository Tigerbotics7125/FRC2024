/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Arm {
    WPI_TalonSRX armMotor1 = new WPI_TalonSRX(5);
    WPI_TalonSRX armMotor2 = new WPI_TalonSRX(6);
    DifferentialDrive armDrive = new DifferentialDrive(armMotor1, armMotor2);

    public static void raiseArm(DifferentialDrive armDrive) {
        armDrive.tankDrive(0.25, 0.25);
    }

    public static void lowerArm(DifferentialDrive armDrive) {
        armDrive.tankDrive(-0.25, -0.25);
    }

    public static void stopArm(DifferentialDrive armDrive) {
        armDrive.tankDrive(0, 0);
    }
}
