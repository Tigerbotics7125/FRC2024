/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Intake {

    private CANSparkMax shooterMotorLeft;
    private CANSparkMax shooterMotorRight;
    private CANSparkMax intakeMotor;

    public Intake(
            int intakeID,
            int shooterLeftID,
            int shooterRightID,
            double shooterSpeed,
            double intakeSpeed) {
        this.shooterMotorLeft = new CANSparkMax(shooterLeftID, MotorType.kBrushless);
        this.shooterMotorRight = new CANSparkMax(shooterRightID, MotorType.kBrushless);
        this.intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);
        shooterMotorLeft.follow(shooterMotorRight);
    }

    public void pickupRing(double intakeSpeed) {

        intakeMotor.set(intakeSpeed);
    }

    public void shootRing(double shooterSpeed) {
        shooterMotorRight.set(shooterSpeed);
    }

    public void stopPickup() {

        intakeMotor.set(0);
    }

    public void stopShooter() {
     shooterMotorRight.set(0);
    }
}
