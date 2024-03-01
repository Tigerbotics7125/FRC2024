/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TimedAutonomous {
    String autonomous1 = "Autonomous 1";
    String autonomous2 = "Autonomous 2";
    String autonomousSelect;
    double startTime = Timer.getFPGATimestamp();
    double distance = 21.991148;
    double velocity;
    double howLong = 4;
    double howLong2 = howLong + (distance / velocity);
    double howLong3 = 14;
    double deltaTime;
    SendableChooser<String> m_chooserAutonomous = new SendableChooser<>();

    public TimedAutonomous() {

        startTime = Timer.getFPGATimestamp();

        m_chooserAutonomous.setDefaultOption("Autonomous 1", autonomous1);
        m_chooserAutonomous.addOption("Autonomous 2", autonomous2);
        SmartDashboard.putData("Autonomous", m_chooserAutonomous);
    }

    public void autoChooser(DifferentialDrive mDrive, Intake kIntake) {
        kIntake.shootRing();
        autonomousSelect = m_chooserAutonomous.getSelected();

        switch (autonomousSelect) {
            case "Autonomous 1":
                runAutoLeft(mDrive, kIntake);
                break;

            case "Autonomous 2":
                runAutoRight(mDrive, kIntake);
                break;

            default:
                break;
        }
    }

    public void runAutoLeft(DifferentialDrive mDrive, Intake kIntake) {
        deltaTime = Timer.getFPGATimestamp() - startTime;
        if (deltaTime < howLong) {
            kIntake.shootRing();
        } else if (deltaTime < howLong2) {
            kIntake.stopShooter();
            kIntake.stopPickup();
            mDrive.tankDrive(0.5, 0);
        } else if (deltaTime < howLong3) {
            mDrive.tankDrive(0.5, 0.5);
        } else {
            mDrive.tankDrive(0, 0);
        }
    }

    public void runAutoRight(DifferentialDrive mDrive, Intake kIntake) {
        deltaTime = Timer.getFPGATimestamp() - startTime;
        if (deltaTime < howLong) {
            kIntake.shootRing();
        } else if (deltaTime < howLong2) {
            mDrive.tankDrive(0, 0.5);
        } else if (deltaTime < howLong3) {
            mDrive.tankDrive(0.5, 0.5);
        } else {
            mDrive.tankDrive(0, 0);
        }
    }
}
