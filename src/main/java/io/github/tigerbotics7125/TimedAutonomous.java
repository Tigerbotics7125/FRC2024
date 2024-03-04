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
    String autonomous1 = "Autonomous Left";
    String autonomous2 = "Autonomous Right";
    String shootOnlyAuto = "Shoot Only";

    double startTime = Timer.getFPGATimestamp();

    double howLong = 5;
    double howLong2 = 6;
    double howLong3 = 5.5;
    double howLong4 = 8;
    double deltaTime;
    SendableChooser<String> m_chooserAutonomous = new SendableChooser<>();

    public TimedAutonomous() {

        startTime = Timer.getFPGATimestamp();

        m_chooserAutonomous.setDefaultOption("Autonomous Left", autonomous1);
        m_chooserAutonomous.addOption("Autonomous Right", autonomous2);
        m_chooserAutonomous.addOption("Shoot Only", shootOnlyAuto);
        SmartDashboard.putData("Autonomous", m_chooserAutonomous);
    }

    public void autoChooser(
            DifferentialDrive mDrive, Arm mArm, String autonomousSelect, Intake kIntake) {
        // kIntake.shootRing(.5);

        System.out.println(autonomousSelect);
        switch (autonomousSelect) {
            case "Autonomous Left":
                runAutoLeft(mDrive, mArm, kIntake);
                break;

            case "Autonomous Right":
                runAutoRight(mDrive, mArm, kIntake);
                break;

            case "Shoot Only":
                shootOnlyAuto(mArm, kIntake);
            default:
                shootOnlyAuto(mArm, kIntake);

                break;
        }
    }

    public void runAutoLeft(DifferentialDrive mDrive, Arm mArm, Intake kIntake) {
        System.out.println("Autonomous Left");
        deltaTime = Timer.getFPGATimestamp() - startTime;

        if (deltaTime < howLong) {
            mArm.goToPosition(mArm.speakerAuto);
            kIntake.shootRing(.5);
        } else if (deltaTime < howLong2) {
            kIntake.stopPickup();
            kIntake.stopShooter();
            mDrive.tankDrive(-0.5, -0.5);
        } else if (deltaTime < howLong3) {
            kIntake.stopShooter();
            kIntake.stopPickup();
        }
        mDrive.tankDrive(-0.75, 0);
        if (deltaTime < howLong4) {
            // kIntake.stopPickup();
            // kIntake.stopShooter();
            // mArm.goToPosition(mArm.down);
            mDrive.tankDrive(-0.5, -0.5);
        } else {
            // kIntake.stopPickup();
            // kIntake.stopShooter();
            mDrive.tankDrive(0, 0);
        }
    }

    public void runAutoRight(DifferentialDrive mDrive, Arm mArm, Intake kIntake) {
        System.out.println("Autonomous Right");
        deltaTime = Timer.getFPGATimestamp() - startTime;
        if (deltaTime < howLong) {
            mArm.goToPosition(mArm.speakerAuto);
            kIntake.shootRing(.5);
        } else if (deltaTime < howLong2) {
            kIntake.stopPickup();
            kIntake.stopShooter();
            mDrive.tankDrive(-0.5, -0.5);
        } else if (deltaTime < howLong3) {
            kIntake.stopShooter();
            kIntake.stopPickup();
            mDrive.tankDrive(0, -0.75);
        } else if (deltaTime < howLong4) {
            kIntake.stopPickup();
            kIntake.stopShooter();
            mDrive.tankDrive(-0.5, -0.5);
        } else {
            kIntake.stopPickup();
            kIntake.stopShooter();
            mDrive.tankDrive(0, 0);
        }
    }

    public void shootOnlyAuto(Arm mArm, Intake kIntake) {
        System.out.println("Shoot Only");
        deltaTime = startTime = Timer.getFPGATimestamp() - startTime;

        if (deltaTime < howLong) {
            mArm.goToPosition(mArm.speakerAuto);
            kIntake.shootRing(.5);
        } else {
            kIntake.stopShooter();
            kIntake.stopPickup();
        }
    }
}
