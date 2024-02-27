/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {

    private CANSparkMax m_shooterMotorLeft;
    private CANSparkMax m_shooterMotorRight;
    private CANSparkMax m_intakeMotor;

    private SparkPIDController m_shooterPID;
    private RelativeEncoder m_shooterEncoder;
    private final double INTAKE_PICKUP_SPEED = 0.5;
    private final double INTAKE_SHOOT_SPEED = 1.0;
    private final double SHOOT_SPEED = 0.25;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    private boolean shooting = false;

    public Intake(
            int intakeID,
            int shooterLeftID,
            int shooterRightID,
            double shooterSpeed,
            double intakeSpeed) {

        m_shooterMotorLeft = new CANSparkMax(shooterLeftID, MotorType.kBrushless);
        m_shooterMotorRight = new CANSparkMax(shooterRightID, MotorType.kBrushless);

        m_shooterEncoder = m_shooterMotorRight.getEncoder();
        m_shooterPID = m_shooterMotorRight.getPIDController();

        kP = 0.0004;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.00017;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;

        m_shooterPID.setP(kP);
        m_shooterPID.setI(kI);
        m_shooterPID.setD(kD);
        m_shooterPID.setIZone(kIz);
        m_shooterPID.setFF(kFF);
        m_shooterPID.setOutputRange(kMinOutput, kMaxOutput);

        m_intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);
        m_shooterMotorLeft.setInverted(true);
        m_shooterMotorRight.setInverted(true);
        m_intakeMotor.setInverted(false);
        m_shooterMotorLeft.follow(m_shooterMotorRight);
    }

    public void pickupRing() {
        if (!shooting) {
            m_intakeMotor.set(INTAKE_PICKUP_SPEED);
        }
    }

    public void shootRing() {
        shooting = true;
        m_shooterPID.setReference(SHOOT_SPEED * maxRPM, CANSparkMax.ControlType.kVelocity);

        // m_shooterMotorRight.set(SHOOT_SPEED);
        if (m_shooterEncoder.getVelocity() >= (SHOOT_SPEED - .05) * maxRPM) {

            m_intakeMotor.set(INTAKE_SHOOT_SPEED);
        }
    }

    public void stopPickup() {
        if (!shooting) m_intakeMotor.set(0);
    }

    public void stopShooter() {
        m_shooterPID.setReference(0, CANSparkMax.ControlType.kVelocity);
        shooting = false;
        SmartDashboard.putNumber("SetRef", 0);
        SmartDashboard.putNumber("ProcessVariable", m_shooterEncoder.getVelocity());
    }
}
