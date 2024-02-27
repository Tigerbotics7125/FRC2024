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

public class Arm {
    private CANSparkMax armMotor1;
    private CANSparkMax armMotor2;

    private RelativeEncoder armEncoder;
    private final double downAngle = 0;
    private final double ampAngle = 180;
    private final double shootingAngle = 90;

    private SparkPIDController mPID;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    public Arm(int armMotor1ID, int armMotor2ID) {
        armMotor1 = new CANSparkMax(armMotor1ID, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(armMotor2ID, MotorType.kBrushless);
        armMotor2.follow(armMotor1);
        armEncoder = armMotor1.getEncoder();
        mPID = armMotor1.getPIDController();

        // PID coefficients
        kP = 0.1;
        kI = 1e-4;
        kD = 1;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;

        // set PID coefficients
        mPID.setP(kP);
        mPID.setI(kI);
        mPID.setD(kD);
        mPID.setIZone(kIz);
        mPID.setFF(kFF);
        mPID.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("Arm P Gain", kP);
        SmartDashboard.putNumber("Arm I Gain", kI);
        SmartDashboard.putNumber("Arm D Gain", kD);
        SmartDashboard.putNumber("Arm I Zone", kIz);
        SmartDashboard.putNumber("Arm Feed Forward", kFF);
        SmartDashboard.putNumber("Arm Max Output", kMaxOutput);
        SmartDashboard.putNumber("Arm Min Output", kMinOutput);
        SmartDashboard.putNumber("Arm Set Rotations", 0);
    }

    public void teleop() {
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != kP)) {
            mPID.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            mPID.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            mPID.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            mPID.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            mPID.setFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            mPID.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }
    }

    public void moveToPostion(double setPostion) {
        double rotations = SmartDashboard.getNumber("Arm Set Rotations", 0);
        System.out.println("Rotations: " + rotations);
        /**
         * PIDController objects are commanded to a set point using the SetReference() method.
         *
         * <p>The first parameter is the value of the set point, whose units vary depending on the
         * control type set in the second parameter.
         *
         * <p>The second parameter is the control type can be set to one of four parameters:
         * com.revrobotics.CANSparkMax.ControlType.kDutyCycle
         * com.revrobotics.CANSparkMax.ControlType.kPosition
         * com.revrobotics.CANSparkMax.ControlType.kVelocity
         * com.revrobotics.CANSparkMax.ControlType.kVoltage
         */
        mPID.setReference(rotations, CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("Arm SetPoint", rotations);
        SmartDashboard.putNumber("Arm ProcessVariable", armEncoder.getPosition());
    }

    public void raiseArm() {

        armMotor1.set(.25);
    }

    public void lowerArm() {

        armMotor1.set(-.25);
    }

    public void stopArm() {
        mPID.setReference(armEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
    }

    public void amp() {
        moveToPostion(ampAngle);
    }

    public void shoot() {
        moveToPostion(shootingAngle);
    }

    public void stow() {
        moveToPostion(downAngle);
    }
}
