/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

public class Arm {
    CANSparkMax armMotor1;
    CANSparkMax armMotor2;
    
    RelativeEncoder armEncoder;
    double downAngle = 0;
    double ampAngle = 180;
    double shootingAngle = 90;

    PIDController mPID;
    double P_GAIN = 20; 
    double I_GAIN = 0; 
    double D_GAIN = .1; 

    public Arm(int armMotor1ID, int armMotor2ID){
        this.armMotor1 = new CANSparkMax(armMotor1ID, MotorType.kBrushless);
        this.armMotor2 = new CANSparkMax(armMotor2ID, MotorType.kBrushless);
        armMotor2.follow(armMotor1);
        this.armEncoder = armMotor1.getEncoder();
        mPID = new PIDController(P_GAIN, I_GAIN, D_GAIN);
        
    }

    public void moveToPostion(double setPostion){
        
        double output = mPID.calculate(armEncoder.getPosition(), setPostion);
        armMotor1.set(output);

        }
        
    
    public void raiseArm() {

        armMotor1.set(.25);
        
    }

    public void lowerArm() {

        armMotor1.set(-.25);
        
    }

    public void stopArm() {

        armMotor1.set(0);
        
    }
}
