/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Arm {
    WPI_TalonSRX armMotor1 = new WPI_TalonSRX(5);
    WPI_TalonSRX armMotor2 = new WPI_TalonSRX(6);
    

    
    public void raiseArm(WPI_TalonSRX armMotor1, WPI_TalonSRX armMotor2) {
        
        armMotor1.set(.25);
        armMotor2.set(.25);
    }

    public void lowerArm(WPI_TalonSRX armMotor1, WPI_TalonSRX armMotor2) {
        
        armMotor1.set(-.25);
        armMotor2.set(-.25);
    }

    public void stopArm(WPI_TalonSRX armMotor1, WPI_TalonSRX armMotor2) {

        armMotor1.set(0);
        armMotor2.set(0);
    }
}
