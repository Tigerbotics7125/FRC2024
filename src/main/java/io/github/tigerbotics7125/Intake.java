package io.github.tigerbotics7125;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Intake {

    private CANSparkMax shooterMotorLeft;
    private CANSparkMax shooterMotorRight;
    private CANSparkMax intakeMotor;

    public Intake(int intakeID, int shooterLeftID, int shooterRightID,
            double shooterSpeed, double intakeSpeed) {
        this.shooterMotorLeft = new CANSparkMax(shooterLeftID, MotorType.kBrushless);
        this.shooterMotorRight = new CANSparkMax(shooterRightID, MotorType.kBrushless);
        this.intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);

    }

    public void pickupRing(double intakeSpeed) {

        intakeMotor.set(intakeSpeed);

    }

    public void shootRing(double shooterSpeed) {

        shooterMotorLeft.follow(shooterMotorRight);
        shooterMotorRight.set(shooterSpeed);

    }

    public void stopPickup() {

        intakeMotor.set(0);

    }

    public void stopShooter() {

        shooterMotorLeft.follow(shooterMotorRight);
        shooterMotorRight.set(0);

    }

}
