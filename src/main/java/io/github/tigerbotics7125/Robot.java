/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // private RobotContainer m_robotContainer;

    // The robot's subsystems and commands are defined here...

    // Additional controllers may be added if needed.

    private CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushed);
    private CANSparkMax rightMotor1 = new CANSparkMax(2, MotorType.kBrushed);
    private CANSparkMax leftMotor2 = new CANSparkMax(3, MotorType.kBrushed);
    private CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushed);
    RelativeEncoder leftMEncoder = leftMotor1.getAlternateEncoder(4096);
    RelativeEncoder rightMEncoder = rightMotor1.getAlternateEncoder(4096);

    int intakeID = 5;
    int shooterLeftID = 6;
    int shooterRightID = 7;
    double shooterSpeed = 1;
    double intakeSpeed = .5;
    Intake kIntake;
    Arm mArm;
    int armMotor1ID = 8;
    int armMotor2ID = 9;

    double autonomousDistance = 50;
    double turnDistance = 21.991148;
    double wheelCircumference = 6 * Math.PI;

    TimedAutonomous mTimedAutonomous;

    private DifferentialDrive mDrive = new DifferentialDrive(leftMotor1, rightMotor1);
    private XboxController mXboxDrive = new XboxController(0);
    private XboxController mXboxOperator = new XboxController(1);
    String tankDrive = "Tank Drive";
    String arcadeDrive = "Arcade Drive";
    String driveSelect;
    SendableChooser<String> m_chooserDrive = new SendableChooser<>();
    String autonomous1 = "Autonomous 1";
    String autonomous2 = "Autonomous 2";
    String autonomousSelect;
    SendableChooser<String> m_chooserAutonomous = new SendableChooser<>();

    WPI_TalonSRX encoderSRX = new WPI_TalonSRX(1);

    /*
     * private CANSparkMax mLeft1 = new CANSparkMax(0, MotorType.kBrushed);
     * private CANSparkMax mLeft2 = new CANSparkMax(1, MotorType.kBrushed);
     * private CANSparkMax mRight1 = new CANSparkMax(2, MotorType.kBrushed);
     * private CANSparkMax mRight2 = new CANSparkMax(3, MotorType.kBrushed);
     *
     * private DifferentialDrive mDrive = new DifferentialDrive(mLeft1, mRight1);
     * private XboxController mXbox = new XboxController(0);
     */
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Configure the trigger bindings

        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);

        rightMotor1.setInverted(true);
        rightMotor2.setInverted(true);

        m_chooserDrive.setDefaultOption("Tank Drive", tankDrive);
        m_chooserDrive.addOption("Arcade Drive", arcadeDrive);
        SmartDashboard.putData("Drive choices", m_chooserDrive);

        m_chooserAutonomous.setDefaultOption("Autonomous 1", autonomous1);
        m_chooserAutonomous.addOption("Autonomous 2", autonomous2);
        SmartDashboard.putData("Autonomous", m_chooserAutonomous);
        CameraServer.startAutomaticCapture();

        kIntake = new Intake(intakeID, shooterLeftID, shooterRightID, shooterSpeed, intakeSpeed);
        mArm = new Arm(armMotor1ID, armMotor2ID);

        leftMEncoder.setPosition(0);
        rightMEncoder.setPosition(0);
        // leftMEncoder.setInverted(true);
        // rightMEncoder.setInverted(true);

        // mArm.goToPosition(mArm.speaker);

    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {

        SmartDashboard.putNumber("Encoder Value Right", rightMEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Value Left", leftMEncoder.getPosition());

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.

    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // An example command will be run in autonomous
        leftMEncoder.setPosition(0);
        rightMEncoder.setPosition(0);
        autonomousSelect = m_chooserAutonomous.getSelected();
        // schedule the autonomous command (example)
        mTimedAutonomous = new TimedAutonomous();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

        // mTimedAutonomous.autoChooser(mDrive, kIntake);

        mDrive.tankDrive(0.5, 0);
        double velocity = encoderSRX.getSelectedSensorVelocity(1);
        SmartDashboard.putNumber("Left Velocity", velocity);

        /*  switch (autonomousSelect) {
            case "Autonomous 1":
                while (rightMEncoder.getPosition() < (turnDistance / wheelCircumference)) {
                    mDrive.tankDrive(0, .5);
                }
                leftMEncoder.setPosition(0);
                rightMEncoder.setPosition(0);

                while (rightMEncoder.getPosition() < (autonomousDistance / wheelCircumference)
                        && leftMEncoder.getPosition() < (autonomousDistance / wheelCircumference)) {
                    mDrive.tankDrive(.5, .5);
                }
                break;

            case "Autonomous 2":
                while (leftMEncoder.getPosition() < (turnDistance / wheelCircumference)) {
                    mDrive.tankDrive(.5, 0);
                }
                leftMEncoder.setPosition(0);
                rightMEncoder.setPosition(0);

                while (rightMEncoder.getPosition() < (autonomousDistance / wheelCircumference)
                        && leftMEncoder.getPosition() < (autonomousDistance / wheelCircumference)) {
                    mDrive.tankDrive(0.5, .5);
                }
                break;

            default:
                break;
        }*/
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        driveSelect = m_chooserDrive.getSelected();

        SmartDashboard.putNumber("Intake Speed", .5);
        SmartDashboard.putNumber("Shooter Speed", 1);

        SmartDashboard.putBoolean("Arm Manual Control", false);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        driveSelect = m_chooserDrive.getSelected();
        // System.out.println("Drive mode: " + driveSelect);
        SmartDashboard.putNumber("Rotations Right Wheel", rightMEncoder.getPosition());

        switch (driveSelect) {
            case "Tank Drive":
                mDrive.tankDrive(mXboxDrive.getLeftY(), mXboxDrive.getRightY());
                break;

            case "Arcade Drive":
                mDrive.arcadeDrive(mXboxDrive.getLeftY(), mXboxDrive.getLeftX(), false);

                break;

            default:
                break;
        }

        SmartDashboard.putNumber("Left Motor Value", leftMotor1.get());
        SmartDashboard.putNumber("Right Motor Value", rightMotor1.get());

        // intakeSpeed = SmartDashboard.getNumber("Intake Speed", .5);
        // shooterSpeed = SmartDashboard.getNumber("Shooter Speed", 1);

        // Intake and shooter controls
        if (mXboxOperator.getRightBumper()) {
            kIntake.pickupRing();
        } else {
            kIntake.stopPickup();
            kIntake.backUpRing(mXboxOperator.getRightTriggerAxis());
        }

        if (mXboxOperator.getLeftBumper()) {
            kIntake.shootRing();
        } else {
            kIntake.stopShooter();
        }

        // Arm Controls
        boolean armControl = SmartDashboard.getBoolean("Arm Manual Control", false);

        if (armControl) {
            // mArm.teleop();
        } else {
            /*if (mXboxOperator.getYButtonPressed()) {
                mArm.goToPosition(mArm.amp)
            } else if (mXboxOperator.getXButtonPressed()) {
                mArm.goToPosition(mArm.speaker)
            } else if (mXboxOperator.getBButtonPressed()) {
                mArm.goToPosition(mArm.down);
            } else {
                mArm.stopArm();
            }*/
        }

        // mArm.setTo0();

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
