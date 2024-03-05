/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import io.github.tigerbotics7125.Constants.DriveTrain.ControlType;
import io.github.tigerbotics7125.subsystems.Drivetrain;
import io.github.tigerbotics7125.subsystems.Intake;
import java.util.Map;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private CommandXboxController m_driver =
            new CommandXboxController(Constants.HID.kDriverControllerPort);
    private CommandXboxController m_operator =
            new CommandXboxController(Constants.HID.kOperatorControllerPort);

    private Drivetrain m_drivetrain = new Drivetrain();
    private Intake m_intake = new Intake();

    int shooterLeftID = 6;
    int shooterRightID = 7;
    double shooterSpeed = 1;
    Arm mArm;
    int armMotor1ID = 8;
    int armMotor2ID = 9;

    double autonomousDistance = 50;
    double turnDistance = 21.991148;
    double wheelCircumference = 6 * Math.PI;

    TimedAutonomous mTimedAutonomous;

    SendableChooser<Constants.DriveTrain.ControlType> m_driveControlChooser =
            new SendableChooser<>();
    String autonomous1 = "Autonomous Left";
    String autonomous2 = "Autonomous Right";
    String shootOnlyAuto = "Shoot Only";
    String autonomousSelect;
    SendableChooser<String> m_chooserAutonomous = new SendableChooser<>();

/**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Configure the trigger bindings

        m_driveControlChooser.setDefaultOption(
                ControlType.CURVE_ROCKETLEAGUE.name(), ControlType.CURVE_ROCKETLEAGUE);
        for (ControlType controlType : ControlType.values()) {
            if (controlType.equals(ControlType.CURVE_ROCKETLEAGUE)) continue;
            m_driveControlChooser.addOption(controlType.name(), controlType);
        }

        m_drivetrain.setDefaultCommand(
                Commands.select(
                        Map.of(
                                ControlType.ARCADE,
                                m_drivetrain.arcadeDrive(
                                        m_driver::getLeftY, m_driver::getRightX, () -> true),
                                ControlType.ARCADE_ROCKETLEAGUE,
                                m_drivetrain.arcadeDrive(
                                        () ->
                                                m_driver.getRightTriggerAxis()
                                                        - m_driver.getLeftTriggerAxis(),
                                        m_driver::getLeftX,
                                        () -> true),
                                ControlType.CURVE,
                                m_drivetrain.curvatureDrive(
                                        m_driver::getLeftY, m_driver::getRightX, () -> true),
                                ControlType.CURVE_ROCKETLEAGUE,
                                m_drivetrain.curvatureDrive(
                                        () ->
                                                m_driver.getRightTriggerAxis()
                                                        - m_driver.getLeftTriggerAxis(),
                                        m_driver::getLeftX,
                                        () -> true)),
                        m_driveControlChooser::getSelected));
        m_driveControlChooser.onChange(
                controlType ->
                        m_drivetrain.setDefaultCommand(
                                switch (controlType) {
                                    case ARCADE -> m_drivetrain.arcadeDrive(
                                            m_driver::getLeftY, m_driver::getRightX, () -> true);
                                    case ARCADE_ROCKETLEAGUE -> m_drivetrain.arcadeDrive(
                                            () ->
                                                    m_driver.getRightTriggerAxis()
                                                            - m_driver.getLeftTriggerAxis(),
                                            m_driver::getLeftX,
                                            () -> true);
                                    case CURVE -> m_drivetrain.curvatureDrive(
                                            m_driver::getLeftY, m_driver::getRightX, () -> true);
                                    case CURVE_ROCKETLEAGUE -> m_drivetrain.curvatureDrive(
                                            () ->
                                                    m_driver.getRightTriggerAxis()
                                                            - m_driver.getLeftTriggerAxis(),
                                            m_driver::getLeftX,
                                            () -> true);
                                        }));

        m_intake.setDefaultCommand(m_intake.disable());

        m_operator.rightBumper().onTrue(m_intake.intake());
        m_operator.rightBumper().onFalse(m_intake.outtake(m_operator::getRightTriggerAxis));

        m_chooserAutonomous.setDefaultOption("Autonomous 1", autonomous1);
        m_chooserAutonomous.addOption("Autonomous 2", autonomous2);
        m_chooserAutonomous.addOption("Shoot Only", shootOnlyAuto);
        SmartDashboard.putData("Autonomous", m_chooserAutonomous);
        CameraServer.startAutomaticCapture();

        mArm = new Arm(armMotor1ID, armMotor2ID);
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
        CommandScheduler.getInstance().run();

        autonomousSelect = m_chooserAutonomous.getSelected();
        SmartDashboard.putData("/DT/ControlType", m_driveControlChooser);

        // double velocity = encoderSRX.getSelectedSensorVelocity(1);
        // SmartDashboard.putNumber("Left Velocity", velocity);

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

        // schedule the autonomous command (example)
        mTimedAutonomous = new TimedAutonomous();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

        // kIntake.shootRing();

        mTimedAutonomous.autoChooser(mDrive, mArm, autonomousSelect, kIntake);

        /*
         * switch (autonomousSelect) {
         * case "Autonomous 1":
         * while (rightMEncoder.getPosition() < (turnDistance / wheelCircumference)) {
         * mDrive.tankDrive(0, .5);
         * }
         * leftMEncoder.setPosition(0);
         * rightMEncoder.setPosition(0);
         *
         * while (rightMEncoder.getPosition() < (autonomousDistance /
         * wheelCircumference)
         * && leftMEncoder.getPosition() < (autonomousDistance / wheelCircumference)) {
         * mDrive.tankDrive(.5, .5);
         * }
         * break;
         *
         * case "Autonomous 2":
         * while (leftMEncoder.getPosition() < (turnDistance / wheelCircumference)) {
         * mDrive.tankDrive(.5, 0);
         * }
         * leftMEncoder.setPosition(0);
         * rightMEncoder.setPosition(0);
         *
         * while (rightMEncoder.getPosition() < (autonomousDistance /
         * wheelCircumference)
         * && leftMEncoder.getPosition() < (autonomousDistance / wheelCircumference)) {
         * mDrive.tankDrive(0.5, .5);
         * }
         * break;
         *
         * default:
         * break;
         * }
         */
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.

        SmartDashboard.putNumber("Intake Speed", .5);
        SmartDashboard.putNumber("Shooter Speed", 1);

        SmartDashboard.putBoolean("Arm Manual Control", false);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        // intakeSpeed = SmartDashboard.getNumber("Intake Speed", .5);
        // shooterSpeed = SmartDashboard.getNumber("Shooter Speed", 1);

        // Intake and shooter controls


        if (mXboxOperator.getLeftBumper()) {
            kIntake.shootRing(.25);
        } else {
            kIntake.stopShooter();
        }

        // Arm Controls
        boolean armControl = SmartDashboard.getBoolean("Arm Manual Control", false);

        if (armControl) {
            mArm.teleop();
        } else {
            if (mXboxOperator.getYButtonPressed()) {
                mArm.goToPosition(mArm.speaker);
            } else if (mXboxOperator.getXButtonPressed()) {
                mArm.goToPosition(mArm.amp);
            } else if (mXboxOperator.getBButtonPressed()) {
                mArm.goToPosition(mArm.down);
            }
        }

        mArm.setTo0();
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