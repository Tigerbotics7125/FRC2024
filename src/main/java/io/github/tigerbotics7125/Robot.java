/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.tigerbotics7125.Constants.Arm.ArmState;
import io.github.tigerbotics7125.Constants.DriveTrain.ControlType;
import io.github.tigerbotics7125.autos.*;
import io.github.tigerbotics7125.subsystems.Arm;
import io.github.tigerbotics7125.subsystems.Drivetrain;
import io.github.tigerbotics7125.subsystems.Intake;
import io.github.tigerbotics7125.subsystems.Shooter;
import java.util.Map;

public class Robot extends TimedRobot {

    private CommandXboxController m_driver =
            new CommandXboxController(Constants.HID.kDriverControllerPort);
    private CommandXboxController m_operator =
            new CommandXboxController(Constants.HID.kOperatorControllerPort);

    private Drivetrain m_drivetrain = new Drivetrain();
    private Intake m_intake = new Intake();
    private Shooter m_shooter = new Shooter();
    private Arm m_arm = new Arm();

    SendableChooser<PathPlannerAuto> m_autoChooser = new SendableChooser<>();

    SendableChooser<Constants.DriveTrain.ControlType> m_driveControlChooser =
            new SendableChooser<>();

    @Override
    public void robotInit() {
        m_autoChooser.setDefaultOption("Example", new PathPlannerAuto("Example Path"));
        m_driveControlChooser.setDefaultOption(
                ControlType.CURVE_ROCKETLEAGUE.name(), ControlType.CURVE_ROCKETLEAGUE);
        for (ControlType controlType : ControlType.values()) {
            if (controlType.equals(ControlType.CURVE_ROCKETLEAGUE)) continue;
            m_driveControlChooser.addOption(controlType.name(), controlType);
        }
        new Trigger(RobotController::getUserButton)
                .onTrue(
                        m_drivetrain
                                .setIdleMode(IdleMode.kBrake)
                                .andThen(m_arm.setIdleMode(IdleMode.kBrake)));
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
        m_shooter.setDefaultCommand(m_shooter.pidControl());
        m_arm.setDefaultCommand(m_arm.disable());

        m_operator.rightBumper().onTrue(m_intake.intake());
        m_operator.rightBumper().onFalse(m_intake.outtake(m_operator::getRightTriggerAxis));
        m_operator.leftBumper().onTrue(m_shooter.shootNote(m_intake));
        m_operator.leftBumper().onFalse(m_shooter.disable());
        m_operator.y().onTrue(m_arm.pidControl(ArmState.SPEAKER));
        m_operator.x().onTrue(m_arm.pidControl(ArmState.AMP));
        m_operator.b().onTrue(m_arm.pidControl(ArmState.INTAKE));
        m_operator.a().onTrue(m_arm.disable());
        // TODO tell seth this is different.
        m_operator.leftStick().onTrue(m_arm.resetEncoder());

        CameraServer.startAutomaticCapture();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putData("DTControlType", m_driveControlChooser);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        // Auto auto = m_autoChooser.getSelected();

        // auto.autoCommand()
        //         .ifPresentOrElse(
        //                 cmd ->
        //                         CommandScheduler.getInstance()
        //                                 .schedule(
        //                                         auto.preCommand()
        //                                                 .andThen(cmd)
        //                                                 .andThen(auto.postCommand())),
        //                 () ->
        //                         CommandScheduler.getInstance()
        //                                 .schedule(Commands.print("PATH PLANNER NOT
        // IMPLEMENTED")));
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // Make sure autonomous commands are canceled for teleop
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
