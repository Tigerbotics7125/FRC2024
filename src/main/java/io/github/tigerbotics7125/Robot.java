/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.tigerbotics7125.commands.ExampleCommand;
import io.github.tigerbotics7125.subsystems.ExampleSubsystem;

// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // private RobotContainer m_robotContainer;

    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    // Additional controllers may be added if needed.

    private WPI_TalonSRX leftMotor = new WPI_TalonSRX(1);
    private WPI_TalonSRX rightMotor = new WPI_TalonSRX(2);
    private DifferentialDrive mDrive = new DifferentialDrive(leftMotor, rightMotor);
    private XboxController mXbox = new XboxController(0);
    String tankDrive = "Tank Drive";
    String arcadeDrive = "Arcade Drive";
    String driveSelect;
    SendableChooser<String> m_chooser = new SendableChooser<>();

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
        configureBindings();
        leftMotor.setInverted(true);

        m_chooser.setDefaultOption("Tank Drive", tankDrive);
        m_chooser.addOption("Arcade Drive", arcadeDrive);
        SmartDashboard.putData("Drive choices", m_chooser);
        CameraServer.startAutomaticCapture();
        CameraServer.getVideo();
        //CameraServer.getInstance().startAutomaticCapture();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule 'ExampleCommand' when 'exampleCondition' changes to 'true'
        new Trigger(m_exampleSubsystem::exampleCondition)
                .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Schedule 'exampleMethodCommand' when the Xbox controller's B button is
        // pressed,
        // cancelling on release.

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

        // schedule the autonomous command (example)

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        driveSelect = m_chooser.getSelected();
      
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        driveSelect = m_chooser.getSelected();
        System.out.println("Drive mode: " + driveSelect);

        switch (driveSelect) {
            case "Tank Drive":
                mDrive.tankDrive(mXbox.getLeftY(), mXbox.getRightY());
                break;

            case "Arcade Drive":
                mDrive.arcadeDrive(mXbox.getLeftY(), mXbox.getLeftX(), false);

                break;

            default:
                break;
        }

        SmartDashboard.putNumber("Left Motor Value", leftMotor.get());
        SmartDashboard.putNumber("Right Motor Value", rightMotor.get());
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
