// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private CANSparkMax tiltMotor;
    private CANSparkMax extMotor;
    private final XboxController m_driver2Controller = new XboxController(1);
    private CANSparkMax armMotor;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();


        tiltMotor = new CANSparkMax(9, MotorType.kBrushless);
        extMotor = new CANSparkMax(10, MotorType.kBrushless);
        armMotor = new CANSparkMax(11, MotorType.kBrushless);

        tiltMotor.restoreFactoryDefaults();
        extMotor.restoreFactoryDefaults();
        armMotor.restoreFactoryDefaults();

        tiltMotor.setIdleMode(CANSparkMax.IdleMode.kBrake); 
        extMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        final RelativeEncoder extEncoder;
        final RelativeEncoder tiltEncoder;
        final RelativeEncoder armEncoder;

        extEncoder = extMotor.getEncoder();
        tiltEncoder = tiltMotor.getEncoder();
        armEncoder = armMotor.getEncoder();

        tiltEncoder.setPosition(0);
        armEncoder.setPosition(0);
        extEncoder.setPosition(0);

        tiltMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        tiltMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        extMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        extMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        
        tiltMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,0 );
        tiltMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -575);

        extMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,0);
        extMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -455);

        armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 3);
        armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -2);


    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
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
        CommandScheduler.getInstance().run();

        //m_robotContainer.periodic();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
       m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }



    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        double rightJoystick = m_driver2Controller.getRightY();
        double leftJoystick = -m_driver2Controller.getLeftY();
        double arm = 0;


        if(Math.abs(rightJoystick) < 0.05)
        {
            rightJoystick = 0;
        }
        if(Math.abs(leftJoystick) < 0.05)
        {
            leftJoystick = 0;
        }

        extMotor.set(rightJoystick);
        tiltMotor.set(leftJoystick);

        if (m_driver2Controller.getBButton()){
            arm = 0.15;
        }
        else if( m_driver2Controller.getXButton()){
            arm = -0.15;
        }
        else {
            arm = 0;
        }

        armMotor.set(arm);

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
