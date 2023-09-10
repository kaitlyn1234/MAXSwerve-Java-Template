// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  CANSparkMax rightliftmotor = new CANSparkMax(9, MotorType.kBrushless);; // Encoder 0 = level, down is +, up is - (incrementing clockwise when looking at right side)
  CANSparkMax leftliftmotor = new CANSparkMax(12, MotorType.kBrushless);;
  CANSparkMax wrist = new CANSparkMax(11, MotorType.kBrushless);; // Encoder, 0 = straight, down is +, up is - (incrementing clockwise when looking at right side)
  CANSparkMax leftintake= new CANSparkMax(10, MotorType.kBrushless);;
  CANSparkMax rightintake= new CANSparkMax(14, MotorType.kBrushless);;
  Joystick stick = new Joystick(2);
  
  PIDController lift_pos_pid = new PIDController(0.5, 0.0, 0.0);
  PIDController wrist_pos_pid = new PIDController(0.5, 0.0, 0.0);

  double wrist_setpoint = 0;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    wrist_pos_pid.enableContinuousInput(-Math.PI, Math.PI);
    lift_pos_pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double wrapAngle(double ang) {
    return Math.atan2(Math.sin(ang), Math.cos(ang));
  }

  public double getWristAngle() {
      return wrapAngle(-wrist.getAbsoluteEncoder(Type.kDutyCycle).getPosition() * Math.PI * 2);
  }

  public double getLiftAngle() {
    return wrapAngle(-rightliftmotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition() * Math.PI * 2);
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    SmartDashboard.putNumber("lift encoder", getLiftAngle());
    SmartDashboard.putNumber("wrist encoder", getWristAngle());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    //LIFT TOGETHER

    if (stick.getRawButton(3)) {
      //up
      rightliftmotor.set(-0.05);
      leftliftmotor.set(0.05);
    }
    else if (stick.getRawButton(4)) {
      //down
      rightliftmotor.set(0.05);
      leftliftmotor.set(-0.05);
    }
    else {
      rightliftmotor.set(0);
      leftliftmotor.set(0);
    }

    //WRIST

    wrist_setpoint = stick.getY();
    double wrist_cmd = wrist_pos_pid.calculate(getWristAngle(), wrist_setpoint);
    wrist.set(wrist_cmd);

    //INTAKE

    
    if (stick.getRawButton(11)) {
      //in
      rightintake.set(0.05);
      leftintake.set(0.05);
    }
    else if (stick.getRawButton(12)) {
      //out
      rightintake.set(-0.05);
      leftintake.set(-0.05);
    }
    else {
      rightintake.set(0);
      leftintake.set(0);
    }
    

/* 
    //TEST INDIVIDUAL LIFT MOTORS

     if (stick.getRawButton(3)) {
      //up
      rightliftmotor.set(-0.05);
      
    }
    else if (stick.getRawButton(4)) {
      //down
      rightliftmotor.set(0.05);
   
    }
    else{
      rightliftmotor.set(0);
  
    }

    if (stick.getRawButton(5)) {
      //down
      leftliftmotor.set(-0.05);
      
    }
    else if (stick.getRawButton(6)) {
      //up
      leftliftmotor.set(0.05);
   
    }
    else{
      leftliftmotor.set(0);
  
    }
*/
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
