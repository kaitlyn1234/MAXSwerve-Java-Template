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
  
  PIDController lift_pos_pid = new PIDController(0.2, 0.0, 0.0);
  PIDController wrist_pos_pid = new PIDController(0.2, 0.0, 0.0);

  double lift_setpoint_lower_limit = 0.3;
  double lift_setpoint_upper_limit = 2 * Math.PI;
  double wrist_setpoint_lower_limit = 0.45;
  double wrist_setpoint_upper_limit = 5.45;

  double wrist_joystick_speed = 0.01;
  double lift_joystick_speed = 0.01;

  double wrist_setpoint = 0;
  double lift_setpoint = 0;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // wrist_pos_pid.enableContinuousInput(-Math.PI, Math.PI);
    // lift_pos_pid.enableContinuousInput(-Math.PI, Math.PI);

    lift_setpoint = getLiftFeedback();
    wrist_setpoint = getWristFeedback();
  }

  public double wrapAngle(double ang) {
    return Math.atan2(Math.sin(ang), Math.cos(ang));
  }

  public double getWristAngle() {
      return wrapAngle(-wrist.getAbsoluteEncoder(Type.kDutyCycle).getPosition() * Math.PI * 2);
  }

  public double getLiftAngle() {
    return wrapAngle(-rightliftmotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition() * Math.PI * 2 - Math.PI / 2.0);
  }

  public double getLiftFeedback() {
    // Offset so horizontal angle is 90 deg
    return getLiftAngle() + Math.PI;
  }

  public double getWristFeedback() {
    // Offset so the centered angle is pi
    return getWristAngle() + Math.PI;
  }

  public void clampSetpoints() {
    if (lift_setpoint < lift_setpoint_lower_limit) { lift_setpoint = lift_setpoint_lower_limit; }
    if (lift_setpoint > lift_setpoint_upper_limit) { lift_setpoint = lift_setpoint_upper_limit; }

    if (wrist_setpoint < wrist_setpoint_lower_limit) { lift_setpoint = wrist_setpoint_lower_limit; }
    if (wrist_setpoint > wrist_setpoint_upper_limit) { lift_setpoint = wrist_setpoint_upper_limit; }
  }

  public void controlWrist() {
    double wrist_cmd = wrist_pos_pid.calculate(getWristFeedback(), wrist_setpoint);
    wrist.set(-wrist_cmd);
  }

  public double getLiftFF() {
    return 0.04 * Math.cos(getLiftAngle());
  }

  public void controlLift() {
    double lift_fb = getLiftFeedback();
    double lift_cmd = lift_pos_pid.calculate(lift_fb, lift_setpoint);
    lift_cmd = lift_cmd + getLiftFF();
    rightliftmotor.set(-lift_cmd);
    leftliftmotor.set(lift_cmd);
  }

  public void arbitrateSetpoints() {
    if (stick.getRawButton(9)) {
      wrist_setpoint = Math.PI; // Straight
      lift_setpoint = Math.PI / 2; // Parallel to floor
    }
    else {
      double stick_x = stick.getX();
      double stick_y = stick.getY();
      if (Math.abs(stick_x) > 0.1) {
        lift_setpoint = lift_setpoint + stick_x * lift_joystick_speed;
      }
  
      if (Math.abs(stick_y) > 0.1) {
        wrist_setpoint = wrist_setpoint + stick_y * wrist_joystick_speed;
      }
    }
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

    SmartDashboard.putNumber("lift feedback", getLiftFeedback());
    SmartDashboard.putNumber("wrist feedback", getWristFeedback());
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

    arbitrateSetpoints();
    clampSetpoints();
    controlWrist();
    controlLift();

    //INTAKE
    if (stick.getRawButton(11)) {
      //in
      rightintake.set(0.3);
      leftintake.set(0.3);
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
