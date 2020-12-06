/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2017;

import java.util.Optional;

import com.team1678.frc2017.controlboard.ControlBoard;
import com.team1678.frc2017.loops.Looper;
import com.team1678.frc2017.paths.TrajectoryGenerator;
import com.team1678.frc2017.controlboard.ControlBoard;
import com.team1678.frc2017.controlboard.GamepadButtonControlBoard;
import com.team1678.frc2017.subsystems.superstructure.Superstructure;
import com.team1678.frc2017.subsystems.superstructure.climber.Climber;
import com.team1678.frc2017.subsystems.superstructure.gearIntake.GearIntake;
import com.team1678.frc2017.subsystems.superstructure.indexer.Indexer;
import com.team1678.frc2017.subsystems.superstructure.intake.Intake;
import com.team1678.frc2017.subsystems.superstructure.shooter.Hood;
import com.team1678.frc2017.subsystems.superstructure.shooter.Shooter;
import com.team1678.frc2017.subsystems.superstructure.trigger.Trigger;
import com.team254.lib.wpilib.TimedRobot;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1678.frc2017.SubsystemManager;
import com.team1678.frc2017.subsystems.*;
import com.team254.lib.util.*;
import com.team254.lib.wpilib.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;

//import com.team1678.frc2017.auto.AutoModeExecutor;
//import com.team1678.frc2017.auto.modes.AutoModeBase;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot<Turret> extends TimedRobot {

  private final Looper mEnabledLooper = new Looper();
  private final Looper mDisabledLooper = new Looper();

  private final ControlBoard mControlBoard = ControlBoard.getInstance();
  private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
  private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  private final Drive mDrive = Drive.getInstance();
  private final Indexer mIndexer = Indexer.getInstance();
  //private final Infrastructure mInfrastructure = Infrastructure.getInstance();
  private final Limelight mLimelight = Limelight.getInstance();

  private final Intake mIntake = Intake.getInstance();
  private final GearIntake mGearIntake = GearIntake.getInstance();
  private final Superstructure mSuperstructure = Superstructure.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Trigger mTrigger = Trigger.getInstance();
  private final Climber mClimber = Climber.getInstance();
  private final Hood mHood = Hood.getInstance();

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
