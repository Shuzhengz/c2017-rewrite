/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2017;

import com.team1678.frc2017.controlboard.ControlBoard;
import com.team1678.frc2017.loops.Looper;
import com.team1678.frc2017.paths.TrajectoryGenerator;
import com.team1678.frc2017.subsystems.superstructure.Superstructure;
import com.team1678.frc2017.subsystems.superstructure.climber.Climber;
import com.team1678.frc2017.subsystems.superstructure.intake.GearIntake;
import com.team1678.frc2017.subsystems.superstructure.magazine.Magazine;
import com.team1678.frc2017.subsystems.superstructure.intake.Intake;
import com.team1678.frc2017.subsystems.superstructure.shooter.Hood;
import com.team1678.frc2017.subsystems.superstructure.shooter.Shooter;
import com.team1678.frc2017.subsystems.superstructure.trigger.Trigger;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.wpilib.TimedRobot;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1678.frc2017.subsystems.*;
import com.team254.lib.util.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
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
  private final Magazine mMagazine = Magazine.getInstance();
  private final Infrastructure mInfrastructure = Infrastructure.getInstance();
  private final Limelight mLimelight = Limelight.getInstance();

  private final Intake mIntake = Intake.getInstance();
  private final GearIntake mGearIntake = GearIntake.getInstance();
  private final Superstructure mSuperstructure = Superstructure.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Trigger mTrigger = Trigger.getInstance();
  private final Climber mClimber = Climber.getInstance();
  private final Hood mHood = Hood.getInstance();
  private final Canifier mCanifier = Canifier.getInstance();

  private final RobotState mRobotState = RobotState.getInstance();
  private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
  private boolean climb_mode = false;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private boolean mPivoted = false;

  private Joystick joystick = new Joystick(0);

  private WheelDrive frontLeft = new WheelDrive(6, 7, 3);
  private WheelDrive frontRight = new WheelDrive(4, 5, 2);
  private WheelDrive backLeft = new WheelDrive(2, 3, 1);
  private WheelDrive backRight = new WheelDrive(0, 1, 0);

  public Robot() {
    CrashTracker.logRobotConstruction();
    mTrajectoryGenerator.generateTrajectories();
  }

  @Override
  public void robotPeriodic() {
    RobotState.getInstance().outputToSmartDashboard();
    mSubsystemManager.outputToSmartDashboard();
    mEnabledLooper.outputToSmartDashboard();

    SmartDashboard.putBoolean("Climb Mode", climb_mode);
    SmartDashboard.putBoolean("Pivoted", mPivoted);
  }

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    try {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
      MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0", Constants.kCameraStreamPort);
      cameraServer.setSource(camera);

      CrashTracker.logRobotInit();

      mSubsystemManager.setSubsystems(
              mRobotStateEstimator,
              mCanifier,
              mDrive,
              mHood,
              mLimelight,
              mIntake,
              mGearIntake,
              mMagazine,
              mShooter,
              mTrigger,
              mSuperstructure,
              mInfrastructure,
              mClimber
      );

      mSubsystemManager.registerEnabledLoops(mEnabledLooper);
      mSubsystemManager.registerDisabledLoops(mDisabledLooper);

      // Robot starts forwards.
      mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
      mDrive.setHeading(Rotation2d.identity());

      mTrajectoryGenerator.generateTrajectories();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }


  //No Auto in rewrite
  /*
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }*/

  /*
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
  }*/

  /**
   * This function is called once when teleop is enabled.
   */

  @Override
  public void teleopInit() {
      try {
        CrashTracker.logTeleopInit();
        mDisabledLooper.stop();
        mClimber.setBrakeMode(true);

        mInfrastructure.setIsDuringAuto(false);

        //mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
        mEnabledLooper.start();
        mLimelight.setLed(Limelight.LedMode.ON);
        mLimelight.setPipeline(Constants.kPortPipeline);
        mHood.setNeutralMode(NeutralMode.Brake);

        mControlBoard.reset();
      } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
      }
    }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    try {
      double timestamp = Timer.getFPGATimestamp();
      double throttle = mControlBoard.getThrottle();
      double turn = mControlBoard.getTurn();
      double hood_jog = mControlBoard.getJogHood();
      Rotation2d turret_jog = mControlBoard.getJogTurret();

      if (mControlBoard.getShotUp()) {
        mSuperstructure.setAngleAdd(1.0);
      } else if (mControlBoard.getShotDown()) {
        mSuperstructure.setAngleAdd(-1.0);
      }

      mDrive.//setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn()));
              setCheesyishDrive(throttle, turn, mControlBoard.getQuickTurn());

      //mLimelight.setLed(Limelight.LedMode.ON);

      // mSuperstructure.setWantFieldRelativeTurret(Rotation2d.fromDegrees(180.0));//mControlBoard.getTurretCardinal().rotation);

      if (mControlBoard.climbMode()) {
        climb_mode = true;
        mPivoted = false;
      }

      if (!climb_mode){
        mSuperstructure.enableIndexer(true);
        mSuperstructure.setWantUnjam(mControlBoard.getWantUnjam());

        mSuperstructure.setManualZoom(mControlBoard.getManualZoom());

        if (mSuperstructure.getWantShoot()) {
          mControlBoard.setRumble(true);
        } else {
          mControlBoard.setRumble(false);
        }

        mSuperstructure.setWantHoodScan(mControlBoard.getWantHoodScan());

        if (mControlBoard.getFendorShot()) {
          mSuperstructure.setWantFendor();
          //mSuperstructure.setWantFieldRelativeTurret(Rotation2d.fromDegrees(180.));
        } else {
          mSuperstructure.setWantAutoAim(Rotation2d.fromDegrees(180.0));
        }

        if (mControlBoard.getShoot()) {
          if (mSuperstructure.isAimed() || mSuperstructure.getWantFendor() || mSuperstructure.getWantSpit() || mSuperstructure.getLatestAimingParameters().isEmpty()) {
            mSuperstructure.setWantShoot();
          }
        } else if (mControlBoard.getPreShot()) {
          mSuperstructure.setWantPreShot(true);
        } else if (mControlBoard.getSpinUp()) {
          mSuperstructure.setWantSpinUp();
        } else if (mControlBoard.getTuck()) {
          mSuperstructure.setWantTuck(true);
        } else if (mControlBoard.getUntuck()) {
          mSuperstructure.setWantTuck(false);
        } else if (mControlBoard.getTurretReset()) {
          mRobotState.resetVision();
          mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
        } else if (mControlBoard.getTestSpit()) {
          mSuperstructure.setWantTestSpit();
        } else if (mControlBoard.getRunIntake()) {
          if (!mSuperstructure.getWantShoot()) {
            mIntake.setState(Intake.WantedAction.INTAKE);
          } else {
            mIntake.setState(Intake.WantedAction.STAY_OUT);
          }
          mSuperstructure.setAutoIndex(false);
        } else if (mControlBoard.getRetractIntake()) {
          mIntake.setState(Intake.WantedAction.RETRACT);
        } else {
          mIntake.setState(Intake.WantedAction.NONE);
          //mRoller.stop();
        }
      } else {
        Climber.WantedAction climber_action = Climber.WantedAction.NONE;
        mSuperstructure.enableIndexer(false);
        mIntake.setState(Intake.WantedAction.NONE);
        mSuperstructure.setWantSpinUp(false);
        mSuperstructure.setWantShoot(false);
        mSuperstructure.setWantPreShot(false);
        mSuperstructure.setWantUnjam(false);

        if (mControlBoard.getArmHug()) { // Press B
          climber_action = (Climber.WantedAction.SPIN_UP); // hook onto the rung
        } else if (mControlBoard.getClimb()) { // Press Y
          climber_action = (Climber.WantedAction.CLIMB);
        } else if (mControlBoard.getManualArmRetract()) { // Press and hold right joystick
          climber_action = (Climber.WantedAction.MANUAL_CLIMB);
        } else if (mControlBoard.getBrake()) { // Release Y
          climber_action = (Climber.WantedAction.BRAKE);
        } else if (mControlBoard.getLeaveClimbMode()) {
          climb_mode = false;
        }

//                if (mControlBoard.getStopExtend() || mControlBoard.getStopClimb()) {
//                    climber_action = Climber.WantedAction.STOP;
//                }

        mClimber.setState(climber_action);
      }
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    try {
      CrashTracker.logDisabledInit();
      mEnabledLooper.stop();
      mClimber.setBrakeMode(true);

      //mRobotState.resetVision();

      mInfrastructure.setIsDuringAuto(true);

      Drive.getInstance().zeroSensors();
      RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

      mDisabledLooper.start();

      mLimelight.setLed(Limelight.LedMode.ON);
      mLimelight.triggerOutputs();

      mHood.setNeutralMode(NeutralMode.Coast);
      mDrive.setBrakeMode(false);
      mLimelight.writePeriodicOutputs();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
    SmartDashboard.putString("Match Cycle", "DISABLED");

    try {
      mLimelight.setLed(Limelight.LedMode.OFF);
      mLimelight.writePeriodicOutputs();

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
    SmartDashboard.putString("Match Cycle", "TEST");

    try {
      System.out.println("Starting check systems.");

      mDisabledLooper.stop();
      mEnabledLooper.stop();

      mDrive.checkSystem();

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
