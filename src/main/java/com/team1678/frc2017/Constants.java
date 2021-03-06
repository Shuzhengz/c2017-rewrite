package com.team1678.frc2017;

import com.team1678.frc2017.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team1678.frc2017.subsystems.ServoMotorSubsystem.TalonFXConstants;
import com.team1678.frc2017.subsystems.Limelight.LimelightConstants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.Solenoid;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {
    public static final double kLooperDt = 0.01;
    public static final boolean kDebuggingOutput = false;

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)
    public static final int kCANTimeoutMs = 10; // use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    /* ROBOT PHYSICAL CONSTANTS */
    // Wheels
    public static final double kDriveWheelTrackWidthInches = 31.170;
    public static final double kDriveWheelDiameterInches = 5.67;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0; // Tune me!
    // Tuned dynamics
    public static final double kRobotLinearInertia = 60.0; // kg TODO tune
    public static final double kRobotAngularInertia = 25.0; // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 30.0; // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 0.8; // V
    public static final double kDriveKv = 0.19; // V per rad/s
    public static final double kDriveKa = 0.012; // V per rad/s^2
    public static final double kPathKX = 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches
    public static final double kPathFollowingMaxAccel = 80.0; // inches per second ^ 2
    // PID gains for drive velocity loop
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveVelocityKp = 0.1;
    public static final double kDriveVelocityKi = 0.0;
    public static final double kDriveVelocityKd = 1.0;
    public static final double kDriveVelocityKf = 0.0;
    public static final int kDriveVelocityIZone = 0;
    public static final double kDrivePositionKp = 0.021;
    public static final double kDrivePositionKi = 0.0;
    public static final double kDrivePositionKd = 0.0;
    public static final double kDrivePositionKf = 0.05;
    public static final int kDrivePositionIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    // climber

    //GearIntake
    public static final int kGearIntakeRollerId = 15;
    public static final int kDeploySolenoidId = 0;

    // drive
    public static final int kRightDriveMasterId = 3;
    public static final int kRightDriveSlaveId = 4;

    public static final int kLeftDriveMasterId = 1;
    public static final int kLeftDriveSlaveId = 2;

    public static final int kIndexerId = 5;
    public static final int kIndexerSlaveId = 6;

    // Intake
    public static final int kIntakeRollerId = 15;
    public static final int kDeployGearSolenoidId = 0;

    // Color Panel
    public static final int kColorPanelID = 14;

    public static final double kVelocityConversion = 600.0 / 2048.0;

    // Indexer/Magazine
    public static final int kIndexerLimitSwitch = 6;

    public static final int kIndexerSolenoidID = 2;
    public static final int kUpperConveyorMotor = 5;
    public static final int kSideConveyorMotor = 4;
    public static final int kLowerConveyorMotor = 3;

    public static final double kIndexerKp = 0.2;
    public static final double kIndexerKi = 0.;
    public static final double kIndexerKd = 0.;
    public static final double kIndexerKf = .05;
    public static final double kIndexerVelocityKp = 0.05;
    public static final double kIndexerVelocityKi = 0.;
    public static final double kIndexerVelocityKd = 0.;
    public static final double kIndexerVelocityKf = .05;
    public static final int kIndexerMaxVelocity = 20000; // ticks / 100ms
    public static final int kIndexerMaxAcceleration = 40000; // ticks / 100ms / sec

    public static final int kIndexerSlots = 5;
    public static final int kAnglePerSlot = 360 / kIndexerSlots;
    public static final double kIndexerDeadband = 2.0; // degrees

    public static final double kTestEpsilon = 1e-6;

    // hood
    public static final ServoMotorSubsystemConstants kHoodConstants = new ServoMotorSubsystemConstants();
    static {
        kHoodConstants.kName = "Hood";

        kHoodConstants.kMasterConstants.id = 8;
        kHoodConstants.kMasterConstants.invert_motor = true;
        kHoodConstants.kMasterConstants.invert_sensor_phase = false;

        // Unit == Degrees
        kHoodConstants.kHomePosition = 0.0; // Degrees
        kHoodConstants.kTicksPerUnitDistance = (2048.0 * 93.8) / 360.0;
        kHoodConstants.kKp = 0.5;
        kHoodConstants.kKi = 0;
        kHoodConstants.kKd = 0;
        kHoodConstants.kKf = 0.05;
        kHoodConstants.kMaxIntegralAccumulator = 0;
        kHoodConstants.kIZone = 0; // Ticks
        kHoodConstants.kDeadband = 0; // Ticks

        kHoodConstants.kPositionKp = 0.1;
        kHoodConstants.kPositionKi = 0;
        kHoodConstants.kPositionKd = 0;
        kHoodConstants.kPositionKf = 0.0;
        kHoodConstants.kPositionMaxIntegralAccumulator = 0;
        kHoodConstants.kPositionIZone = 0; // Ticks
        kHoodConstants.kPositionDeadband = 0; // Ticks

        // degrees of limelight pitch from horiz
        kHoodConstants.kMinUnitsLimit = 17.66;
        kHoodConstants.kMaxUnitsLimit = 89.5;

        kHoodConstants.kCruiseVelocity = 20000; // Ticks / 100ms
        kHoodConstants.kAcceleration = 20000; // Ticks / 100ms / s
        kHoodConstants.kRampRate = 0.0; // s
        kHoodConstants.kContinuousCurrentLimit = 35; // amps
        kHoodConstants.kPeakCurrentLimit = 40; // amps
        kHoodConstants.kPeakCurrentDuration = 10; // milliseconds
        kHoodConstants.kMaxVoltage = 12.0;
    }

    public static final double kHoodRadius = 11.904; // radius of hood

    // pigeon
    public static final int kPigeonIMUId = 16;

    public static final boolean kUseDriveGamepad = false;
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 1;
    public static final int kMainTurnJoystickPort = 0;
    public static final double kJoystickThreshold = 0.2;

    // Roller
    public static final int kRollerId = 0;
    public static final int kRollerSolenoid = 2;

    // solenoids
    public static final int kPCMId = 20;
    public static final int kPDPId = 21;

    // limelight
    public static final LimelightConstants kLimelightConstants = new LimelightConstants();
    static {
        kLimelightConstants.kName = "Limelight";
        kLimelightConstants.kTableName = "limelight";
        kLimelightConstants.kHeight = 24.5; // inches
        kLimelightConstants.kTurretToLens = Pose2d.identity();
        kLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(0.0);
    }

    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    public static final double kMaxTrackerDistance = 15.0;
    public static final double kMaxGoalTrackAge = 30.0;
    public static final double kMaxGoalTrackAgeNotTracking = 0.3;
    public static final double kMaxGoalTrackSmoothingTime = 1.5;
    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;
    public static final boolean kEnableCachedGoal = true;
    
    public static final double kCameraFrameRate = 90.0;
    public static final double kMinStability = 0.5;
    public static final int kPortPipeline = 0;
    public static final int kBallPipeline = 2;
    public static final double kGoalHeight = 90.0;

    public static final int kCanifierId = 0;

    // shooter
    public static final int kMasterFlywheelID = 9;
    public static final int kSlaveFlywheelID = 10;
    public static final int kTriggerWheelID = 6;
    public static final int kTriggerPopoutSolenoidID = 1;
    public static final double kShooterP = 0.2;
    public static final double kShooterI = 0.00004;
    public static final double kShooterD = 0.0;
    public static final double kShooterF = 0.05;
    public static final double kTriggerP = 0.05;
    public static final double kTriggerI = 0.0;
    public static final double kTriggerD = 0.0;
    public static final double kTriggerF = 0.05;

    public static final double kTriggerRPM = 5000.0;

    public static final double kMaxTopLimelightHeight = 16.0;

    public static final double kGenerateTrajectoryTime = 0.5;
    public static final double kUseNextTrajectoryTime = 0.75;
    public static final Rotation2d kMaxDeviance = Rotation2d.fromDegrees(0); // max angle away from ball that robot can
                                                                             // be and still pick it up

    // Drive control
    public static final double kStingerForwardPower = 0.8;

    public static final double kInnerGoalDepth = 0;
	public static final double kHoodToTurret = 4.25; // center of the turret to the axis of rotation of the hood
	public static final double kLimelightPitchOffset = 17.66; // limelight pitch at hood 0
	public static final double kAutoAimPredictionTime = 4.0; // lookahead for robot state during aiming
	public static final double kJoystickJogThreshold = 0.1;     
	public static final int kCameraStreamPort = 5810;

    public static Solenoid makeSolenoidForId(int solenoidId) {
        if (solenoidId < 8) {
            return new Solenoid(kPCMId, solenoidId);
        }
        throw new IllegalArgumentException("Solenoid ID not valid: " + solenoidId);
    }

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }

        return "";
    }
}
