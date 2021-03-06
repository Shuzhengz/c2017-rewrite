package com.team1678.frc2017.subsystems.superstructure;

import java.util.Optional;

import com.team1678.frc2017.Constants;
import com.team1678.frc2017.RobotState;
import com.team1678.frc2017.loops.ILooper;
import com.team1678.frc2017.loops.Loop;
import com.team1678.frc2017.states.SuperstructureConstants;
import com.team1678.frc2017.subsystems.*;
import com.team1678.frc2017.subsystems.superstructure.magazine.Magazine;
import com.team1678.frc2017.subsystems.superstructure.intake.Intake;
import com.team1678.frc2017.subsystems.superstructure.intake.GearIntake;
import com.team1678.frc2017.subsystems.superstructure.shooter.Hood;
import com.team1678.frc2017.subsystems.superstructure.shooter.Shooter;
import com.team1678.frc2017.subsystems.superstructure.trigger.Trigger;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.util.Units;
import com.team254.lib.util.Util;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {
    private static final double kZoomedOutRange = 190.0;
    private static final double kZoomedInRange = 220.0;

    // Instances
    private static Superstructure mInstance;
    private final Shooter mShooter = Shooter.getInstance();
    private final Trigger mTrigger = Trigger.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final GearIntake mGearIntake = GearIntake.getInstance();
    private final Magazine mMagazine = Magazine.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();
    private boolean mAutoIndex = false;

    private Rotation2d mFieldRelativeTurretGoal = null;

    private boolean mHasTarget = false;
    private boolean mOnTarget = false;
    private int mTrackId = -1;

    private double mHoodFeedforwardV = 0.0;
    private double mTurretFeedforwardV = 0.0;
    private Optional<AimingParameters> mLatestAimingParameters = Optional.empty();
    private double mCorrectedRangeToTarget = 0.0;
    private boolean mEnforceAutoAimMinDistance = false;
    private double mAutoAimMinDistance = 500;

    private boolean mWantsShoot = false;
    private boolean mWantsSpinUp = false;
    private boolean mWantsTuck = false;
    private boolean mWantsFendor = false;
    private boolean mWantsTestSpit = false;
    private boolean mUseInnerTarget = false;
    private boolean mWantsPreShot = false;
    private boolean mWantsUnjam = false;
    private boolean mWantsHoodScan = false;

    private double mCurrentHood = 0.0;
    private double mUpperConveyorVoltage = 6.0;
    private double mSideConveyorVoltage = 7.0;
    private double mLowerConveyorVoltage = 8.0;

    private double mHoodSetpoint = 70.5;
    private double mShooterSetpoint = 4000.0;
    private boolean mGotSpunUp = false;
    private boolean mEnableIndexer = true;
    private boolean mManualZoom = false;
    private boolean mDisableLimelight = false;

    private double mAngleAdd = 0.0;

    public synchronized void enableIndexer(boolean indexer) {
        mEnableIndexer = indexer;
    }

    private double mTurretThrottle = 0.0;
    private TimeDelayedBoolean trigger_popout = new TimeDelayedBoolean();
    private boolean estim_popout = false;

    public synchronized boolean spunUp() {
        return mGotSpunUp;
    }
    
    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Superstructure() {
    }

    public boolean getWantShoot() {
        return mWantsShoot;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Superstructure.this) {
                    if (SuperstructureConstants.kUseSmartdashboard) {
                        SmartDashboard.putNumber("Shooting RPM", mShooterSetpoint);
                        SmartDashboard.putNumber("Hood Angle", mHoodSetpoint);
                    }
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    maybeUpdateGoalFromVision(timestamp);
                    maybeUpdateGoalFromFieldRelativeGoal(timestamp);
                    maybeUpdateGoalFromHoodScan(timestamp);
                    followSetpoint();
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized boolean getWantsShoot() {
        return mWantsShoot;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Shooting", mWantsShoot);
        SmartDashboard.putBoolean("Spinning Up", mWantsSpinUp);
        SmartDashboard.putBoolean("Pre Shot", mWantsPreShot);

        SmartDashboard.putBoolean("Test Spit", mWantsTestSpit);
        SmartDashboard.putBoolean("Fendor Shot", mWantsFendor);
        SmartDashboard.putBoolean("Tuck", mWantsTuck);

        SmartDashboard.putNumber("Angle Add", -mAngleAdd);

        SmartDashboard.putNumber("Hood Goal", mHoodSetpoint);
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public synchronized boolean isAimed() {
        return mOnTarget;
    }

    private double getShootingSetpointRpm(double range) {
        if (SuperstructureConstants.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Shooting RPM", 0);
        } else if (SuperstructureConstants.kUseFlywheelAutoAimPolynomial) {
            return SuperstructureConstants.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return SuperstructureConstants.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    private double getHoodSetpointAngle(double range) {
        if (SuperstructureConstants.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Hood Angle", 0);
        } else if (SuperstructureConstants.kUseHoodAutoAimPolynomial) {
            return SuperstructureConstants.kHoodAutoAimPolynomial.predict(range);
        } else {
            return SuperstructureConstants.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value + mAngleAdd;
        }
    }

    public synchronized double getCorrectedRangeToTarget() {
        return mCorrectedRangeToTarget;
    }

    // Jog Hood
    public synchronized void setWantHoodScan(boolean scan) {
        if (scan != mWantsHoodScan) {
            if (scan) {
                mHoodSetpoint = Constants.kHoodConstants.kMinUnitsLimit + 10;
            } else {
                mHoodSetpoint = mHood.getAngle();
            }
        }
        mWantsHoodScan = scan;
    }

    public synchronized boolean getScanningHood() {
        return mWantsHoodScan;
    }

    public synchronized double getAngleAdd() {
        return mAngleAdd;
    }
    
    public synchronized void setAngleAdd(double add) {
        mAngleAdd -= add;
    }

    public synchronized void setGoal(double shooter, double hood) {
        if (mHasTarget) {
            // Keep current setpoints
        } else {
            mHoodSetpoint = hood;
            mShooterSetpoint = shooter;
        }
    }

    public synchronized void resetAimingParameters() {
        mHasTarget = false;
        mOnTarget = false;
        mTurretFeedforwardV = 0.0;
        mTrackId = -1;
        mLatestAimingParameters = Optional.empty();
    }

    public synchronized boolean getDisableLimelight() {
        return mDisableLimelight;
    }

    public void safetyReset() {
        if (mHoodSetpoint < Constants.kHoodConstants.kMinUnitsLimit) {
            // logic for when hood fully in
            mHoodSetpoint = Constants.kHoodConstants.kMinUnitsLimit;
        }
        if (mHoodSetpoint > Constants.kHoodConstants.kMaxUnitsLimit) {
            mHoodSetpoint = Constants.kHoodConstants.kMaxUnitsLimit;
            // logic for when hood fully extended
        }
    }

    public synchronized void maybeUpdateGoalFromHoodScan(double timestamp) {
        if (!mWantsHoodScan) {
            return;
        }

        if (Util.epsilonEquals(mHood.getAngle(), Constants.kHoodConstants.kMinUnitsLimit + 10, 10.0)) {
            mHoodSetpoint = Constants.kHoodConstants.kMaxUnitsLimit - 10;
        } else if (Util.epsilonEquals(mHood.getAngle(), Constants.kHoodConstants.kMaxUnitsLimit - 10, 10.0)) {
            mHoodSetpoint = Constants.kHoodConstants.kMinUnitsLimit + 10;
        }
    }

    public synchronized void maybeUpdateGoalFromVision(double timestamp) {

        if (mWantsShoot && mGotSpunUp) {
            mLatestAimingParameters = mRobotState.getAimingParameters(mUseInnerTarget, mTrackId, Constants.kMaxGoalTrackAge);
        } else {
            mLatestAimingParameters = mRobotState.getAimingParameters(mUseInnerTarget, -1, Constants.kMaxGoalTrackAge);
        }

        if (mLatestAimingParameters.isPresent()) {
            mTrackId = mLatestAimingParameters.get().getTrackId();

            Pose2d robot_to_predicted_robot = mRobotState.getLatestFieldToVehicle().getValue().inverse()
                    .transformBy(mRobotState.getPredictedFieldToVehicle(Constants.kAutoAimPredictionTime));
            Pose2d predicted_turret_to_goal = robot_to_predicted_robot.inverse()
                    .transformBy(mLatestAimingParameters.get().getTurretToGoal());
            mCorrectedRangeToTarget = predicted_turret_to_goal.getTranslation().norm();

            // Don't aim if not in min distance
            if (mEnforceAutoAimMinDistance && mCorrectedRangeToTarget > mAutoAimMinDistance) {
                return;
            }

            final double shooting_setpoint = getShootingSetpointRpm(mCorrectedRangeToTarget);
            mShooterSetpoint = shooting_setpoint;

            final double aiming_setpoint = getHoodSetpointAngle(mCorrectedRangeToTarget);
           
            if (!mWantsHoodScan) {
                mHoodSetpoint = aiming_setpoint;
            }

            final Twist2d velocity = mRobotState.getMeasuredVelocity();
            // Angular velocity component from tangential robot motion about the goal.
            final double tangential_component = mLatestAimingParameters.get().getTurretToGoalRotation().sin()
                    * velocity.dx / mLatestAimingParameters.get().getRange();
            final double angular_component = Units.radians_to_degrees(velocity.dtheta);
            // Add (opposite) of tangential velocity about goal + angular velocity in local
            // frame.
            mTurretFeedforwardV = -(angular_component + tangential_component);

            safetyReset();

            mHasTarget = true;
            final double hood_error = mCurrentHood - mHoodSetpoint;

            if (Util.epsilonEquals(0, 0.0, 3.0) && Util.epsilonEquals(hood_error, 0.0, 3.0)) {
                mOnTarget = true;
            } else {
                mOnTarget = false;
            }

        } else {
            mHasTarget = false;
            mOnTarget = false;
        }
    }

    public synchronized void maybeUpdateGoalFromFieldRelativeGoal(double timestamp) {
        if (!mLatestAimingParameters.isEmpty()) {
            // Vision will control the turret.
            return;
        }
        if (mFieldRelativeTurretGoal == null) {
            return;
        }
        final double kLookaheadTime = 4.0;
        safetyReset();
    }

    public synchronized void followSetpoint() {

        if (SuperstructureConstants.kUseSmartdashboard) {
            mShooterSetpoint = getShootingSetpointRpm(0);
            mHoodSetpoint = getHoodSetpointAngle(0);
        }

        if (mWantsTuck || !mEnableIndexer) {
            mHood.setSetpointPositionPID(Constants.kHoodConstants.kMinUnitsLimit, 0);
        } else if (mWantsFendor) {
            mHood.setSetpointMotionMagic(39.5);
        } else if (mWantsTestSpit) {
            mHood.setSetpointMotionMagic(Constants.kHoodConstants.kMinUnitsLimit);
        } else {
            mHood.setSetpointMotionMagic(mHoodSetpoint);
        }

        Magazine.WantedAction indexerAction = Magazine.WantedAction.NONE;
        double real_trigger = 0.0;
        double real_shooter = 0.0;
        boolean real_popout = false;

        if (Intake.getInstance().getState() == Intake.State.INTAKING) {
            indexerAction = Magazine.WantedAction.ZOOM;
            real_trigger = -600.0;
        }

        if (mWantsSpinUp) {
            real_shooter = mShooterSetpoint;
            indexerAction = Magazine.WantedAction.ZOOM;
            real_trigger = -600.0;
        } else if (mWantsPreShot) {
            real_shooter = mShooterSetpoint;
            indexerAction = Magazine.WantedAction.HELLA_ZOOM;
            real_trigger = Constants.kTriggerRPM;
            real_popout = false;
        } else if (mWantsShoot) {
            real_shooter = mShooterSetpoint;

            if (mLatestAimingParameters.isPresent()) {
                if (mLatestAimingParameters.get().getRange() > 240.) {
                    indexerAction = Magazine.WantedAction.ZOOM;
                } else {
                    indexerAction = Magazine.WantedAction.ZOOM;
                }
            } else {
                indexerAction = Magazine.WantedAction.ZOOM;
            }
            real_trigger = Constants.kTriggerRPM;

            if (mGotSpunUp) {
                real_popout = true;
            }

            if (mShooter.spunUp() && mTrigger.spunUp()) {
                mGotSpunUp = true;
            }
        }

        
        if (mWantsUnjam) {
            indexerAction = Magazine.WantedAction.UNZOOM;
            real_popout = true;
            real_trigger = -5000;
        }

        if (mEnableIndexer) {
            mMagazine.setState(indexerAction);
        } else {
            mMagazine.setState(Magazine.WantedAction.UNZOOM);
        }

        mTrigger.setPopoutSolenoid(real_popout);
        mTrigger.setVelocity(real_trigger);
        if (Math.abs(real_shooter) < Util.kEpsilon) {
            mShooter.setOpenLoop(0);
        } else if (mWantsFendor) {
            mShooter.setVelocity(1500);
        } else if (mWantsTestSpit) {
            mShooter.setVelocity(1200);
        } else {
            mShooter.setVelocity(real_shooter);
        }

        if (mLatestAimingParameters.isPresent() && !mWantsHoodScan) {
            if (mManualZoom) {
                Limelight.getInstance().setPipeline(Limelight.kZoomedInPipeline);
            } else if (mLatestAimingParameters.get().getRange() > kZoomedInRange
                    && Limelight.getInstance().getPipeline() == Limelight.kDefaultPipeline) {
                Limelight.getInstance().setPipeline(Limelight.kZoomedInPipeline);
            } else if (mLatestAimingParameters.get().getRange() < kZoomedOutRange
                    && Limelight.getInstance().getPipeline() == Limelight.kZoomedInPipeline) {
                Limelight.getInstance().setPipeline(Limelight.kDefaultPipeline);
            }
        } else if (mManualZoom) {
            Limelight.getInstance().setPipeline(Limelight.kZoomedInPipeline);
        } else {
            Limelight.getInstance().setPipeline(Limelight.kDefaultPipeline);
        }

//        Limelight.getInstance().setPipeline(Limelight.kDefaultPipeline);

        // mTurret.setOpenLoop(0);
        // mHood.setOpenLoop(0);
        
        estim_popout = trigger_popout.update(real_popout, 0.2);
    }

    public synchronized Optional<AimingParameters> getLatestAimingParameters() {
        return mLatestAimingParameters;
    }

    public synchronized boolean isOnTarget() {
        return mOnTarget;
    }

    public synchronized void setWantUnjam(boolean unjam) {
        mWantsUnjam = unjam;
    }

    public synchronized void setWantAutoAim(Rotation2d field_to_turret_hint, boolean enforce_min_distance,
            double min_distance) {
        mFieldRelativeTurretGoal = field_to_turret_hint;
        mEnforceAutoAimMinDistance = enforce_min_distance;
        mAutoAimMinDistance = min_distance;
    }

    public synchronized void setWantAutoAim(Rotation2d field_to_turret_hint) {
        setWantAutoAim(field_to_turret_hint, false, 500);
    }

    public synchronized void setWantShoot() {
        mWantsSpinUp = false;
        mWantsShoot = !mWantsShoot;
        mGotSpunUp = false;
        mWantsPreShot = false;
    }

    public synchronized void setWantPreShot(boolean pre_shot) {
        mWantsSpinUp = false;
        mWantsShoot = false;
        mGotSpunUp = false;
        mWantsPreShot = pre_shot;
    }

    public synchronized void setWantSpinUp() {
        mWantsSpinUp = !mWantsSpinUp;
        mWantsShoot = false;
        mGotSpunUp = false;
        mWantsPreShot = false;
    }

    public synchronized void setManualZoom(boolean zoom) {
        mManualZoom = zoom;
    }

    public synchronized void setWantShoot(boolean shoot) {
        mWantsSpinUp = false;
        mWantsShoot = shoot;
        mGotSpunUp = false;
        mWantsPreShot = false;
    }

    public synchronized void setWantSpinUp(boolean spin_up) {
        mWantsSpinUp = spin_up;
        mWantsShoot = false;
        mWantsPreShot = false;
    }

    public synchronized void setWantTuck(boolean tuck) {
        mWantsTuck = tuck;
    }

    public synchronized void setWantFendor() {
        mWantsFendor = !mWantsFendor;
        if (mWantsFendor) {
            mRobotState.resetVision();
        }
    }

    public synchronized boolean getWantFendor() {
        return mWantsFendor;
    }

    public synchronized boolean getWantSpit() {
        return mWantsTestSpit;
    }

    public synchronized boolean getTucked() {
        return mWantsTuck;
    }

    public synchronized void setWantTestSpit() {
        mWantsTestSpit = !mWantsTestSpit;
    }

    public synchronized void setWantInnerTarget(boolean inner) {
        mUseInnerTarget = inner;
    }

    public synchronized void setAutoIndex(boolean auto_index) {
        mAutoIndex = auto_index;
    }
}
