package com.team1678.frc2017.subsystems.superstructure.indexer;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2017.Constants;
import com.team1678.frc2017.loops.ILooper;
import com.team1678.frc2017.loops.Loop;
import com.team1678.frc2017.subsystems.Subsystem;
import com.team254.lib.drivers.TalonFXFactory;
import com.team1678.frc2017.planners.IndexerMotionPlanner;
import com.team1678.lib.util.HallCalibration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Magazine extends Subsystem {
    private static Magazine mInstance = null;
    private IndexerMotionPlanner mMotionPlanner;

    private static final double kZoomingVelocity = 80.;
    private static final double kPassiveIndexingVelocity = 80.0;
    private static final double kGearRatio = (60. / 16.) * (160. / 16.);
    private static final boolean[] kFullSlots = {true, true, true, true, true };
    private static final boolean[] kEmptySlots = {false, false, false, false, false };

    private boolean front_magazine_extended = false;
    private boolean side_magazine_extended = false;

    private double mIndexerStart = Timer.getFPGATimestamp();
    private static final double kAngleConversion = (2048.0 * kGearRatio) / 360.0;

    private static final double kJamCurrent = 150.0;
    private double mLastCurrentSpikeTime = 0.0;
    private static final double kCurrentIgnoreTime = 1.0;

    private static double upper_voltage = 0;
    private static double side_voltage =0;
    private static double lower_voltage = 0;

    public enum State {
        IDLE, FEEDING, ZOOMING, HELLA_ZOOMING, UNZOOMING,
    }

    public enum WantedAction {
        NONE, FEED, ZOOM, HELLA_ZOOM, UNZOOM
    }

    public enum UpperState {
        UPPER_IDLE, UPPER_FORWARD, UPPER_BACKWARD,
    }

    public enum SideState {
        SIDE_IDLE, SIDE_PULL_IN, SIDE_AGITATE,
    }

    public enum LowerState {
        LOWER_IDLE, LOWER_FORWARD, LOWER_BACKWARD,
    }

    private boolean mGeneratedGoal = false;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean[] mCleanSlots = { false, false, false, false, false };

    private final TalonFX mMaster;
    private final Solenoid mPopoutSolenoid;
    private State mState = State.IDLE;
    private double mInitialTime = 0;
    private boolean mStartCounting = false;
    private double mWaitTime = .1; // seconds
    private boolean mHasBeenZeroed = false;
    private boolean mBackwards = false;
    private int mSlotGoal;
    private DigitalInput mLimitSwitch = new DigitalInput(Constants.kIndexerLimitSwitch);
    private HallCalibration calibration = new HallCalibration(-37.0);
    private double mOffset = 0;
    private double mAngleGoal = 0;

    private Magazine() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kIndexerId);

        mMaster.config_kP(0, Constants.kIndexerKp, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, Constants.kIndexerKi, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, Constants.kIndexerKd, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(0, Constants.kIndexerKf, Constants.kLongCANTimeoutMs);
        mMaster.config_kP(1, Constants.kIndexerVelocityKp, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(1, Constants.kIndexerVelocityKi, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(1, Constants.kIndexerVelocityKd, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(1, Constants.kIndexerVelocityKf, Constants.kLongCANTimeoutMs);

        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mMaster.configMotionCruiseVelocity(Constants.kIndexerMaxVelocity);
        mMaster.configMotionAcceleration(Constants.kIndexerMaxAcceleration);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mMaster.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);
        mMaster.configClosedloopRamp(0.0);

        mMotionPlanner = new IndexerMotionPlanner();

        mPopoutSolenoid = Constants.makeSolenoidForId(Constants.kIndexerPopoutSolenoidID);
    }

    public synchronized State getState() {
        return mState;
    }

    public synchronized static Magazine getInstance() {
        if (mInstance == null) {
            mInstance = new Magazine();
        }
        return mInstance;
    }

    public boolean atHomingLocation() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putString("IndexerControlMode", mPeriodicIO.indexer_control_mode.name());
        SmartDashboard.putNumber("IndexerSetpoint", mPeriodicIO.conveyor_demand);
        SmartDashboard.putBoolean("Indexer Calibrated", calibration.isCalibrated());
        SmartDashboard.putNumber("IndexerVelocity", mPeriodicIO.indexer_velocity);
        SmartDashboard.putNumber("IndexerOffset", mOffset);

        SmartDashboard.putNumber("SlotNumberGoal", mSlotGoal);

        SmartDashboard.putString("CleanSlots", Arrays.toString(mCleanSlots));
        SmartDashboard.putBoolean("Snapped", mPeriodicIO.snapped);
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.indexer_control_mode = ControlMode.PercentOutput;
        mPeriodicIO.conveyor_demand = percentage;
    }

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    @Override
    public void zeroSensors() {
        mMaster.setSelectedSensorPosition(0, 0, 10);
        mHasBeenZeroed = true;
    }

    public synchronized boolean hasBeenZeroed() {
        return mHasBeenZeroed;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Magazine.this) {
                    runStateMachine();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
            }
        });
    }

    public synchronized void resetIfAtLimit() {
        if (mPeriodicIO.limit_switch) {
            zeroSensors();
        }
    }

    public synchronized void setBackwardsMode(boolean backwards) {
        mBackwards = backwards;
    }

    public synchronized boolean slotsFilled() {
        return Arrays.equals(mCleanSlots, kFullSlots);
    }

    public synchronized boolean slotsEmpty() {
        return Arrays.equals(mCleanSlots, kEmptySlots);
    }

    public void runStateMachine() {
        final double now = Timer.getFPGATimestamp();

        switch (mState) {
        case IDLE:
            mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
            mPeriodicIO.conveyor_demand = 0;
            break;
        case FEEDING:
            break;
        case ZOOMING:
            mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
            mPeriodicIO.conveyor_demand = mBackwards ? -kZoomingVelocity : kZoomingVelocity;
            break;
        case HELLA_ZOOMING:
            mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
            mPeriodicIO.conveyor_demand = (mBackwards ? -kZoomingVelocity : kZoomingVelocity) * 1.5;
            break;
        default:
            System.out.println("Fell through on Indexer states!");
        }
    }

    public double getIndexerVelocity() {
        if (mPeriodicIO.indexer_control_mode == ControlMode.Velocity) {
            return mPeriodicIO.conveyor_demand;
        } else {
            return 0;
        }
    }

    public void setState(WantedAction wanted_state) {
        final State prev_state = mState;
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case FEED:
                mState = State.FEEDING;
                break;
            case ZOOM:
                mState = State.ZOOMING;
                break;
            case HELLA_ZOOM:
                mState = State.HELLA_ZOOMING;
                break;
            case UNZOOM:
                mState = State.UNZOOMING;
                break;
        }

        if (mState != prev_state && mState == State.ZOOMING) {
            mMaster.configClosedloopRamp(0.2, 0);
        } else if (mState != prev_state) {
            mMaster.configClosedloopRamp(0.0, 0);
        }
    }

    public void setState(UpperState upper_state) {
        final State prev_state = mState;
        switch (upper_state) {
        case UPPER_IDLE:
            upper_voltage = 0;
            mState = State.IDLE;
            break;
        case UPPER_FORWARD:
            upper_voltage = 12;
            mState = State.ZOOMING;
            break;
        case UPPER_BACKWARD:
            upper_voltage = -4;
            mState = State.ZOOMING;
            break;
        }

        if (mState != prev_state && (mState == State.ZOOMING || mState == State.HELLA_ZOOMING)) {
            mMaster.configClosedloopRamp(0.2, 0);
        } else if (mState != prev_state) {
            mMaster.configClosedloopRamp(0.0, 0);
        }
    }

    public void setState(SideState side_state) {
        final State prev_state = mState;
        switch (side_state) {
            case SIDE_IDLE:
                upper_voltage = 0;
                mState = State.IDLE;
                break;
            case SIDE_AGITATE:
                upper_voltage = 6;
                mState = State.ZOOMING;
                break;
            case SIDE_PULL_IN:
                upper_voltage = -6;
                mState = State.ZOOMING;
                break;
        }

        if (mState != prev_state && (mState == State.ZOOMING || mState == State.HELLA_ZOOMING)) {
            mMaster.configClosedloopRamp(0.2, 0);
        } else if (mState != prev_state) {
            mMaster.configClosedloopRamp(0.0, 0);
        }
    }

    public void setState(LowerState lower_state) {
        final State prev_state = mState;
        switch (lower_state) {
            case LOWER_IDLE:
                upper_voltage = 0;
                mState = State.IDLE;
                break;
            case LOWER_FORWARD:
                upper_voltage = 12;
                mState = State.ZOOMING;
                break;
            case LOWER_BACKWARD:
                upper_voltage = -12;
                mState = State.ZOOMING;
                break;
        }

        if (mState != prev_state && (mState == State.ZOOMING || mState == State.HELLA_ZOOMING)) {
            mMaster.configClosedloopRamp(0.2, 0);
        } else if (mState != prev_state) {
            mMaster.configClosedloopRamp(0.0, 0);
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.limit_switch = !mLimitSwitch.get();
        mPeriodicIO.indexer_velocity = mMaster.getSelectedSensorVelocity(0) * 600. / 2048. / kGearRatio;
        mPeriodicIO.indexer_current = mMaster.getStatorCurrent();
        mPeriodicIO.side_mag_extended = side_magazine_extended;
        mPeriodicIO.front_mag_extended = front_magazine_extended;

        if (atHomingLocation() && !mHasBeenZeroed) {
            mMaster.setSelectedSensorPosition((int) Math.floor(mOffset * kAngleConversion));
            mMaster.overrideSoftLimitsEnable(true);
            System.out.println("Homed!!!");
            mHasBeenZeroed = true;
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.indexer_control_mode == ControlMode.Velocity) {
            if (mPeriodicIO.indexer_current > kJamCurrent && Timer.getFPGATimestamp() - mLastCurrentSpikeTime > kCurrentIgnoreTime) {
                mBackwards = !mBackwards;
                mLastCurrentSpikeTime = Timer.getFPGATimestamp();
            }
            mMaster.selectProfileSlot(1, 0);
            mMaster.set(mPeriodicIO.indexer_control_mode, (mPeriodicIO.conveyor_demand / 600.0) * kGearRatio * 2048.0);
        } else if (mPeriodicIO.indexer_control_mode == ControlMode.MotionMagic) {
            mMaster.selectProfileSlot(0, 0);
            mMaster.set(mPeriodicIO.indexer_control_mode, (mPeriodicIO.conveyor_demand / 360.0) * kGearRatio * 2048.0);
        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }


    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public boolean limit_switch;

        public double indexer_velocity;
        public double indexer_current;
        public boolean snapped;
        public boolean side_mag_extended;
        public boolean front_mag_extended;

        // OUTPUTS
        public ControlMode indexer_control_mode = ControlMode.PercentOutput;
        public double conveyor_demand;
    }
}
