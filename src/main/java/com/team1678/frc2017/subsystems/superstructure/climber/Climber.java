package com.team1678.frc2017.subsystems.superstructure.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team1678.frc2017.Constants;
import com.team1678.frc2017.loops.ILooper;
import com.team1678.frc2017.loops.Loop;

import com.team1678.frc2017.subsystems.Subsystem;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Climber extends Subsystem {
    private static Climber mInstance = null;

    private static final double kIdleVoltage = 0.0;
    private static final double kTopVoltage = -2.0;
    private static final double kClimbingVoltage = -12.0;
    private static final double kSpinUpVelocity = 0.42;
    private static final double kStartClimbingVelocity = 0.35;
    private static final double kFinalVelocity = 0.15;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    public enum WantedAction {
        NONE, SPIN_UP, CLIMB, MANUAL_CLIMB, BRAKE, STOP,
    }

    public enum State {
        IDLE, SPIN_UP ,APPROACHING ,MANUAL_CLIMBING,  CLIMBING , AT_TOP,
    }

    private State mState = State.IDLE;

    private final TalonFX mMaster;
    private final TalonFX mSlave;
    private double mHoldingPos = 0.0;
    private double mZeroPos;
    private boolean mPivoted = false;
    private boolean mExtended = false;
    private TimeDelayedBoolean brake_activation = new TimeDelayedBoolean();

    public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 40, 40, .2);
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private Climber() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kWinchMasterId);
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(true);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mMaster.configMotionAcceleration(40000, Constants.kLongCANTimeoutMs);
        mMaster.configMotionCruiseVelocity(20000, Constants.kLongCANTimeoutMs);
        mMaster.config_kP(0, 0.5);
        mMaster.config_kI(0, 0);
        mMaster.config_kD(0, 0);
        mMaster.config_kF(0, 0.05);

        mSlave = TalonFXFactory.createPermanentSlaveTalon(Constants.kWinchSlaveId, Constants.kWinchMasterId);
        mSlave.setInverted(false);
        mSlave.follow(mMaster);

        mMaster.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);

        mMaster.setNeutralMode(NeutralMode.Coast);
        mSlave.setNeutralMode(NeutralMode.Coast);

        mMaster.configStatorCurrentLimit(STATOR_CURRENT_LIMIT);
    }

    public synchronized static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public synchronized void setBrakeMode(boolean brake) {
        mMaster.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
        mSlave.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("ClimberState", mState.name());
        SmartDashboard.putNumber("ClimbVoltage", mPeriodicIO.demand);
        SmartDashboard.putNumber("ClimberPosition", mPeriodicIO.position);
        SmartDashboard.putNumber("ClimberVelocity", mPeriodicIO.velocity);

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    @Override
    public void zeroSensors() {
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
                synchronized (Climber.this) {
                    runStateMachine();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
            }
        });
    }

    public synchronized State getState() {
        return mState;
    }

    public void setBrake(boolean brake) {
        mPeriodicIO.brake_solenoid = brake;
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
    }

    public void setZeroPosition() {
        mZeroPos = mPeriodicIO.position;
    }

    public void runStateMachine() {
        final double now = Timer.getFPGATimestamp();
        switch (mState) {
            case IDLE:
                mPeriodicIO.demand = kIdleVoltage;
                break;
            case SPIN_UP:
                mPeriodicIO.demand = kClimbingVoltage;
                if (mPeriodicIO.velocity > kSpinUpVelocity) {
                    mState = State.APPROACHING;
                }
                break;
            case APPROACHING:
                mPeriodicIO.demand = kClimbingVoltage;
                if (mPeriodicIO.velocity < kStartClimbingVelocity) {
                    mState = State.CLIMBING;
                }
                break;
            case CLIMBING:
                mPeriodicIO.demand = kClimbingVoltage;
                if (mPeriodicIO.velocity < kFinalVelocity) {
                    mState = State.AT_TOP;
                }
                break;
            case MANUAL_CLIMBING:
                mPeriodicIO.demand = kClimbingVoltage;
                break;
            case AT_TOP:
                mPeriodicIO.demand = kTopVoltage;
                break;
            default:
                System.out.println("Fell through on Climber states!");
        }
    }

    public void setState(WantedAction wanted_state) {
        if (wanted_state == WantedAction.BRAKE && mState != State.AT_TOP) {
            mHoldingPos = mPeriodicIO.position;
        }
        switch (wanted_state) {
            case NONE:
                if (mState == State.MANUAL_CLIMBING) {
                    mState = State.IDLE;
                }
                break;
            case SPIN_UP:
                mState = State.SPIN_UP;
                break;
            case CLIMB:
                mState = State.CLIMBING;
                break;
            case MANUAL_CLIMB:
                mState = State.MANUAL_CLIMBING;
                break;
            case BRAKE:
                mState = State.AT_TOP;
                break;
            case STOP:
                mState = State.IDLE;
                break;
            default:
                System.out.println("No climber goal!");
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.position = mMaster.getSelectedSensorPosition(0);
        mPeriodicIO.velocity = mMaster.getSelectedSensorVelocity(0);

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
        mPeriodicIO.current = mMaster.getSupplyCurrent();
        mPeriodicIO.voltage = mMaster.getMotorOutputVoltage();
        //LogSend();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mState == State.SPIN_UP || mState == State.APPROACHING || mState == State.CLIMBING) {
            mMaster.set(ControlMode.MotionMagic, mPeriodicIO.demand);
        } else {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        }
    }

    public static class PeriodicIO {
        // INPUTS
        public double position;
        public double velocity;
        public boolean braked;
        public double current;
        public double voltage;

        // OUTPUTS
        public double demand;
        public boolean arm_solenoid;
        public boolean brake_solenoid;
    }
}
