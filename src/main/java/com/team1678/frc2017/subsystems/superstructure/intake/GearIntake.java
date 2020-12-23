package com.team1678.frc2017.subsystems.superstructure.intake;

import com.team1678.frc2017.Constants;
import com.team1678.frc2017.loops.ILooper;
import com.team1678.frc2017.loops.Loop;
import com.team1678.frc2017.subsystems.Subsystem;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearIntake extends Subsystem {

    private static GearIntake mInstance;

    public enum WantedAction {
        NONE, DROP, RISE, SCORE, OUTTAKE, START_DROPPING_BALLS, STOP_DROPPING_BALLS,
    }

    public enum State {
        IDLE, INTAKING, PICKING_UP, CARRYING, SCORING, OUTTAKING, MANUAL_CLIMBING, DROP_BALL_WITHOUT_GEAR, DROP_BALL_WITH_GEAR,
    }

    private State mState = State.IDLE;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private double mGearIntakeVoltage = 0;
    private boolean intake_down = false;
    private boolean current_spiked = false;
    private int pickup_timer = 0;

    public static double kGearIntakeVoltage = 12.0;
    public static double kPickupVoltage = 2.5;
    public static double kCarryVoltage = 1.5;
    public static double kScoreVoltage = -12.0;
    public static double kOuttakeVoltage = -4.0;
    public static int kPickupTicks = 300;
    public static double kCurrentThreshold = 60.0;

    private TimeDelayedBoolean mGearIntakeSolenoidTimer = new TimeDelayedBoolean();

    private final TalonFX mMaster;

    private Solenoid mDeploySolenoid;

    private boolean mRunningManual = false;

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private GearIntake(){
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kGearIntakeRollerId);
        mDeploySolenoid = Constants.makeSolenoidForId(Constants.kDeployGearSolenoidId);
    }

    public synchronized static GearIntake getInstance() {
        if (mInstance == null) {
            mInstance = new GearIntake();
        }
        return mInstance;
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Gear Intake Current", mPeriodicIO.current);
        SmartDashboard.putString("Gear Intake State", mState.toString());

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // startLogging();
                mState = GearIntake.State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (GearIntake.this) {
                    runStateMachine();

                }
            }
            @Override
            public void onStop(double timestamp) {
                mRunningManual = false;
                mState = State.IDLE;
                stop();
            }
        });
    }

    public double getVoltage() {
        return mPeriodicIO.demand;
    }

    public synchronized GearIntake.State getState() {
        return mState;
    }

    public void runStateMachine() {
        switch (mState) {
            case IDLE:
                mGearIntakeVoltage = 0.0;
                intake_down = false;
                break;
            case INTAKING:
                mGearIntakeVoltage = kGearIntakeVoltage;
                intake_down = true;
                if (mPeriodicIO.current > kCurrentThreshold) {
                    current_spiked = true;
                    mState = State.PICKING_UP;
                    pickup_timer = kPickupTicks;
                    mGearIntakeVoltage = kPickupVoltage;
                }
                break;
            case DROP_BALL_WITH_GEAR:
                mGearIntakeVoltage = 2.5;
                intake_down = true;
                break;
            case DROP_BALL_WITHOUT_GEAR:
                mGearIntakeVoltage = 0;
                intake_down = true;
                break;
            case PICKING_UP:
                mGearIntakeVoltage = kPickupVoltage;
                intake_down = false;
                if (--pickup_timer < 0) {
                    mState = State.CARRYING;
                }
                break;
            case CARRYING:
                mGearIntakeVoltage = kCarryVoltage;
                intake_down = false;
                break;
            case SCORING:
                mGearIntakeVoltage = kScoreVoltage;
                intake_down = false;
                break;
            case OUTTAKING:
                mGearIntakeVoltage = kOuttakeVoltage;
                intake_down = true;
        }
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case DROP:
                mState = State.INTAKING;
                break;
            case RISE:
                if (mState == State.INTAKING) {
                    mState = State.IDLE;
                }
                break;
            case SCORE:
                if (mState == State.CARRYING || mState == State.IDLE) {
                    mState = State.SCORING;
                }
                break;
            case NONE:
                if (mState == State.SCORING) {
                    mState = State.IDLE;
                }
                break;
            case OUTTAKE:
                mState = State.OUTTAKING;
                break;
            case START_DROPPING_BALLS:
                if (mState == State.PICKING_UP || mState == State.CARRYING) {
                    mState = State.DROP_BALL_WITH_GEAR;
                }
                if (mState == State.INTAKING || mState == State.IDLE) {
                    mState = State.DROP_BALL_WITHOUT_GEAR;
                }
                break;
            case STOP_DROPPING_BALLS:
                if (mState == State.DROP_BALL_WITHOUT_GEAR) {
                    mState = State.IDLE;
                }
                if (mState == State.DROP_BALL_WITH_GEAR) {
                    mState = State.CARRYING;
                }
                break;
        }
    }

    public synchronized void readPeriodicInputs() {
        mPeriodicIO.gear_intake_out = mGearIntakeSolenoidTimer.update(mPeriodicIO.deploy, 0.2);

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        mDeploySolenoid.set(mPeriodicIO.deploy);
    }

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;
        public double current;
        public boolean gear_intake_out;

        //OUTPUTS
        public double demand;
        public boolean deploy;

    }
}
