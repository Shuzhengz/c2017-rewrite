package com.team1678.frc2017.subsystems.superstructure.gearIntake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2017.Constants;
import com.team1678.frc2017.loops.ILooper;
import com.team1678.frc2017.loops.Loop;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import com.team1678.frc2017.subsystems.Subsystem;
import com.team1678.frc2017.subsystems.superstructure.Superstructure;
import com.team1678.frc2017.subsystems.superstructure.intake.Intake;
import com.team1678.frc2017.subsystems.superstructure.shooter.Shooter;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearIntake extends Subsystem {

    private static GearIntake mInstance;

    public enum WantedAction {
        NONE, DROP, RISE, SCORE, OUTTAKE, START_DROPPING_BALLS, STOP_DROPPING_BALLS,
    }

    public enum State {
        IDLE, INTAKING, PICKING_UP, CARRYING, SCORING, OUTTAKING, MANUAL_CLIMBING, DROP_BALL_WITHOUT_GEAR,
    }

    private State mState = State.IDLE;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private final double mGearIntakeVoltage = 0;
    private final boolean intake_down = false;
    private final boolean current_spiked = false;

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

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
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

    public synchronized GearIntake.State getState() {
        return mState;
    }

    public void runStateMachine() {
        switch (mState) {
            // Cases for each state and what the actuators should be at those states
        }
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            // Cases to switch
        }
    }

    public synchronized void readPeriodicInputs() {
        // Update anything you want to read from sensors or actuators, these are usually in your inputs under periodicIO

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {

        // Use .set on each of your actuators to whatever output you have been setting from periodicIO. This is also a good place to add limits to your code.

    }

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;

        //OUTPUTS
    }
}
