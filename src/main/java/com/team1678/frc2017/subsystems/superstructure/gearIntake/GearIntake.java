package com.team1678.frc2017.subsystems.superstructure.gearIntake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2017.Constants;
import com.team1678.frc2017.loops.ILooper;
import com.team1678.frc2017.loops.Loop;

import com.team1678.frc2017.subsystems.Subsystem;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearIntake extends Subsystem {
    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }
}
