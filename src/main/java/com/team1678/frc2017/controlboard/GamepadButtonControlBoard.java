package com.team1678.frc2017.controlboard;

import com.team1678.frc2017.Constants;
import com.team1678.frc2017.controlboard.CustomXboxController.Button;
import com.team1678.frc2017.controlboard.CustomXboxController.Side;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Deadband;
import com.team254.lib.util.DelayedBoolean;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class GamepadButtonControlBoard {
    private final double kDeadband = 0.15;


    private int mDPadRight = -1;
    private int mDPadLeft = -1;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;

    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final CustomXboxController mController;

    private GamepadButtonControlBoard() {
        mController = new CustomXboxController(Constants.kButtonGamepadPort);
        reset();
    }

    public Rotation2d getJogTurret() {
        double jogX = mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.X);
        double jogY = mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.Y);
        
        Translation2d mag = new Translation2d(jogX, jogY);
        Rotation2d turret = mag.direction();

        if (Deadband.inDeadband(mag.norm(), 0.5)) {
            return null;
        }
        return turret;
    }

    public double getJogHood() {
        double jog = mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.Y);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    public void setRumble(boolean on) { //TODO: all 5 power cells indexed
        mController.setRumble(on);
    }

    public boolean getSpinUp() {
        return mController.getController().getAButtonReleased();
    }

    public boolean getTuck() {
        return mController.getButton(Button.X);
    }

    public boolean getFendorShot() {
        return false;
        //return mController.getController().getStickButtonReleased(Hand.kLeft);
    }

    public boolean getUntuck() {
        return mController.getButton(Button.START);
    }

    public boolean getTurretReset() {
        return mController.getController().getBackButtonReleased();
    }

    public boolean getTestSpit() {
        return mController.getController().getStickButtonReleased(Hand.kRight);
    }

    public boolean getRevolve() {
        return mController.getButton(CustomXboxController.Button.X);
    }

    public boolean getShoot() {
        return mController.getController().getYButtonReleased();
    }

    public boolean getPreShot() {
        return mController.getController().getBButtonReleased();
    }
    
    public boolean getIntake() {
        return mController.getTrigger(CustomXboxController.Side.RIGHT);
    } 
    
    public boolean getOuttake(){
        return mController.getTrigger(CustomXboxController.Side.LEFT);
    }

    public boolean getControlPanelRotation() {
        int pov = mController.getDPad();

        if (pov != mDPadRight) {
            mDPadRight = pov;
            return pov == 90;
        }
        return false;
    }

    public boolean getWantUnjam() {
        return mController.getController().getBumper(Hand.kLeft);
    }

    public boolean getManualZoom() {
        return mController.getController().getBumper(Hand.kRight);
    }

    public boolean getControlPanelPosition() {
        int pov = mController.getDPad();

        if (pov != mDPadLeft) {
            mDPadLeft = pov;
            return pov == 270;
        }
        return false;
    }

    public boolean climbMode() {
        return mController.getButton(CustomXboxController.Button.LB) && mController.getButton(CustomXboxController.Button.RB)  && 
        mController.getTrigger(CustomXboxController.Side.LEFT) &&  mController.getTrigger(CustomXboxController.Side.RIGHT);
    }

    public boolean getArmExtend() {
        return mController.getController().getAButtonReleased();
    }

    public boolean getStopClimb() {
        return mController.getController().getStickButtonReleased(Hand.kRight); 
    }

    public boolean getStopExtend() {
        System.out.println(mController.getController().getStickButton(Hand.kLeft));
        return mController.getController().getStickButtonReleased(Hand.kLeft);
    }

    public boolean getBuddyDeploy() {
        return mController.getController().getBackButtonReleased();
    }

    public boolean getArmHug() {
        return mController.getController().getBButtonReleased();
    }

    public boolean getManualArmExtend() {
        return //mController.getController().getStickButton(Hand.kLeft);
        mController.getButton(CustomXboxController.Button.L_JOYSTICK);
    }

    public boolean getWantHoodScan() {
        return mController.getButton(CustomXboxController.Button.L_JOYSTICK);
    }

    public boolean getManualArmRetract() {
        return //mController.getController().getStickButton(Hand.kRight);
        mController.getButton(CustomXboxController.Button.R_JOYSTICK);
    }

    public boolean getClimb() {
        return mController.getController().getYButtonReleased();
    }

    public boolean getBrake() {
        return false; // mController.getController().getYButtonReleased();
    }

    public boolean getWrangle() {
        return mController.getButton(CustomXboxController.Button.X);
    }

    public boolean getLeaveClimbMode() {
        return mController.getDPad() != -1;
    }

    public void reset() {
        mDPadValid = new DelayedBoolean(Timer.getFPGATimestamp(), kDPadDelay);
    }

    public boolean getAutoAim() {
        return mController.getTrigger(CustomXboxController.Side.LEFT);
    }

    public double getJoggingX() {
        double jog = mController.getJoystick(CustomXboxController.Side.RIGHT, CustomXboxController.Axis.X);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    public double getJoggingZ() {
        double jog = mController.getJoystick(CustomXboxController.Side.RIGHT, CustomXboxController.Axis.Y);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    // Intake
    public boolean getRunIntake() {
        return mController.getTrigger(Side.RIGHT);
    }

    public boolean getRetractIntake() {
        return mController.getTrigger(Side.LEFT);
    }
    

}