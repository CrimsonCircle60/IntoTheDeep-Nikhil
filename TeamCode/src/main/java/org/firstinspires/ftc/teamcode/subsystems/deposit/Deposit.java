package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;

//@Config
public class Deposit {
    public enum State{
        IDLE,
        READY_FOR_TRANSFER,
        TRANSFER_INTO_IDLE_TRANSFER,
        READY_FOR_DEPOSIT_PIXEL,
        READY_FOR_DEPOSIT_SAMPLE,
        DEPOSIT,
        TEST
    }

    public static final double PIXEL_DEPOSIT_HEIGHT = 30.0; // inches; TODO: need to find the actual target height for the pixel
    public static final double SAMPLE_DEPOSIT_HEIGHT = 16.5; // inches; TODO: need to find the actual target height for the sample
    public static final double PIXEL_DEPOSIT_ERROR_THRESH = 0.3; // inches

    public State state;

    private final Robot robot;
    public final VerticalSlides verticalSlides;
    public final Arm arm;
    public final Claw claw;

    // prepare for transfer positions
    public static double transferRad = -Math.toRadians(45), transferY = 7.0;

    // deposit positions
    public static double depositPixelRad = Math.toRadians(45), depositPixelY = 26.0;
    public static double depositSampleRad = Math.toRadians(45), depositSampleY = 10.0;

    public static int relicType = Globals.hasSamplePreload ? 2 : Globals.hasColorfulPixelPreload ? 1 : Globals.hasColorlessPixelPreload ? 0 : -1;

    // moving positions with a sample
    public static double sampleHoldRad = 0.0, holdY = 0.0, sampleHoldClawRad = -Math.PI / 2;
    public static double specimenGrabRad = 0.04, specimenGrabClawRad = 0.0, specimenConfirmRad = Math.toRadians(40), specimenConfirmClawRad = Math.toRadians(40);

    // sample basket positions
    public static double sampleLY = 16.75, sampleHY = 32.5, sampleRaiseRad = Math.toRadians(90), sampleDepositRad = 2.2, sampleDepositClawRad = -0.2;

    // outtake positions, drop behind robot
    public static double outtakeRad = Math.toRadians(180), outtakeY = 0.0, outtakeClawRad = 0.0;

    // grabbing positions, holdGrab -> off the wall, grabRetract --> moving with a specimen
    // specimen chamber positions
    public static double speciLSY = 18.4;
    public static double  speciHRad = 2.5, speciHClawRad = -1.5, speciHY = 18.6;


    private long currentTime = -1;
    private long specimenReleaseTime = -1;
    public static int sampleReleaseDuration = 300;
    public static int specimenReleaseDuration = 700;
    private long grabStartTime = -1;
    public static int transferBufferDuration = 200;

    private boolean high = true;

    private boolean upBuf = false;

    public enum HangMode {
        OUT,
        PULL,
        OFF
    }
    public HangMode hangMode = HangMode.OFF;
    public boolean holdSlides = false;

    public Deposit(Robot robot){
        this.robot = robot;

        verticalSlides = new VerticalSlides(this.robot);
        arm = new Arm(this.robot);
        claw = new Claw(this.robot);

        state = Globals.RUNMODE == RunMode.TELEOP ? State.READY_FOR_TRANSFER : Globals.RUNMODE == RunMode.TESTER ? State.TEST : State.IDLE; // only other option is AUTO in which it starts in IDLE
        // TODO: need to figure out how to log what state the robot was in during auto
        // currently assumes that the robot will be in READY_FOR_TRANSFER by the end of auto, which I could make happen
        // perhaps with a byte in Globals for the state represented by 0 to 4?
    }

    public void update(){
        currentTime = System.nanoTime();
        switch (state) {
            case IDLE:
                // idk what to do here, I figure the bot is put into TRANSFER_INTO_IDLE_TRANSFER from somewhere else cause IDLE is basically to prevent movement during the hot start of auto
                break;
            case READY_FOR_TRANSFER:
                readyForTransfer();
                if (relicType != -1) state = State.TRANSFER_INTO_IDLE_TRANSFER;
                break;
            case TRANSFER_INTO_IDLE_TRANSFER:
                transferIntoIdleTransfer();
                state = relicType == 2 ? State.READY_FOR_DEPOSIT_SAMPLE : State.READY_FOR_DEPOSIT_PIXEL;
                break;
            case READY_FOR_DEPOSIT_PIXEL:
                readyForDepositPixel();
                if (inPositionForDeposit()) state = State.DEPOSIT;
                break;
            case READY_FOR_DEPOSIT_SAMPLE:
                readyForDepositSample();
                if (inPositionForDeposit()) state = State.DEPOSIT;
                break;
            case DEPOSIT:
                deposit();
                state = State.READY_FOR_TRANSFER;
                break;
            case TEST:
                break;
        }

        if (holdSlides) {
            verticalSlides.setTargetLength(targetY);
        }

        verticalSlides.update();

        if (hangMode == HangMode.PULL) {
            verticalSlides.setTargetPowerFORCED(-0.9);
            targetY = verticalSlides.getLength() - 0.5;
        } else if (hangMode == HangMode.OUT) {
            verticalSlides.setTargetPowerFORCED(0.7);
            targetY = verticalSlides.getLength() + 0.5;
        }
        if (hangMode != HangMode.OFF) holdSlides = true;

        TelemetryUtil.packet.put("Deposit.state", this.state);
        LogUtil.depositState.set(this.state.toString());
        TelemetryUtil.packet.put("Deposit.targetY", this.targetY);
        TelemetryUtil.packet.put("Deposit: Arm inPosition", arm.inPosition());
        TelemetryUtil.packet.put("Deposit: Slides inPosition", verticalSlides.inPosition(0.5));
        TelemetryUtil.packet.put("Deposit: Arm inPosition", arm.inPosition());
        TelemetryUtil.packet.put("Deposit: Claw inPosition", claw.inPosition());
        TelemetryUtil.packet.put("Deposit: Hanging", hangMode);

        hangMode = HangMode.OFF;
    }

    public void moveToWithRad(double armTargetRad, double targetY){
        arm.setArmRotation(armTargetRad, 1.0);
        verticalSlides.setTargetLength(targetY);
    }

    public void readyForTransfer(){
        // close the claw whilst the claw is out of range of internal components, then descend for transfer and open claw when in position
        claw.close();
        verticalSlides.setTargetLength(transferY);
        arm.setArmRotation(transferRad, 1.0);
        robot.waitWhile(claw::inPosition);
        robot.waitWhile(arm::inPosition);
        robot.waitWhile(verticalSlides::inPosition);
    }

    public void transferIntoIdleTransfer() {
        // close claw (transfer) and wait until claw is closed
        claw.close();
        robot.waitWhile(claw::inPosition);
    }

    public void readyForDepositPixel() {
        // raise & rotate to position determined by constant IK & no other deposit movement until readied

    }

    public void readyForDepositSample() {
        // raise & rotate to position determined by constant IK & no other deposit movement until readied

    }

    public void deposit() {
        // open the claw, wait till claw opened and an additional 30 ms before transitioning to readyForTransfer
        claw.open();
        robot.waitFor(30);
    }

    public int getRelicType() {
        // TODO: use Intake.java to setRelicType when it intakes stuff so it isn't called constantly
        // this deals with errors like seeing nothing in the intake because the relic has been transferred
        /*
        -1 -> nothing
        0  -> colorless pixel
        1  -> colorful pixel
        2  -> sample
         */
        return -1; // for now
    }

    public boolean inPositionForDeposit() {
        // inPosition should incorporate Drivetrain stuff to make sure robot is in position for deposit
        if (state == State.READY_FOR_DEPOSIT_PIXEL) {
            if (relicType == -1) { state = State.READY_FOR_TRANSFER; return false; }
            if (relicType == 2) {state = State.READY_FOR_DEPOSIT_SAMPLE; return false; }

            // verify that the claw is rotated appropriately for proper deposit
            if (!claw.inPosition()) return false;

            // verify that the target height has been achieved first (this is IK stuff)
            if (Math.abs(PIXEL_DEPOSIT_HEIGHT - (VerticalSlides.MIN_HEIGHT + verticalSlides.getLength() + arm.getArmHeight() + Claw.PIXEL_VERTICAL_CENTER_DISTANCE)) < PIXEL_DEPOSIT_ERROR_THRESH) {
                switch (relicType) { // then verify drivetrain stuff
                    case 0:
                        // TODO: need to figure out vision & odometry for this one
                        break;
                    case 1:

                        break;
                }
            } else return false;
        } else if (state == State.READY_FOR_DEPOSIT_SAMPLE) {
            // verify that the target height has been achieved first (this is IK stuff)
            if (SAMPLE_DEPOSIT_HEIGHT - (VerticalSlides.MIN_HEIGHT + verticalSlides.getLength() + arm.getArmHeight()) > 0) return false;
            // then verify drivetrain stuff
            // TODO: need to figure out coordinate system & where its centered
        }

        return false; // if nothing works
    }

    private double targetY = speciHY;
    public void setDepositHeight(double target){
        this.targetY = Utils.minMaxClip(target, 0.0, VerticalSlides.maxSlidesHeight);
    }

    // Tells the vertical slides preemptively to go up as soon as it is in a state to do so
    public void bufferUpwards() {
        upBuf = true;
    }

    public void setDepositHeightLowSample(){
        targetY = sampleLY;
        high = false;
    }

    public void setDepositHeightHighSample(){
        targetY = sampleHY;
        high = true;
    }

    public void setDepositLowSpeci(){
        targetY = speciLSY;
        high = false;
    }

    public void setDepositHighSpeci(){
        targetY = speciHY;
        high = true;
    }

    public double getDepositHeight(){
        return targetY;
    }

    public static double heightThresh = 12.0;
    public boolean safeToMove(){
        return verticalSlides.getLength() <= heightThresh;
    }

    public boolean isSpecimenDepositDone() {
        return state == State.IDLE;
    }


    public boolean isRetractDone(){
        return state == State.IDLE;
    }
}
