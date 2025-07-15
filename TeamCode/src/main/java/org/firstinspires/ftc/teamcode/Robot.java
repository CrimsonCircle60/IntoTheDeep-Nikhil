package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.vision.LLBlockDetectionPostProcessor;
import com.acmerobotics.dashboard.canvas.Canvas;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class Robot {
    public final HardwareMap hardwareMap;
    public final HardwareQueue hardwareQueue;
    public final Sensors sensors;
    public final Drivetrain drivetrain;
    public final nClawIntake nclawIntake;
    public final nDeposit ndeposit;
    public final Hang hang;
    public final LLBlockDetectionPostProcessor vision;

    private BooleanSupplier stopChecker = null;

    public ArrayList<Consumer<Canvas>> canvasDrawTasks;

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = new HardwareQueue();

        sensors = new Sensors(this);
        nclawIntake = new nClawIntake(this);
        drivetrain = new Drivetrain(this);
        ndeposit = new nDeposit(this);
        hang = new Hang(this);
        vision = new LLBlockDetectionPostProcessor(this);
        vision.start();

        canvasDrawTasks = new ArrayList<>();

        TelemetryUtil.setup();
        LogUtil.reset();
    }

    public void update() {
        START_LOOP();

        if (this.stopChecker != null && this.stopChecker.getAsBoolean()) {
            /*Log.i("STOP", Globals.RUNMODE.toString());
            if (Globals.RUNMODE == RunMode.AUTO) {
                this.drivetrain.setBrakePad(false);
                this.drivetrain.brakePad.setForceUpdate();
                this.ndeposit.arm.armRotation.setTargetAngle(Math.toRadians(-135));
                this.ndeposit.arm.armRotation.setForceUpdate();
                this.ndeposit.arm.clawRotation.setTargetAngle(0);
                this.ndeposit.arm.clawRotation.setForceUpdate();
                this.ndeposit.arm.clawOpen();
                this.ndeposit.arm.claw.setForceUpdate();
            }*/
            this.hardwareQueue.update();
            return;
        }

        this.sensors.update();

        drivetrain.update();
        nclawIntake.update();
        ndeposit.update();
        hang.update();
        vision.update();

        hardwareQueue.update();

        this.updateTelemetry();
    }

    /**
     * Sets the condition that should stop waiting (waitWhile)
     * @param func the function to check (return true to stop)
     */
    public void setStopChecker(BooleanSupplier func) { this.stopChecker = func; }

    public boolean isStopRequested() { return this.stopChecker.getAsBoolean(); }

    /**
     * Waits while a condition is true
     * @param func the function to check
     */
    public void waitWhile(BooleanSupplier func) {
        do {
            update();
        } while (!this.stopChecker.getAsBoolean() && func.getAsBoolean());
    }

    /**
     * Waits for a duration
     * @param duration the duration in milliseconds
     */
    public void waitFor(long duration) {
        long start = System.currentTimeMillis();
        do {
            update();
        } while (!this.stopChecker.getAsBoolean() && System.currentTimeMillis() - start < duration);
    }

    public void followSpline(Spline spline, BooleanSupplier func) {
        long start = System.currentTimeMillis();
        drivetrain.setPath(spline);
        drivetrain.state = Drivetrain.State.FOLLOW_SPLINE;
        drivetrain.setMaxPower(1);
        update();

        do {
            update();
        } while (!this.stopChecker.getAsBoolean() && (func == null || func.getAsBoolean()) && System.currentTimeMillis() - start <= 10000 && drivetrain.isBusy());
    }

    private void updateTelemetry() {

        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        for (Consumer<Canvas> task : canvasDrawTasks) task.accept(canvas);

        TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
        //LogUtil.loopTime.set(GET_LOOP_TIME());
        TelemetryUtil.sendTelemetry();
        LogUtil.send();
    }
}
