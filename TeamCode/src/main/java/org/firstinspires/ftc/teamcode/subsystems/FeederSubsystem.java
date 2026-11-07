package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem controlling the feeder ("Car Wash") and launch gate.
 *
 * Two continuous-rotation servos spin vertically to push stored balls
 * toward the launcher system.  A servo gate controls whether balls
 * can reach the launcher.
 */
public class FeederSubsystem {

    private final CRServo carWashLeft;
    private final CRServo carWashRight;
    private final Servo launchGate;

    /** Gate positions */
    public static final double GATE_OPEN = 0.0;
    public static final double GATE_CLOSED = 1.0;

    public FeederSubsystem(HardwareMap hardwareMap) {
        carWashLeft = hardwareMap.get(CRServo.class, "Car Wash Left");
        carWashRight = hardwareMap.get(CRServo.class, "Car Wash Right");
        launchGate = hardwareMap.get(Servo.class, "Launch Gate");
    }

    // ── Low-level control ────────────────────────────────────────────────

    /** Run the car wash fans to push balls toward the launcher. */
    public void runCarWash() {
        carWashLeft.setPower(-1.0);
        carWashRight.setPower(1.0);
    }

    /** Stop the car wash fans. */
    public void stopCarWash() {
        carWashLeft.setPower(0.0);
        carWashRight.setPower(0.0);
    }

    /** Open the launch gate (allow balls through). */
    public void openGate() {
        launchGate.setPosition(GATE_OPEN);
    }

    /** Close the launch gate (block balls). */
    public void closeGate() {
        launchGate.setPosition(GATE_CLOSED);
    }

    /**
     * Engage the full feeder: open gate + run car wash.
     */
    public void engage() {
        openGate();
        runCarWash();
    }

    /**
     * Disengage the full feeder: close gate + stop car wash.
     */
    public void disengage() {
        closeGate();
        stopCarWash();
    }

    // ── Commands ─────────────────────────────────────────────────────────

    /**
     * Command that engages the feeder (gate open, car wash running)
     * until interrupted.
     */
    public Command feedCommand() {
        return new CommandBuilder()
                .setStart(this::engage)
                .setEnd(endCondition -> disengage())
                .requiring(carWashLeft, carWashRight, launchGate);
    }

    /**
     * Command that immediately disengages the feeder.
     */
    public Command stopCommand() {
        return Commands.instant(this::disengage)
                .requiring(carWashLeft, carWashRight, launchGate);
    }

    /**
     * Teleop command: hold a button to feed, release to stop.
     *
     * @param feedButtonSupplier supplies the current state of the feed button
     * @return a continuous command suitable for scheduling during teleop
     */
    public Command teleopHoldToFeed(java.util.function.BooleanSupplier feedButtonSupplier) {
        return Commands.infinite(() -> {
            if (feedButtonSupplier.getAsBoolean()) {
                engage();
            } else {
                disengage();
            }
        })
                .setEnd(endCondition -> disengage())
                .requiring(carWashLeft, carWashRight, launchGate);
    }
}
