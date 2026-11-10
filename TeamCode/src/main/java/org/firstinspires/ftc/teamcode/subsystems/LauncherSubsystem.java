package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem controlling the ball launcher.
 *
 * The launcher motor spins a flywheel; its speed determines
 * how high/far the ball is thrown.  Speed is controlled in RPM
 * using the motor's built-in velocity PID.
 */
public class LauncherSubsystem {

    /** Encoder ticks per revolution for the goBILDA 5202 with 3.7:1 gearbox. */
    public static final double TICKS_PER_REV = 28.0 * 3.7;

    /** Default RPM when first starting the launcher. */
    public static final double START_RPM = 1000; //1450

    /** RPM adjustment step per button press. */
    public static final double RPM_STEP = 50.0;

    /** Maximum allowed RPM. */
    public static final double MAX_RPM = 6000.0;

    private final DcMotorEx launcherMotor;
    private double targetRpm = 0.0;

    public LauncherSubsystem(HardwareMap hardwareMap) {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "LauncherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ── Low-level control ────────────────────────────────────────────────

    /** Set the target RPM and apply it to the motor. */
    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0.0, Math.min(rpm, MAX_RPM));
        applyVelocity();
    }

    /** Increment RPM by one step. If stopped, starts at START_RPM. */
    public void incrementRpm() {
        if (targetRpm == 0.0) {
            targetRpm = START_RPM;
        } else {
            targetRpm = Math.min(targetRpm + RPM_STEP, MAX_RPM);
        }
        applyVelocity();
    }

    /** Decrement RPM by one step (minimum 0). */
    public void decrementRpm() {
        targetRpm = Math.max(targetRpm - RPM_STEP, 0.0);
        applyVelocity();
    }

    /** Reset launcher to default starting RPM. */
    public void resetRpm() {
        targetRpm = START_RPM;
        applyVelocity();
    }

    /** Stop the launcher motor. */
    public void stop() {
        targetRpm = 0.0;
        launcherMotor.setVelocity(0);
    }

    /** @return current target RPM */
    public double getTargetRpm() {
        return targetRpm;
    }

    /** @return actual RPM as reported by the encoder */
    public double getActualRpm() {
        return (launcherMotor.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    /** @return actual velocity in ticks/sec */
    public double getActualTicksPerSec() {
        return launcherMotor.getVelocity();
    }

    private void applyVelocity() {
        double ticksPerSec = (targetRpm * TICKS_PER_REV) / 60.0;
        launcherMotor.setVelocity(ticksPerSec);
    }

    // ── Commands ─────────────────────────────────────────────────────────

    /**
     * Command that runs the launcher at a fixed RPM until interrupted.
     *
     * @param rpm the target RPM
     */
    public Command runAtRpm(double rpm) {
        return new CommandBuilder()
                .setStart(() -> setTargetRpm(rpm))
                .setEnd(endCondition -> stop())

                .requiring(launcherMotor);
    }

    /**
     * Command that immediately stops the launcher.
     */
    public Command stopCommand() {
        return Commands.instant(this::stop)
                .requiring(launcherMotor);
    }

    /**
     * Teleop command for D-pad RPM control.
     *
     * <ul>
     *   <li>D-Pad Up:    start at 1450 RPM when stopped; +50 RPM when running</li>
     *   <li>D-Pad Down:  −50 RPM</li>
     *   <li>D-Pad Left:  reset to 1450 RPM</li>
     *   <li>D-Pad Right: stop (0 RPM)</li>
     * </ul>
     *
     * @param upSupplier    D-pad up button state
     * @param downSupplier  D-pad down button state
     * @param leftSupplier  D-pad left button state
     * @param rightSupplier D-pad right button state
     * @return a continuous command suitable for scheduling during teleop
     */
    public Command teleopDpadControl(java.util.function.BooleanSupplier upSupplier,
                                     java.util.function.BooleanSupplier downSupplier,
                                     java.util.function.BooleanSupplier leftSupplier,
                                     java.util.function.BooleanSupplier rightSupplier) {
        return new CommandBuilder()
                .setStart(() -> targetRpm = 0.0)
                .setExecute(new Runnable() {
                    private boolean prevUp = false;
                    private boolean prevDown = false;
                    private boolean prevLeft = false;
                    private boolean prevRight = false;

                    @Override
                    public void run() {
                        boolean up = upSupplier.getAsBoolean();
                        boolean down = downSupplier.getAsBoolean();
                        boolean left = leftSupplier.getAsBoolean();
                        boolean right = rightSupplier.getAsBoolean();

                        if (right && !prevRight) {
                            targetRpm = 0.0;
                        } else if (left && !prevLeft) {
                            targetRpm = START_RPM;
                        } else if (up && !prevUp) {
                            if (targetRpm == 0.0) {
                                targetRpm = START_RPM;
                            } else {
                                targetRpm = Math.min(targetRpm + RPM_STEP, MAX_RPM);
                            }
                        } else if (down && !prevDown) {
                            targetRpm = Math.max(targetRpm - RPM_STEP, 0.0);
                        }

                        prevUp = up;
                        prevDown = down;
                        prevLeft = left;
                        prevRight = right;

                        applyVelocity();
                    }
                })
                .setEnd(endCondition -> stop())
                .requiring(launcherMotor);
    }
}
