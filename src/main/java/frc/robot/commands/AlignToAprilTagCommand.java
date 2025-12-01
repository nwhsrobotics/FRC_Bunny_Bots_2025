package frc.robot.commands;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;

/**
 * Vision alignment command that:
 *   1) Detects only AprilTags 5–8 (your COSMIC Converters)
 *   2) Rotates robot to face the tag head-on (tx = 0)
 *   3) Horizontally adjusts left/right until centered (target-space Y = 0)
 *   4) Drives forward to a specified distance (target-space X = desiredDist)
 *
 * 
 */
public class AlignToAprilTagCommand extends Command {

    // ---------------------------------------------------------------
    // The alignment process uses a basic finite-state machine:
    // ---------------------------------------------------------------
    private enum State {
        ACQUIRE,    // Look for a valid tag (IDs 5–8)
        ROTATE,     // Rotate until Limelight tx ≈ 0 (face the tag)
        STRAFE,     // Strafe left/right until Y offset ≈ 0 (centered)
        DRIVE,      // Drive forward to desired X distance
        DONE        // Stop and finish
    }

    private final SwerveSubsystem swerve;
    private final String limelightName;

    private State state = State.ACQUIRE;  // start in ACQUIRE state
    private int lostTagCycles = 0;        // counts how many loops we've lost vision

    // Only align to these tag IDs (your COSMIC Converter tags)
    private static final int[] VALID_TAGS = {5, 6, 7, 8};

    // ------------------------------------------------------------
    // Abort behavior: if we lose the tag for too long -> stop
    // ------------------------------------------------------------
    private static final int MAX_LOST_CYCLES = 12;



    // ------------------------------------------------------------
    // PID-like tuning values (simple proportional control)
    // ------------------------------------------------------------

    // Rotation (tx error correction)
    private static final double kRotKP = 0.03; // proportional gain for rotation
    private static final double kRotMax = 0.45;

    // Strafe (robot-relative Y error correction)
    private static final double kStrafeKP = 2.0;
    private static final double kStrafeMax = 0.5;

    // Forward driving (robot-relative X error correction)
    private static final double kDriveKP = 2.0;
    private static final double kDriveMax = 0.6;

    // Alignment tolerances
    private static final double kTxTolDeg = 1.2;      // degrees
    private static final double kYTolMeters = 0.03;   // meters (~3 cm)
    private static final double kXTolMeters = 0.035;  // meters (~3.5 cm)

    // Desired final distance from the tag (in meters)
    private final double desiredDistMeters;



    /**
     * Constructor.
     *
     * @param swerve      Your SwerveSubsystem
     * @param limelight   Name of your Limelight (from LimelightConstants.llFront)
     * @param desiredDistMeters  Your final stand-off distance in meters
     */
    public AlignToAprilTagCommand(
            SwerveSubsystem swerve,
            String limelight,
            double desiredDistMeters)
    {
        this.swerve = swerve;
        this.limelightName = limelight;
        this.desiredDistMeters = desiredDistMeters;

        addRequirements(swerve);
    }



    @Override
    public void initialize() {
        // Reset state machine
        state = State.ACQUIRE;
        lostTagCycles = 0;
    }



    @Override
    public void execute() {

        // ------------------------------------------------------------
        // Check if we see any target at all
        // ------------------------------------------------------------
        boolean hasTarget = LimelightHelpers.getTV(limelightName);

        if (!hasTarget) {
            // Increment lost counter; if too long, abort alignment
            lostTagCycles++;
            if (lostTagCycles > MAX_LOST_CYCLES) {
                state = State.DONE;
            }
        } else {
            lostTagCycles = 0;
        }

        // ------------------------------------------------------------
        // Run state machine
        // ------------------------------------------------------------
        switch (state) {

            case ACQUIRE -> handleAcquire();
            case ROTATE  -> handleRotate();
            case STRAFE  -> handleStrafe();
            case DRIVE   -> handleDrive();
            case DONE    -> swerve.stopModules();
        }
    }



    // =====================================================================================
    //  STATE 1 – ACQUIRE (Find a valid AprilTag 5–8)
    // =====================================================================================
    private void handleAcquire() {
        if (isValidTagInView()) {
            state = State.ROTATE;
        } else {
            swerve.stopModules();  // sit still until tag is found
        }
    }



    // =====================================================================================
    //  STATE 2 – ROTATE (tx -> 0 degrees)
    // =====================================================================================
    private void handleRotate() {
        double tx = LimelightHelpers.getTX(limelightName);

        // Simple proportional rotation: error * gain
        double rotCmd = MathUtil.clamp(kRotKP * tx, -kRotMax, kRotMax);

        // Rotate only, no translation
        swerve.drive(0.0, 0.0, rotCmd, true, true);

        // If we're facing the tag, move to next phase
        if (Math.abs(tx) < kTxTolDeg) {
            state = State.STRAFE;
        }
    }



    // =====================================================================================
    //  STATE 3 – STRAFE (RobotSpace Y -> 0 meters)
    // =====================================================================================
    private void handleStrafe() {

        if (!LimelightHelpers.getTV(limelightName)) return;

        // RobotSpace pose: target relative to robot
        double[] pose = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
        if (pose == null) return;

        double y = pose[1];  // lateral error; + means target is LEFT of robot

        // Strafe to remove Y error
        double strafeCmd = MathUtil.clamp(-kStrafeKP * y, -kStrafeMax, kStrafeMax);

        // Strafe only, no rotation or forward movement
        swerve.drive(0.0, strafeCmd, 0.0, true, true);

        // If centered, move to final phase
        if (Math.abs(y) < kYTolMeters) {
            state = State.DRIVE;
        }
    }



    // =====================================================================================
    //  STATE 4 – DRIVE (RobotSpace X -> desiredDistMeters)
    // =====================================================================================
    private void handleDrive() {

        if (!LimelightHelpers.getTV(limelightName)) return;

        // RobotSpace pose: target relative to robot
        double[] pose = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
        if (pose == null) return;

        double x = pose[0]; // forward distance to tag

        double error = x - desiredDistMeters;

        double fwdCmd = MathUtil.clamp(kDriveKP * error, -kDriveMax, kDriveMax);

        // Drive forward only; no strafe or rotation
        swerve.drive(fwdCmd, 0.0, 0.0, true, true);

        // If within tolerance, we’re fully aligned
        if (Math.abs(error) < kXTolMeters) {
            state = State.DONE;
            swerve.stopModules();
        }
    }



    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }



    // =====================================================================================
    //  HELPER: check if current tag is one of IDs 5–8
    // ===== ================================================================================
    private boolean isValidTagInView() {
        if (!LimelightHelpers.getTV(limelightName)) return false;

        int fid = (int) LimelightHelpers.getFiducialID(limelightName);
        for (int t : VALID_TAGS) {
            if (fid == t) return true;
        }
        return false;
    }
}