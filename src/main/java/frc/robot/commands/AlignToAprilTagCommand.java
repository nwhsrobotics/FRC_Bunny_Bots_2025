
//imports that we need
package frc.robot.commands; //command imports
import frc.robot.util.LimelightHelpers; //limelight library
import edu.wpi.first.math.MathUtil;//might need to use later
import edu.wpi.first.math.geometry.Pose2d; //might need to use later
import edu.wpi.first.wpilibj2.command.Command; //this is a child class to the parent class of Command
import frc.robot.subsystems.SwerveSubsystem;//we are connected to the swerve subsystem, as anything that limelight needs to do will be executed through the swerve
import frc.robot.Constants.DriveConstants;

/**
 * Vision alignment command that:
 *   1) Detects only AprilTags 5–8 (your COSMIC CONVERTERS)
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

    private final SwerveSubsystem swerve;//the swerve modules that we are using
    private final String limelightName;//name of our limelight, we will configure this later

    private State state = State.ACQUIRE;  // we are always going to start in the ACQUIRE state, more on that later
    private int lostTagCycles = 0;        // this kind of gives our limelight a grace period, saying oh i lost the april tag for a second, let me wait to see if i can regain it

    // Only align to these tag IDs (your COSMIC Converter tags)
    //WHAT ARE THE NUMBERS OF THE TAGS THAT WE WANT TO LOOK AT??
    private static final int[] VALID_TAGS = {};

    // ------------------------------------------------------------
    // Abort behavior: if we lose the tag for too long -> stop
    // ------------------------------------------------------------
    private static final int MAX_LOST_CYCLES = 12;
    //why do we want the max to be 12 cycles. this is the amount of time it takes for the robot to run its main loop. think of it like the robots heart beat



    // ------------------------------------------------------------
    // PID-like tuning values (simple proportional control)
    // ------------------------------------------------------------

    // Rotation (tx error correction)
    private static final double kRotKP = 0.03; //controls how fast we turn(WILL CHANGE)
    private static final double kRotMax = 0.45;//max amount of speed you can be turning at(WILL CHANGE)

    // Strafe (robot-relative Y error correction)
    private static final double kStrafeKP = 2.0;//controls how fast we move left or right
    private static final double kStrafeMax = 0.5;//max amount of speed you can be moving horizontally at

    // Forward driving (robot-relative X error correction)
    private static final double kDriveKP = 2.0;
    private static final double kDriveMax = 0.6;

    // Alignment tolerances, the amount of error that we can have to think, ok we are good enough
    private static final double kTxTolDeg = 1.2;      // amount we can be off by when we are rotating to move from rotating to strafing
    private static final double kYTolMeters = 0.03;   // amount we can be off by when we are strafing to move from strafing to driving
    private static final double kXTolMeters = 0.035;  // amount we can be off by when we are driving up to the cosmic converter to move from driving to done

    // Desired final distance from the tag (in meters) we are talking abouut the distance from the limelight to the april tag
    private final double desiredDistMeters;//we will change this value, that's why its not set yet



    /**
     * Constructor.
     *
     * @param swerve      Your SwerveSubsystem
     * @param limelight   Name of your Limelight (from LimelightConstants.llFront)
     * @param desiredDistMeters  Your final stand-off distance in meters
     */

    //THIS IS OUR CONSTRUCTOR, IT CONSTRUCTS THIS COMMAND
    //this is run one time to create the command object that we will be using in the code
    //this stores things like what swerve we are using, what limelight we are using, and what the desired final distance is
    public AlignToAprilTagCommand(
            SwerveSubsystem swerve,
            String limelight,
            double desiredDistMeters)
    {
        this.swerve = swerve;
        this.limelightName = limelight;
        this.desiredDistMeters = desiredDistMeters;
        //this line of code says this whole command will need the swerve subsystem
        addRequirements(swerve);
    }


    //THIS STARTS THE PROGRAM, AND WE ALWAYS START IN THE ACQUIRE STATE
    //right now, this initialization happens when the driver presses the button, we have to change that later to when the april tag is detected
    @Override
    public void initialize() {
        // Reset state machine
        state = State.ACQUIRE;//starts the robot in its required state
        lostTagCycles = 0;//we haven't lost any cycle yet
    }



    @Override
    public void execute() {

        // ------------------------------------------------------------
        // Check if we see any target at all
        // ------------------------------------------------------------

        

        // ------------------------------------------------------------
        // Run state machine
        // ------------------------------------------------------------
        
    }



    // =====================================================================================
    //  STATE 1 – ACQUIRE (Find a valid AprilTag 5–8)
    // =====================================================================================
    private void handleAcquire() {
        //if the tag is in view, switch the state to ROTATE
        //otherwise, stop the swerve modules and wait
    }



    // =====================================================================================
    //  STATE 2 – ROTATE (tx -> 0 degrees)
    // =====================================================================================
    private void handleRotate() {
        //get the orientation of the limelight
        //rotate command 
        //use the rotate commmand, ensure that the x and y coordinates at at the origin(0,0)
        //if we are turned enough, where we are well within the error rate, move on to next phase
    }



    // =====================================================================================
    //  STATE 3 – STRAFE (RobotSpace Y -> 0 meters)
    // =====================================================================================
    private void handleStrafe() {
        //get the position of our target(the april tag) based on where we are
        //lateral error
        //strafe to remove the y axis error 
        //only moving horizontally, no rotation or forward driving, reflect this in a code line
        //if we are within the tolerance, 
    }



    // =====================================================================================
    //  STATE 4 – DRIVE (RobotSpace X -> desiredDistMeters)
    // =====================================================================================
    private void handleDrive() {
        //get the position of our target(the april tag) based on where we are
        //get the forward distance to the tag
        //find the error, or the amount we need to move in order to get to our desired position


    }



    @Override
    public boolean isFinished() {
        //switch state to DONE
    }

    @Override
    public void end(boolean interrupted) {
        //stop our modules
    }



    // =====================================================================================
    //  HELPER: check if current tag is one of IDs 5–8
    // =====================================================================================
    private boolean isValidTagInView() {
        
    }
}

