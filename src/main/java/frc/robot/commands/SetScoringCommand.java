

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.ScoringSubsystem.ScoringStates;


public class SetScoringCommand extends Command {

  private final ScoringSubsystem subsystem;
  private final Timer timer;
  private final ScoringStates state;
  private final ScoringStates endState;
  private final double duration;


  public SetScoringCommand(ScoringSubsystem subsystem, ScoringStates state, ScoringStates endState, double duration) {
    this.subsystem = subsystem;
    this.state = state;
    this.endState = endState;
    this.duration = duration;
    timer = new Timer();
    addRequirements(subsystem);
  }

  public SetScoringCommand(ScoringSubsystem subsystem, ScoringStates state, double duration) {
    this(subsystem, state, ScoringStates.Idle, duration);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    subsystem.setState(state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    subsystem.setState(endState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }
}
