//
//  This command will set a triggering Event, and will end when that event has been processed.
//  Thus you can create a SerialCommandGroup full of triggeringEvents and they will execute one at a time.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tower.TowerEvent;
import frc.robot.subsystems.tower.Tower;

public class ScoreAndGotoAlgaeLevel extends Command {
  Tower tower;
  int    level;

  /** Creates a new TriggerEvent. */
  public ScoreAndGotoAlgaeLevel(Tower tower, int level) {
    this.tower = tower;
    this.level = level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tower.setDirectAlgaeLevel(level);
    tower.triggerEvent(TowerEvent.SCORE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
