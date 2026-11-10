package org.firstinspires.ftc.teamcode.localization;

/** Image-space aiming remains usable while robot-space measurements are gated during pan motion. */
public final class AprilTagAim {
  private final int id;
  private final double txDegrees;
  public AprilTagAim(int id, double txDegrees) { this.id = id; this.txDegrees = txDegrees; }
  public int getId() { return id; }
  public double getTxDegrees() { return txDegrees; }
}
