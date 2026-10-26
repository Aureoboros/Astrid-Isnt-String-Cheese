package org.firstinspires.ftc.teamcode.control;

/** Expected inventory under calibrated timing, NOT a sensor measurement. Four-element limit: G407. */
public final class PollenBatchInventory {
  public static final int CAPACITY = 4;
  private int remaining = CAPACITY;
  private int totalFed;
  private boolean known = true;

  public void recordFed(int count) {
    if (!known || count < 0 || count > remaining) { throw new IllegalStateException("Cannot feed more than known expected inventory"); }
    remaining -= count;
    totalFed += count;
  }

  public void recordPickup() {
    if (!canPickup()) { throw new IllegalStateException("Pickup requires empty expected inventory (G407)"); }
    remaining = CAPACITY;
  }

  public void invalidate() { known = false; }
  public boolean isKnown() { return known; }
  public boolean canPickup() { return known && remaining == 0; }
  public int getRemaining() { return remaining; }
  public int getTotalFed() { return totalFed; }
}
