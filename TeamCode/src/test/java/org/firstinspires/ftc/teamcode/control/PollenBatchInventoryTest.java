package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import org.junit.Test;

public final class PollenBatchInventoryTest {
  @Test public void preloadsAndThreeRefillsSupplyFourSeparateBatches() {
    PollenBatchInventory inventory = new PollenBatchInventory();
    assertEquals(4, inventory.getRemaining()); assertFalse(inventory.canPickup());
    for (int batch = 0; batch < 4; batch++) {
      inventory.recordFed(4); assertTrue(inventory.canPickup());
      if (batch < 3) { inventory.recordPickup(); assertEquals(4, inventory.getRemaining()); }
    }
    assertEquals(16, inventory.getTotalFed()); assertEquals(0, inventory.getRemaining());
  }
  @Test(expected = IllegalStateException.class) public void pickupWithPreloadsRejected() {
    new PollenBatchInventory().recordPickup();
  }
  @Test(expected = IllegalStateException.class) public void pickupWithPartialBatchRejected() {
    PollenBatchInventory inventory = new PollenBatchInventory(); inventory.recordFed(3); inventory.recordPickup();
  }
  @Test(expected = IllegalStateException.class) public void overfeedingRejected() {
    new PollenBatchInventory().recordFed(5);
  }
  @Test public void interruptionMakesInventoryUnknownAndPreventsFurtherPickups() {
    PollenBatchInventory inventory = new PollenBatchInventory(); inventory.recordFed(4); inventory.invalidate();
    assertFalse(inventory.isKnown()); assertFalse(inventory.canPickup());
  }
}
