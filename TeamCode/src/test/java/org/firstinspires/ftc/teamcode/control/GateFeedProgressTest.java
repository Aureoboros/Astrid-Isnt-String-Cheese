package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;
import static org.junit.Assert.*;

public final class GateFeedProgressTest {
  @Test public void stoppedGateDoesNotConsumeFeedPulseTime() {
    GateFeedProgress feed = new GateFeedProgress(400, 60);
    feed.start(10);
    assertEquals(0, feed.update(10,true,1),0);
    assertEquals(0, feed.update(10,false,1),0);
    assertEquals(.1,feed.update(50,true,.1),1e-9);
  }
  @Test public void reverseTravelMustBeRecoveredBeforeCreditingFeed() {
    GateFeedProgress feed = new GateFeedProgress(400,60);
    feed.start(0);
    assertEquals(.1,feed.update(40,true,.1),1e-9);
    assertEquals(0,feed.update(0,false,.15),0);
    assertEquals(0,feed.update(20,true,.1),0);
    assertEquals(.05,feed.update(60,true,.1),1e-9);
  }
  @Test public void reverseConfiguredGateUsesNegativeEncoderTravelAsFeed() {
    GateFeedProgress feed = new GateFeedProgress(400,-60);
    feed.start(100);
    assertEquals(.1,feed.update(60,true,.1),1e-9);
    assertEquals(0,feed.update(100,false,.15),0);
    assertEquals(0,feed.update(60,true,.1),0);
    assertEquals(.1,feed.update(20,true,.1),1e-9);
  }
  @Test public void restartingNextPulseClearsOnlyPreviousPulseProgress() {
    GateFeedProgress feed = new GateFeedProgress(400,60);
    feed.start(0);
    feed.update(-40,false,.1);
    feed.start(10);
    assertEquals(.1,feed.update(50,true,.1),1e-9);
  }
  @Test public void estimatedTimeCannotExceedElapsedTimeEvenWithEncoderJump() {
    GateFeedProgress feed = new GateFeedProgress(400,60);
    feed.start(0);
    assertEquals(.01,feed.update(100000,true,.01),1e-9);
  }
}
