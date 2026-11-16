package org.firstinspires.ftc.teamcode.config;

import static org.junit.Assert.*;
import java.io.File;
import java.util.HashSet;
import java.util.Set;
import javax.xml.parsers.DocumentBuilderFactory;
import org.junit.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

/** The reference wiring must match the installed devices, not legacy odometry/servo names. */
public final class RobotHardwareXmlTest {
  private Document read() throws Exception {
    File file = new File("../.run/RobotConfig.xml");
    if (!file.isFile()) { file = new File(".run/RobotConfig.xml"); }
    return DocumentBuilderFactory.newInstance().newDocumentBuilder().parse(file);
  }

  private Element device(Document xml, String name) {
    NodeList all = xml.getElementsByTagName("*");
    Element found = null;
    for (int i = 0; i < all.getLength(); i++) {
      Element e = (Element) all.item(i);
      if (name.equals(e.getAttribute("name"))) {
        assertNull("Duplicate device name: " + name, found);
        found = e;
      }
    }
    assertNotNull("Missing device: " + name, found);
    return found;
  }

  private void port(Document xml, String name, String hub, String type, int port) {
    Element e = device(xml, name);
    assertEquals(type, e.getTagName());
    assertEquals(Integer.toString(port), e.getAttribute("port"));
    assertEquals(hub, ((Element) e.getParentNode()).getAttribute("name"));
  }

  @Test public void driveAndMechanismMotorsUseLatestHubPorts() throws Exception {
    Document xml = read();
    port(xml, RobotConfig.Hardware.FRONT_RIGHT, "Control Hub", "goBILDA5202SeriesMotor", 0);
    port(xml, RobotConfig.Hardware.BACK_RIGHT, "Control Hub", "goBILDA5202SeriesMotor", 1);
    port(xml, RobotConfig.Hardware.FRONT_LEFT, "Control Hub", "goBILDA5202SeriesMotor", 2);
    port(xml, RobotConfig.Hardware.BACK_LEFT, "Control Hub", "goBILDA5202SeriesMotor", 3);
    port(xml, RobotConfig.Hardware.INTAKE, "Expansion Hub 2", "goBILDA5202SeriesMotor", 0);
    port(xml, RobotConfig.Hardware.GATE_MOTOR, "Expansion Hub 2", "Motor", 1);
    port(xml, RobotConfig.Hardware.SHOOTER_1, "Expansion Hub 2", "GoBilda6000RpmMotor", 2);
  }

  @Test public void servoPinpointImuAndCameraMatchCurrentConfiguration() throws Exception {
    Document xml = read();
    port(xml, RobotConfig.Hardware.SERVO_MOTOR, "Control Hub", "ServoFullRange", 0);
    port(xml, RobotConfig.Hardware.PINPOINT, "Control Hub", "goBILDAPinpoint", 0);
    assertEquals("goBILDAPinpoint", com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class
        .getAnnotation(com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties.class).xmlTag());
    assertEquals(Integer.toString(RobotConfig.Hardware.PINPOINT_I2C_BUS),
        device(xml, RobotConfig.Hardware.PINPOINT).getAttribute("bus"));
    port(xml, RobotConfig.Hardware.IMU, "Control Hub", "ControlHubImuBHI260AP", 0);
    assertEquals("0", device(xml, RobotConfig.Hardware.IMU).getAttribute("bus"));
    Element camera = device(xml, RobotConfig.Hardware.LIMELIGHT);
    assertEquals("EthernetDevice", camera.getTagName());
    assertEquals("EthernetOverUsb:eth0:" + RobotConfig.Localization.LIMELIGHT_API_HOST,
        camera.getAttribute("serialNumber"));
  }

  @Test public void containsOnlyInstalledDevicesAndPreservesHubIdentifiers() throws Exception {
    Document xml = read();
    Set<String> actual = new HashSet<>();
    NodeList all = xml.getElementsByTagName("*");
    for (int i = 0; i < all.getLength(); i++) {
      Element e = (Element) all.item(i);
      if (e.getTagName().equals("Robot") || e.getTagName().equals("LynxUsbDevice")
          || e.getTagName().equals("LynxModule")) { continue; }
      assertTrue("Duplicate name", actual.add(e.getAttribute("name")));
    }
    assertEquals(new HashSet<>(java.util.Arrays.asList("frontRightMotor", "backRightMotor",
        "frontLeftMotor", "backLeftMotor", "intake", "gateMotor", "Shoot1",
        "servoMotor", "imu", "pinpoint", "limelight")), actual);
    Element portal = device(xml, "Control Hub Portal");
    assertEquals("(embedded)", portal.getAttribute("serialNumber"));
    assertEquals("173", portal.getAttribute("parentModuleAddress"));
    assertEquals("173", device(xml, "Control Hub").getAttribute("port"));
    assertEquals("2", device(xml, "Expansion Hub 2").getAttribute("port"));
  }
}
