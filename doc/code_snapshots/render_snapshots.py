"""Render exact current Java excerpts into PNGs. Requires Pillow and Pygments."""
from pathlib import Path
import textwrap
from PIL import Image, ImageDraw, ImageFont
from pygments import lex
from pygments.lexers import JavaLexer
from pygments.styles import get_style_by_name

ROOT = Path(__file__).resolve().parents[2]
JAVA = ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
OUT = Path(__file__).resolve().parent
FONT = "/System/Library/Fonts/Menlo.ttc"
W, PAD, ROW = 2280, 70, 36
CODE_FONT = ImageFont.truetype(FONT, 25)
LABEL_FONT = ImageFont.truetype(FONT, 23)
TITLE_FONT = ImageFont.truetype(FONT, 46)
SUB_FONT = ImageFont.truetype(FONT, 25)
STYLE = get_style_by_name("monokai")


def excerpt(path, start, end=None, count=None):
    lines = (JAVA / path).read_text().splitlines()
    first = next(i for i, line in enumerate(lines) if start in line)
    if count is not None:
        stop = first + count
    else:
        stop = next(i for i in range(first, len(lines)) if end in lines[i]) + 1
    return path, first + 1, lines[first:stop]


def render(name, title, description, sections, note):
    sublines = textwrap.wrap(description, 135)
    notelines = textwrap.wrap(note, 135)
    top = 175 + 38 * len(sublines)
    heights = [118 + sum(len(block[2]) * ROW + 61 for block in blocks) for _, blocks in sections]
    height = top + sum(heights) + 28 * len(sections) + 85 + 38 * len(notelines)
    canvas = Image.new("RGB", (W, height), "#0b1220")
    draw = ImageDraw.Draw(canvas)
    draw.text((PAD, 43), title, font=TITLE_FONT, fill="#f1f5f9")
    draw.text((PAD, 111), "FTC / ASTRID   |   CURRENT SOURCE EXCERPTS", font=LABEL_FONT, fill="#38bdf8")
    for i, line in enumerate(sublines):
        draw.text((PAD, 155 + i * 38), line, font=SUB_FONT, fill="#cbd5e1")
    y = top
    for (heading, blocks), card_height in zip(sections, heights):
        draw.rounded_rectangle((PAD, y, W - PAD, y + card_height), radius=18, fill="#111c2e", outline="#29384f", width=2)
        draw.text((PAD + 30, y + 24), heading, font=SUB_FONT, fill="#67e8f9")
        cy = y + 76
        for path, first, lines in blocks:
            source = f"{path}  /  lines {first}-{first + len(lines) - 1}"
            draw.text((PAD + 30, cy), source, font=LABEL_FONT, fill="#94a3b8")
            cy += 40
            source_text = textwrap.dedent("\n".join(lines))
            x = PAD + 125
            line_y = cy
            tokens = lex(source_text, JavaLexer())
            if source_text.lstrip().startswith("*"):
                from pygments.token import Comment
                tokens = [(Comment.Multiline, source_text)]
            for token, value in tokens:
                color = STYLE.style_for_token(token).get("color") or "e2e8f0"
                parts = value.split("\n")
                for j, part in enumerate(parts):
                    if part:
                        if x + draw.textlength(part, font=CODE_FONT) > W - PAD - 26:
                            raise ValueError(f"Code overflow in {path}; select shorter excerpts")
                        draw.text((x, line_y), part, font=CODE_FONT, fill="#" + color)
                        x += draw.textlength(part, font=CODE_FONT)
                    if j < len(parts) - 1:
                        x = PAD + 125
                        line_y += ROW
            for i in range(len(lines)):
                draw.text((PAD + 28, cy + i * ROW), str(first + i), font=LABEL_FONT, fill="#63758e")
            cy += len(lines) * ROW + 21
        y += card_height + 28
    for i, line in enumerate(notelines):
        draw.text((PAD, y + 13 + i * 38), line, font=SUB_FONT, fill="#94a3b8")
    destination = OUT / name
    canvas.save(destination, optimize=True, dpi=(180, 180))
    print(f"{destination}: {W} x {height}")


render("01_odometry_coordinates.png", "01 / Odometry and field coordinates",
       "Robot-center pose in inches. Field origin is the center; +X faces Blue, +Y points away from the audience.", [
    ("Field coordinates and tile-to-inch math", [
        excerpt("navigation/BiobuzzFieldMap.java", " * BIOBUZZ TU06", count=4),
        excerpt("navigation/BiobuzzFieldMap.java", "public static final double FIELD_SIZE_INCHES", count=4),
        excerpt("navigation/BiobuzzFieldMap.java", "public static Waypoint tileCenter", count=7),
    ]),
    ("Two-pod scale, offsets and Pinpoint setup", [
        excerpt("config/RobotConfig.java", "public static final class Pinpoint", count=12),
        excerpt("pedroPathing/Constants.java", "public static PinpointConstants pinpointConstants", count=10),
    ]),
    ("Read and update the fused pose through Pedro", [
        excerpt("pedroPathing/PushTunaFinal.java", "public void updatePose()", count=2),
        excerpt("pedroPathing/PushTunaFinal.java", "        io.updatePose();", count=2),
        excerpt("opmodes/TeleTourney.java", "      follower.setStartingPose(new Pose(0", count=3),
        excerpt("opmodes/TeleTourney.java", "        Pose pose = follower.getPose();", count=1),
    ]),
], "Pinpoint hardware integrates the two odometry pods and onboard IMU. The active app reads its fused pose through Pedro; "
   "these files do not manually integrate individual wheel deltas. TeleTourney starts at (0,0,0); autos seed the alliance field start.")

render("02_limelight_servo_aiming.png", "02 / Limelight error to servo position",
       "Horizontal image error tx (degrees) becomes a bounded positional-servo correction. Increasing servo position turns left.", [
    ("Verified servo range and feedback settings", [
        excerpt("config/CameraPanConfig.java", "public static final double MIN_POSITION", count=4),
        excerpt("config/CameraPanConfig.java", "  // Verified: increasing servo", count=2),
        excerpt("config/CameraPanConfig.java", "public static final double TX_DEADBAND_DEGREES", count=1),
        excerpt("config/CameraPanConfig.java", "public static final double IMAGE_UPDATE_INTERVAL_SECONDS", count=5),
    ]),
    ("Read the tag's horizontal angular error", [
        excerpt("localization/AprilTagTracker.java", "    for (LLResultTypes.FiducialResult", count=4),
    ]),
    ("Fresh frames, deadband, gain and clamped motion", [
        excerpt("control/ImagePanTracker.java", "    if (!Double.isFinite(timestamp)", "    status = \"TRACKING: correcting horizontal offset\";"),
        excerpt("control/ImagePanTracker.java", "  private double clamp(double value)", count=1),
    ]),
    ("Send the new position to the camera servo", [
        excerpt("control/LimelightPanController.java", "        manualImageControls.update(false, false", count=2),
        excerpt("control/LimelightPanController.java", "      position = imageAim.getPosition();", count=2),
    ]),
], "Correction: delta = clamp(-0.001 * tx, -0.01, +0.01); position stays within 0.05-0.35. "
   "This aims left/right only. TeleTourney uses this tracking; live Red/Blue autos hold center for tag-height confirmation.")

render("03_gate_jam_recovery.png", "03 / Gate motor jam recovery",
       "Encoder movement and measured RPM detect a sustained stall. Recovery stops, reverses gently, pauses and resumes.", [
    ("Detection and recovery settings", [
        excerpt("config/RobotConfig.java", "public static final boolean JAM_RECOVERY_ENABLED", count=11),
    ]),
    ("Detect low progress, retry or latch a fault", [
        excerpt("control/GateJamRecoveryController.java", "      case RUNNING:", "      default: break;"),
    ]),
    ("Nonblocking stop -> reverse -> pause -> resume", [
        excerpt("control/GateJamRecoveryController.java", "      case STOPPING:", count=14),
        excerpt("control/GateJamRecoveryController.java", "  private void beginRunning", "  private void fault()"),
    ]),
    ("Apply the recovery command to the motor", [
        excerpt("subsystems/GateMotorSubsystem.java", "    double command = getCommandedRpm();", "    motor.setVelocity(output"),
    ]),
], "After three unsuccessful retry cycles the controller commands zero RPM. A zero requested RPM clears the fault. "
   "Recovery runs every loop without sleeps, allowing drive and shooter controls to keep updating.")
