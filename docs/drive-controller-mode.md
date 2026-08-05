# Drive Controller Mode — Swapping Controllers Between Matches

> **NOT IMPLEMENTED IN THIS TEMPLATE.** The 2026 robot had a dashboard-selectable
> controller swap so the drive team could rotate drivers between matches without a
> redeploy. It depended on `Triggers.java` and a controller-constants block,
> neither of which carried over.
>
> This document is kept as a **design record**, because the pattern is worth rebuilding
> and the details below were learned the hard way at an event. Delete this banner once
> the 2027 version exists, and rewrite the sections as operating instructions.

---

## The problem it solves

Two drivers, two controller types, one match schedule. Redeploying code between matches
to switch which stick drives is slow and risky. The 2026 solution let the drive coach
pick from a dashboard dropdown during the pre-match, with both controllers plugged in the
whole time — only their *roles* swapped.

---

## The design

A `LoggedDashboardChooser<Boolean>` published in the `RobotContainer` constructor, read
**exactly once** per enable, in `teleopInit()`:

```java
// RobotContainer — declare the options
driveControllerChooser = new LoggedDashboardChooser<>("Drive controller");
driveControllerChooser.addDefaultOption(FLIGHTSTICK_OPTION, false);
driveControllerChooser.addOption(XBOX_OPTION, true);

public boolean isXboxDriveSelected() {
  return driveControllerChooser.get();
}

// Robot.teleopInit() — the latch. The ONLY place the selection is read.
ControllerConstants.XBOX_DRIVE_MODE = robotContainer.isXboxDriveSelected();

// Robot.robotPeriodic() — log the latched value so match logs explain the behavior
Logger.recordOutput("driveController", ControllerConstants.XBOX_DRIVE_MODE);
```

Every drive-axis read then routes through a single accessor rather than touching a
controller object directly:

```java
private double getDriveX() { return Triggers.getInstance().driveXSupplier(); }
private double getDriveY() { return Triggers.getInstance().driveYSupplier(); }
private double getDriveRot() { return Triggers.getInstance().driveRotSupplier(); }
```

### Why latch instead of reading per-loop

Two reasons, both learned in practice:

1. **Correctness.** If the selection is read every loop, changing the dropdown mid-match
   swaps the controller out from under the driver. Latching at `teleopInit` means the
   choice can only change on a disable → enable boundary.
2. **Loop time.** Reading a NetworkTables value every cycle puts dashboard traffic in the
   20 ms loop for no benefit.

### Why every axis read must go through the accessor

This is the failure mode that actually bit. If one command reads
`Triggers.getInstance().thrustmaster.getX()` directly while the default drive command
uses the gated supplier, that command follows a joystick nobody is holding. The symptom
is an align command that drifts on its own while normal driving works fine — hard to
diagnose, trivial to prevent.

---

## Operational notes worth keeping

**Driver station ports.** Confirm which port each controller landed on in the DS USB tab
before the first match. If a controller is unplugged and replugged it can come back on a
different port, and the symptom is "my controller does nothing" — no error message.

**Brownouts don't reset it.** A brownout cuts motor power but does not restart robot
code, so the latched selection survives. This was confirmed against 2026 practice logs
with 64, 9, 3, and 1 brownout events — all kept running with no code restart.

**Code restart, RIO reboot, or redeploy loses it.** The selection is not persisted
anywhere. Robot code comes back on the default option. Re-pick from the dropdown before
enabling, and re-check it after any redeploy between matches.

---

## If you rebuild this

1. Put the chooser in `RobotContainer`, latch in `Robot.teleopInit()`, log in
   `robotPeriodic()`
2. Route **every** drive-axis read through the gated suppliers — no exceptions
3. Publish the widget under SmartDashboard so it appears before enable
4. Save the dashboard layout so the widget survives an Elastic restart
5. Update `docs/driver-controls-card.md` with a section per mode
6. Add the port map to `docs/hardware-layout.md`
