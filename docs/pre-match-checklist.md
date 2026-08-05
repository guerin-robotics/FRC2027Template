# Pre-Match Checklist

Print this. Keep it in the pit. Work it every match, including the ones you think don't
matter.

> **Fill in the mechanism-specific rows as the 2027 robot is built.** The structure below is
> what the 2026 team ran; the specific checks were theirs. A checklist nobody updates is a
> checklist the drive team stops reading.

A short version of this is published to the dashboard by `Robot.java` under
**"Pre-Match Checklist"** so it is visible from the driver station without paper. Keep the
two in sync — edit both or neither.

---

## Between Matches (start as soon as you're off the field)

- [ ] **Swap the battery.** Every match. A battery that "felt fine" is the one that browns
      out in the last 20 seconds
- [ ] Log the battery ID used, so a bad one can be traced across matches
- [ ] Pull match logs off the robot (`tools/log-sync`, or the USB stick)
- [ ] Anything unusual reported by the drive team → write it down *now*, while it's fresh

## Mechanical

- [ ] Bumpers secure, correct alliance color, numbers legible
- [ ] Nothing loose — grab each mechanism and shake it
- [ ] All fasteners on anything that moved this match
- [ ] Wheels/tread: no chunks missing, no visible wear step
- [ ] Chains/belts at tension
- [ ] *(add 2027 mechanism-specific checks here)*

## Electrical

- [ ] Battery fully seated and strapped
- [ ] Main breaker in, battery leads tight
- [ ] No visible damage to wiring; nothing pinched by a mechanism
- [ ] **USB drive present** — no USB means no match log, and no way to debug afterward
- [ ] Radio powered and lights normal

## Software

- [ ] **Phoenix Tuner: device count matches expected.** Write the number here once known:
      `____ devices`. A missing device is the single fastest thing this check catches
- [ ] No CAN error indicators; CANivore bus utilization normal
- [ ] Correct code deployed — check `GitSHA` / `GitDirty` metadata in the log or dashboard
- [ ] `GitDirty` reads "All changes committed". Uncommitted code at an event means the log
      cannot be matched to source later
- [ ] Tuning mode OFF, demo mode OFF
- [ ] Correct auto selected on the dashboard
- [ ] Robot placed to match the selected auto's starting pose

## Function Check (on the cart or in the pit, before queueing)

- [ ] Drive — all four modules respond, robot translates and rotates correctly
- [ ] Gyro reads sensible; heading holds when pushed straight
- [ ] Vision: cameras connected, tags detected (`Vision/Camera*/TagCount` non-zero when
      pointed at a tag)
- [ ] *(add 2027 mechanism function checks here — one line each, in the order the drive
      team will exercise them)*

## In Queue

- [ ] Robot enabled and disabled once without error
- [ ] Driver station battery charged, laptop plugged in or > 50%
- [ ] Driver knows which auto is selected
- [ ] Someone has eyes on the field for the robot placement

---

## After a Problem

If something went wrong on the field, before touching code:

1. Pull the log immediately — logs are overwritten by nothing, but robots get reflashed
2. Write down what the drive team *saw*, physically. The log has the data; only they have
   the physical context
3. Use `/debug-match-log` to correlate the two
4. Any fix that touches gains, thresholds, timeouts or vision filters is Level 3+ at an
   event — see `docs/change-classification.md` for the competition standard, which requires
   sign-off

---

## Competition Change Log

Keep a running list of every change made between matches. At the 2026 events this was the
difference between "the robot got worse and we don't know why" and a one-line revert.

| Match | Change | Why | Who |
|---|---|---|---|
| | | | |
