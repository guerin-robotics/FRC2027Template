---
mode: agent
description: Logical and behavioral audit of robot code — bugs, not style.
---

# Code Review

Follow [`.github/agents/code-review.md`](../agents/code-review.md) in full. Read that file
first, then carry out the review exactly as it specifies — scope, what to look for, and the
output format including the Hard Stop section and the closing risk classification.

This file is a thin entry point on purpose. It used to be a near-verbatim copy of the agent
definition, and the two drifted together: both ended up telling reviewers that `Triggers.java`
might not exist and that bindings live inline in `RobotContainer` — the opposite of the rule,
and now a build failure. One definition, one place to change it.
