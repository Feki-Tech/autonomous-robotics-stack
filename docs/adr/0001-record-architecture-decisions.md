# ADR-0001: Record architecture decisions

## Status

Accepted

## Context

The stack will grow across several packages (core algorithms, ROS 2 adapters,
simulation, missions). Design decisions made early — dependency rules, build
strategy, testing policy — must remain discoverable and reviewable long after
the commits that introduced them.

## Decision

We record every architecturally significant decision as an Architecture
Decision Record (ADR) in `docs/adr/`, numbered sequentially, using the
Status / Context / Decision / Consequences format.

## Consequences

- Reviewers can audit *why* the code is shaped the way it is.
- Changing a past decision requires a superseding ADR, keeping history intact.
- Small overhead per significant decision; negligible for routine changes.
