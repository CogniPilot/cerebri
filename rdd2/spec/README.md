# RDD2 Specification Index

This directory is the source of truth for the `rdd2` platform.

## Active Specifications

| Spec | Title | Domain | Lines | Status |
|------|-------|--------|-------|--------|
| [SPEC_0000](SPEC_0000_SPEC_GUIDELINES.md) | Specification Writing Guidelines | process | ~55 | ACCEPTED |
| [SPEC_0001](SPEC_0001_REPO_ARCHITECTURE.md) | Repository Architecture | architecture | ~45 | ACCEPTED |
| [SPEC_0002](SPEC_0002_LATENCY_DRIVEN_ARCHITECTURE.md) | Latency-Driven Architecture | architecture | ~45 | ACCEPTED |
| [SPEC_0003](SPEC_0003_RATE_MODE_CONTROL_SCOPE.md) | Manual Flight Control Scope | control | ~45 | ACCEPTED |
| [SPEC_0004](SPEC_0004_TROPIC_HARDWARE_SCOPE.md) | Tropic Hardware Scope | hardware | ~45 | ACCEPTED |
| [SPEC_0005](SPEC_0005_GNSS_STAGING.md) | GNSS Staging | hardware | ~35 | DRAFT |
| [SPEC_0006](SPEC_0006_CODE_SIZE_AND_DEBUG_SHELL.md) | Code Size and Debug Shell | convention | ~35 | ACCEPTED |
| [SPEC_0007](SPEC_0007_FLATBUFFER_TOPICS.md) | FlatBuffer Topic Format | convention | ~30 | ACCEPTED |
| [SPEC_0008](SPEC_0008_SELF_DESCRIBING_LOG_RECORDS.md) | Self-Describing Log Records | logging | ~30 | ACCEPTED |
| [SPEC_0009](SPEC_0009_NATIVE_SIM_SITL.md) | Native Sim SITL Transport | simulation | ~35 | ACCEPTED |

## Usage

- Read this file first.
- Read `SPEC_0002`, `SPEC_0004`, and `SPEC_0006` before changing flight-path code.
- Read `SPEC_0007` before changing topic payload layout or debug topic publication.
- Read `SPEC_0008` before adding SD-card logging or changing log-record format.
- Read `SPEC_0009` before changing native-sim SITL transport or simulator message contracts.
- Read `SPEC_0000` before adding or changing specs.
