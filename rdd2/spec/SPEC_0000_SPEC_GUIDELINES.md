# SPEC_0000: Specification Writing Guidelines

## Status
ACCEPTED

## Summary
Every active `rdd2` spec lives in `spec/`, uses a numbered `SPEC_NNNN_*.md` filename, and states its rules in fast-to-scan REQUIRED/PROHIBITED form.

## Specification

**REQUIRED:**
- Active specs live in `spec/`.
- Active spec filenames follow `SPEC_NNNN_UPPER_SNAKE_CASE.md`.
- Every active spec is listed in `spec/README.md`.
- Every spec starts with these sections in this order:
  - `# SPEC_NNNN: Title`
  - `## Status`
  - `## Summary`
  - `## Specification`
- Status is one of:
  - `ACCEPTED`
  - `DRAFT`
  - `REFERENCE`
- Specs use direct rules, not narrative essays.
- Specs describe the current repo or an explicitly marked draft direction.
- New project rules are added to a spec in the same change that introduces them.

**SHOULD:**
- Keep specs short enough to scan in one read.
- Add `## Motivation` when the reason for the rule is not obvious.
- Add `## References` when another spec or external document governs the rule.
- Show actual file paths or concrete repo locations when that reduces ambiguity.

**PROHIBITED:**
- Unnumbered active specs.
- `ACCEPTED` specs that describe behavior the repo does not actually follow.
- Hidden project rules that exist only in code comments or agent prompts.
- Multiple active specs covering the same rule in conflicting ways.

## Motivation

- This repo is intended to be easy for both humans and AI to navigate.
- Fast comprehension matters more than long prose.
- One numbered index avoids drift and ambiguity.

## References

- `../../AGENTS.md`
- `SPEC_0006_CODE_SIZE_AND_DEBUG_SHELL.md`
