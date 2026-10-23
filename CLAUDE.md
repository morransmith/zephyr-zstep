# CLAUDE.md

## General

- All repository content is in English.
- Public symbols use the `zstep_` prefix, public headers go in `include/zstep/`, Kconfig symbols are `CONFIG_ZSTEP_*`, devicetree compatibles use the `zstep` vendor prefix.
- Code that implements a published method names the publication in a comment at the implementation. The publications are listed in the References section of `doc/architecture.md`.

## Third-party code

- Code under Apache-2.0, MIT, BSD or 0BSD may be adapted. Keep the original copyright notice in the file and add the source to `NOTICE`.
- Code under GPL or LGPL may be read to understand behavior. Never copy or transcribe it. This covers the motion control firmwares Klipper, Marlin, RepRapFirmware, grbl and LinuxCNC, including the expressions of their formulas.
- The driver uses only the generic Zephyr counter, GPIO and stepper APIs. Vendor HAL calls must not be used.

## Build and test

```
west build -b native_sim/native/64 tests/unit/<name>
west twister -T tests -p native_sim -p native_sim/native/64
west build -b native_sim/native/64 samples/multi_axis
```

## Where these rules override Claude Code defaults

- Commit messages are one ASCII line with a conventional prefix (feat:, fix:, chore:, ! for breaking). No body except a BREAKING CHANGE: paragraph, and no Co-Authored-By or any other trailer.
- Work happens in a feature/<name> branch off develop. Never merge into develop or master, and never delete a branch.
- Never create a pull request. Never git push unless asked; one request covers one push.

## AI usage

Permitted, as common examples:

- learning, exploring and explaining the codebase;
- suggestions on human-written code;
- mechanical tasks: formatting, repetitive patterns, completing code from an established design;
- documentation drafts for components the user already understands;
- code written from a design the user owns.

Before writing code, make sure the user owns the design choices it depends on: they are recorded in `doc/architecture.md` or were decided by the user in the conversation. Ask about any choice that is not covered.

A change is acceptable only if the user can fully understand it, debug it without AI help, and discuss it with reviewers without AI help. Keep changes small enough for that.

Never write pull request descriptions or responses to reviewers.

## Writing style

Applies to every document here, and to answers given in conversation.

Say it literally. Do not substitute metaphor or flourish for direct statement. Write "a parameter worth varying", not "a dial worth turning"; "this still matters", not "this earns its keep". Mannered phrasing displays the writer instead of conveying the idea, makes the reader work harder, and is imprecise, because metaphors carry connotations that were not chosen and cannot be controlled. When a literal phrase is available, use it. Established technical terms (cache-warm, hot path, backpressure, single-flight) are not flourish and stay.

Do not define things by antithesis. State what is true and stop. Write "the log records every request", not "the log records every request, not an afterthought"; "the config file is the only source of settings", not "the config file is not a hint for the user - it is the only source of settings". The negated half usually carries no information and exists for rhythm. Two exceptions: prohibitions ("must not", "never") are specifications and stay; a rejected alternative may be named where the reason is given with it.

No sycophancy. Do not open with praise or agreement - no "good question", "you're right", "great idea" - and do not praise the design in the documents. Assessment appears only where it carries information the reader does not already have, with the reason attached. Calling something "elegant" or "non-obvious" is a claim about the writer, not about the subject.

Say each thing once. Do not write claim -> explanation -> restatement -> summary -> closing flourish. A paragraph that adds no fact is deleted, including the last one. Length must come from new facts and necessary distinctions, never from explaining the same point again in different words.

## Code

Tests first. Write the test, watch it fail for the right reason, then write the code that makes it pass. Scope is limited by the test rules below.

Code first, comments second. Write the code, then add a comment where the code cannot carry the meaning alone. Comments written before the code state what the code will state, so they end up redundant.

Comments are allowed only where they explain something non-obvious. Keep them to a line or two; no multi-paragraph prose.

Early exit. Return, raise or continue as soon as a case is decided. Rewrite nested conditionals as guard clauses and keep the main path at one indent level.

Short and plain. Prefer the version a human reads faster: fewer branches, fewer locals, direct names. Replace code that needs a comment to be understood with the obvious version.

Keep it simple. A simpler change that does 90% of the job is often preferable to a complex one that does 100%. Flag unnecessary project-defined macros, generic containers and extra abstraction layers. Standard C and Zephyr macros may be used.

Reuse before adding. Extend what is already there. Introducing a new subsystem, a new dependency or a new pattern requires the user's agreement first - ask before writing it. A large change made without prior discussion is likely to be rejected.

Read before writing. Read the files a change touches and the files around them, then match the patterns in place.

No magic numbers. A value with a meaning has a name: a define, an enum value or a Kconfig option. Physical quantities state their unit in the name (`step_width_ns`, `freq_centihz`) or in the comment of the field, and fixed-point values state their scale.

Every source file starts with an `SPDX-License-Identifier: Apache-2.0` line. Code follows the Zephyr coding style.

## Tests

- Follow the existing testing patterns: ztest suites under `tests/`, run by twister. Do not add a new testing system.
- New test cases go into an existing test file when one covers the same component. Add a new file under `tests/` only when none does.
- Add only tests that produce meaningful results. Trivial tests increase suite size and CI time without catching defects.
- No time-sensitive tests: no thresholds on host wall-clock time, no host sleeps, no races between threads. Test timer logic through Zephyr kernel time and the counter of native_sim.
- Flag tests that significantly increase CI run time (expensive computation, large inputs, long waits) or download large amounts of data, and ask whether a smaller input or an existing fixture would do.

## Git

### Commit messages

One line, in English, ASCII only. No unicode characters, no emoji. The text states briefly what was done and nothing else - no explanation, no list of files, no attribution or trailers.

The prefix drives automatic versioning:

| Commit message | Version increase |
|---|---|
| chore: foo | none |
| fix: fixed something | patch |
| feat: added new button to do X | minor |
| fix: fixed thing xyz + BREAKING CHANGE: paragraph | major |
| fix!: fixed something | major |
| feat!: added blah | major |

On every push to master, svu applies this table (.svu.yml) to the commits since the last tag and, when they raise the version, tags the pushed commit v<version>, which starts the release.

### Branches

Three kinds of branch exist:

- master - release state.
- develop - integration state.
- feature/<feature-name> - all work. Every task is done in its own feature branch, branched off develop.

### Merging

Subfeature into its feature branch: merge commit (git merge --no-ff), done here.

Feature into develop, and develop into master: done by the user through the external service. Never merge into develop or master from the command line.

### Pull requests

Never create a pull request. The user creates every pull request.

### Push

git push is forbidden unless the user asks for it explicitly. The request authorises one push. Later commits on the same feature require a new request each time.

### Examples

```
// GOOD: concise, one line, states what was done

fix: retry the request when the connection drops

// BAD: verbose, restates the change, adds a trailer

fix: comprehensive rework of connection retry handling

This commit introduces a comprehensive fix for the connection retry handling
logic, addressing an issue where a dropped connection could leave a request
without a response, thereby improving overall application reliability.

Co-authored-by: Claude Sonnet
```
