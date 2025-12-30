# Repository Guidelines

## Project Structure & Module Organization
Firmware sources live in `keyboards/`, `layouts/`, `quantum/`, and `tmk_core/`; docs stay in `docs/`, while GoogleTest suites sit beside features or under `tests/` with matching `rules.mk`. Hardware drivers and reusable libraries live in `platforms/` and `lib/`. Mirror the existing layout and name `.c/.h` files after their folder (`keyboards/<vendor>/<board>/<board>.c`) so build scripts auto-discover them.

## Build, Test, and Development Commands
Install tooling once with `python3 -m pip install -r requirements-dev.txt`, which provides the `qmk` CLI plus formatters. Common commands:
- `qmk compile -kb <keyboard> -km <keymap>` or `make <keyboard>:<keymap>` builds firmware artifacts.
- `qmk flash -kb <keyboard> -km <keymap>` compiles and flashes while the board sits in bootloader mode.
- `make test:all` (or `make test:<substring> DEBUG=1`) runs the GoogleTest suites defined in `tests/` and prints console traces when debugging.
- `qmk docs -b` previews documentation changes at `http://localhost:8936/`.

## Coding Style & Naming Conventions
Follow the style of the file you edit: 4-space indents, braces on the same line, and descriptive `snake_case` symbols for functions and macros. Many files rely on `clang-format`; respect `// clang-format off` guards, keep includes grouped, and default new feature flags to `no` in `rules.mk`. Python helpers use `yapf` and `flake8` settings from `setup.cfg`, so run those before committing. Document every new option or behavior in `docs/` and keep keyboard READMEs updated with layouts or bootloader notes.

## Testing Guidelines
Any change to shared subsystems (matrix scanning, RGB stack, split comms, transport drivers) should gain deterministic unit tests next to the feature or under `tests/`. Update or add `rules.mk` and `test.mk` files as needed, prefer small focused executables, and cover at least one failure path. Run `make test:all` locally, then spot-check faster shards via `make test:retro_shift` or similar filters for faster feedback. As a final smoke check, rebuild the affected keyboards (`qmk compile -kb clueboard/66/rev4:all`) to ensure no config drifts slipped in.

## Commit & Pull Request Guidelines
Write commits with imperative subjects under 70 characters, blank line, and optional rationale that references the issue or PR (`Fix pointer init in RGB matrix (#25892)`). Separate functional, doc, and cleanup changes into discrete commits so reviewers can bisect easily. Pull requests must list the exact build/test commands executed, summarize user-visible effects, and link relevant docs updates. Keep branches rebased on `master` and request reviews from keyboard maintainers noted in `keyboards/<board>/readme.md`.
