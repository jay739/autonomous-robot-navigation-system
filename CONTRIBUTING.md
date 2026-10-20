# Contributing to autonomous-robot-navigation-system

Thanks for considering a contribution. Issues, discussions and PRs are all
welcome.

## Getting set up

Arduino IDE or arduino-cli with the ESP32 board package (2.0.x). Copy `config.example.h` to `config.h` and fill in your WiFi and network settings. `config.h` is gitignored, never commit it.

## Running the checks

CI compiles both sketches for their boards on every PR. Flash and test on the hardware for anything that changes motion or detection.

## Ground rules

- Open an issue or discussion before large changes, so effort isn't wasted.
- Keep PRs focused: one change per PR.
- New behavior needs a test where the repo has a test suite.
- CI must pass before merge.

## Licensing

By contributing you agree your contributions are licensed under the repo's
MIT license.
