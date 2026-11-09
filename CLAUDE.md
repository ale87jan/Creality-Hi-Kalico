# CLAUDE.md

Kalico for the **Creality Hi** (F018). Fork of [Jacob10383/kalico](https://github.com/Jacob10383/kalico) (Kalico
with open modules for the Creality K2 Plus), itself based on
[KalicoCrew/kalico](https://github.com/KalicoCrew/kalico), a fork of Klipper.

Research, hardware facts, protocols and the roadmap live in the companion repo
[ale87jan/Creality-Hi-CFW](https://github.com/ale87jan/Creality-Hi-CFW) (local `C:\Github\Creality-Hi-CFW`):
read its `docs/roadmap.md`, `docs/hardware.md` and `docs/protocols/` before changing hardware code. This repo
holds code only.

## Working preferences

- Answer in Spanish. Code, comments and commit messages in English.
- Use Haiku or Sonnet subagents for tasks; Claude coordinates and verifies. **Ask the user before** running an
  Opus agent or doing a deep analysis yourself.

## Branches and remotes

| Branch / remote | Role |
| - | - |
| `main` | Mirror of `upstream/main` (Jacob10383/kalico). Only fast-forward from upstream; never commit here |
| `dev` | Default branch: all Hi work, feature branches start from here |
| `origin` | `ale87jan/Creality-Hi-Kalico` |
| `upstream` | `Jacob10383/kalico` (push disabled) |
| `kalicocrew` | `KalicoCrew/kalico` (push disabled) |

Sync with upstream:

```bash
git fetch upstream
git switch main && git merge --ff-only upstream/main
git switch dev && git merge main
```

After each sync, re-check the Hi MCU compatibility (see Creality-Hi-CFW `docs/protocols/mcu.md`).

## Commits

- Format `type(scope): description`, imperative, English, title <= 50 chars. Types: `feat`, `fix`, `refactor`,
  `tune`, `docs`, `chore`. Scope `hi` for Hi-specific work.
- Body: `CHANGES BY FILE:` (new files marked `[NEW]`) and `IMPROVEMENTS:`. No `Co-Authored-By` or
  "Generated with" lines.
- Atomic commits; never push or force-push unless asked.

## How Hi code is added

- Keep upstream merges easy: prefer **new config options** over editing hard-coded K2 values in shared modules
  (e.g. stall pins `STALL_PIN_LAYOUT`, address straps `PIN_DIR_SEQUENCE` in `klippy/extras/motor_control.py`).
- Hi-only modules go in new files; Hi configs in `config/hi/` (mirroring `config/k2/`).
- Do not use the K2 `z_align.py` on the Hi: its MCU lacks `safeDist`. Port the Hi's own `z_align.py`.
- Target runtime: stock Hi Tina OS, Python 3.9.12 (cffi 1.14.6, greenlet 1.0.0, numpy 1.20.1, Jinja2 3.0.3);
  `c_helper.so` must be cross-compiled for ARMv7 hard-float, glibc 2.29.

## Tests

- Host tests run with pytest (`py.test -n auto`, see `.github/workflows/ci-build_test.yaml`); dev
  dependencies are in `pyproject.toml` (`uv sync`).
- Lint/format with ruff (`pyproject.toml`).

## Safety rules

- Printer tests start read-only. Never send motor writes (`SYS_PARAM` write/apply, `FLASH_PARAM`,
  `ENCODER_CALIBRATE`, `ELEC_OFFSET_CALIBRATE`, `MOTOR_FLASH_PARAM COMMIT=1`) without explicit user approval.
- Never run startup code that writes motor parameters until the Hi parameter ids are re-verified for the
  firmware in use (`mot2_002_071` verified 2026-11-09).
- Cloud sessions cannot reach the printer: hardware tests run on the user's PC.
