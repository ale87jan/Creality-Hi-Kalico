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
| `dev` | Default branch: integration of Hi work. Receives task branches through pull requests |
| `main-hi` | Stable Hi branch: receives `dev` (by pull request) once it works on the printer. The installer uses it |
| `<type>/<topic>` | Task branches off `dev` (`feat/motor-stall-pins`, `fix/...`, `docs/...`) |
| `origin` | `ale87jan/Creality-Hi-Kalico` |
| `upstream` | `Jacob10383/kalico` (push disabled) |
| `kalicocrew` | `KalicoCrew/kalico` (push disabled) |

Flow: task branch off `dev` → pull request into `dev` (CI runs on pull requests) → when tested on the printer,
pull request `dev` → `main-hi`. Never commit directly on `main`, `dev` or `main-hi`.

Sync with upstream (the merge into `dev` also goes through a pull request):

```bash
git fetch upstream
git switch main && git merge --ff-only upstream/main && git push origin main
git switch -c chore/sync-upstream dev && git merge main
```

After each sync, re-check the Hi MCU compatibility (see Creality-Hi-CFW `docs/protocols/mcu.md`).

## Commits

- [Conventional Commits](https://www.conventionalcommits.org/): `type(scope): description`, imperative,
  English, title <= 72 chars (aim for 50). Optional free-form body explaining why.
- Types: `feat`, `fix`, `docs`, `refactor`, `test`, `perf`, `build`, `ci`, `chore`, `revert`. Scope `hi` or
  the module (`motor`, `box`, `prtouch`, `z_align`...).
- No `Co-Authored-By` or "Generated with" lines.
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
- GitHub Actions: only the pull request checks (`ci-build_test`, `ci-lintformat`, `ci-docs_test`) should run on
  this fork. The publishing workflows inherited from Kalico (`cd-docker`, `cd-docs`, `ci-builder`,
  `cd-bleeding-edge`, `release-monthly-tag`) must stay disabled.

## Safety rules

- Printer tests start read-only. Never send motor writes (`SYS_PARAM` write/apply, `FLASH_PARAM`,
  `ENCODER_CALIBRATE`, `ELEC_OFFSET_CALIBRATE`, `MOTOR_FLASH_PARAM COMMIT=1`) without explicit user approval.
- Never run startup code that writes motor parameters until the Hi parameter ids are re-verified for the
  firmware in use (`mot2_002_071` verified 2026-11-09).
- Cloud sessions cannot reach the printer: hardware tests run on the user's PC.
