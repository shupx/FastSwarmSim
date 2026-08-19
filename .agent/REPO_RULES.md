# FastSwarmSim Repository Rules

## Scope

These rules apply to agents working in this repository, including Codex and other coding agents.

## Repository

- GitHub repository: `https://github.com/shupx/FastSwarmSim.git`
- Primary remote: `origin`
- Main branch: `main`
- Documentation site: `https://shupx.github.io/FastSwarmSim/`

## Change discipline

1. Before modifying files, inspect `git status`, the current branch, remotes, and recent commits.
2. Keep changes focused on the requested task. Do not modify generated `install/`, `build/`, or `log/` artifacts.
3. After every repository change, report:
   - commit hash and subject;
   - changed files and a concise summary;
   - validation performed and its result;
   - whether changes were pushed;
   - current `git status` and local/remote synchronization state.
4. Do not claim completion from an agent's self-report alone. Verify the files, commit, tests/build, and Git status.
5. Do not push unless the user explicitly asks for pushing. For documentation work, the user has previously authorized pushing only when explicitly requested in that task.

## Documentation

- The canonical full documentation is under `docs/` and is deployed with MkDocs Material to GitHub Pages.
- The root `README.md` is the project entry point and should contain concise setup information, supported platforms, the documentation-site link, and contributors.
- Module `README.md` files remain useful local/module documentation. When a source README is mirrored under `docs/`, preserve synchronization and verify the copies when changing one.
- Supported platform documentation must cover Ubuntu 22.04 + ROS 2 Humble and Ubuntu 24.04 + ROS 2 Jazzy.
- Documentation-only changes should use a `docs:` Conventional Commit when appropriate.
- Validate documentation with `mkdocs build --strict` before reporting success.

## Documentation deployment

- `.github/workflows/docs.yml` deploys on pushes to `main` and via `workflow_dispatch`.
- `mkdocs.yml` uses the Material theme. `repo_url` and `theme.icon.repo` provide the GitHub repository link/icon in the site header.
- Do not commit the generated `/site/` directory; it is ignored by `.gitignore`.

## Changelog and releases

- Changelog generation intentionally does not use AI and does not summarize diffs.
- `git-cliff` generates changelog entries from Conventional Commit messages and groups them by type.
- Keep commit messages informative, for example:
  - `feat(fss-time): add ...`
  - `fix(fss-sensing): correct ...`
  - `docs: update ...`
  - `ci: update ...`
- `cliff.toml` defines the grouping and formatting.
- `.github/workflows/release.yml` triggers on `v*` tags, generates `CHANGELOG.md`, mirrors it to `docs/CHANGELOG.md`, commits the files to `main`, and creates a GitHub Release.
- Release workflow changes must be checked carefully because it pushes an automated changelog commit to `main`, which then triggers documentation deployment.

## Agent delegation

- Delegation is optional. If a sub-agent or Codex is used, the primary agent must verify its result in the actual repository before reporting completion.
- The primary agent should tell the user whether work was done directly, by a Hermes sub-agent, or by Codex CLI.
