# Publish to your private GitHub account

This folder is a separate Git repository. It does not alter the running
development project. The chosen visibility is **private**.

Sign in on your own terminal/browser; do not put passwords or tokens in chat:

```bash
gh auth login --hostname github.com --git-protocol https --web
```

Then, from this repository root:

```bash
./publish_github.sh
```

The helper creates a **private** repository named `mtare-2d` in the signed-in
account and pushes the prepared main branch. To select a different name:

```bash
./publish_github.sh my-mtare-2d-baseline
```

It refuses a dirty working tree or an existing origin remote. Review local
changes and commit them before publishing. If an origin already exists, inspect
`git remote -v` and use `git push` when appropriate. If GitHub already has that
repository name, choose another name instead of overwriting it.

The initial local snapshot is attributed to repository preparation, not to an
invented human identity. If you want your GitHub identity on that snapshot,
configure your chosen Git name/email and amend before the first push.

A private URL is not readable by every ChatGPT session. Use an authorized
GitHub connection, or attach the source archive/key files along with
`docs/CHATGPT_CONTEXT.md`. Large RRD recordings can be shared separately when
needed. Review `THIRD_PARTY_NOTICES.md` before selecting a project-wide reuse
license or changing visibility; this package has not assigned one on your behalf.
