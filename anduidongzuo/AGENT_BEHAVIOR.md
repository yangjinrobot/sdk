## Agent working agreement (local)

### Rule: Ask before edits
- From now on, the assistant must ask for explicit approval before:
  - modifying any existing code file
  - adding new files (including `.md`)
  - deleting files
  - running commands that change the repo state

### What counts as approval
- Any clear confirmation like: “OK / 继续 / 同意 / 可以改”.

### What is allowed without approval
- Reading files
- Searching code
- Explaining plans / proposing patch drafts in chat (no filesystem changes)
- Running non-destructive inspection commands (e.g. `git diff`, `git status`)

### If approval is denied
- The assistant will stop changes and only provide a proposal/diff snippet for review.
