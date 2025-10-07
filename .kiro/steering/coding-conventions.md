# Coding Conventions

## Code Implementation Rules

- **Always write code directly to the correct files** - Never create dummy or pseudo code without permission
- **Write production-ready code** - All code should be complete, functional, and ready to use
- **No placeholder implementations** - Avoid stub functions, TODOs, or incomplete logic unless explicitly requested

## Documentation Policy

- **Never create READMEs, summaries, documentation, or other additional artifacts** (such as README.md files, project overviews, or automatic reports) without explicit request
- **Always wait for direct instruction** from the user before generating or adding such content
- **Focus on code, not documentation** - When asked to implement a feature, write the code only
- **Exception**: Inline code comments are acceptable and encouraged for clarity

## What This Means

When asked to "add a feature" or "implement X":
- ✅ Write the actual implementation code
- ✅ Add necessary imports and dependencies
- ✅ Include inline comments for complex logic
- ❌ Don't create a README explaining what you did
- ❌ Don't write a summary document
- ❌ Don't generate project overviews or reports

The user will ask for documentation when they need it.

## Codebase Investigation

- Explore relevant files and directories
- Search for key functions, classes, or variables related to the issue
- Read and understand relevant code snippets
- Identify the root cause of the problem
- Validate and update your understanding continuously as you gather more context

## Making Code Changes

- **Before editing, always read the relevant file contents or section** to ensure complete context
- **Always read files that are imported** in the file you are editing to ensure you have full context
- Make sure to always understand the full context of the codebase before making changes
- Always think through the implications of your changes on:
  - The entire codebase
  - Any dependencies and interactions with other parts of the code
  - Edge cases and potential pitfalls
  - Performance, security, and maintainability
- If a patch is not applied correctly, attempt to reapply it
- Make small, testable, incremental changes that logically follow from your investigation and plan

## Environment Variables

- Whenever you detect that a project requires an environment variable (such as an API key or secret), always check if a `.env` file exists in the project root
- If it does not exist, automatically create a `.env` file with a placeholder for the required variable(s) and inform the user
- Do this proactively, without waiting for the user to request it
