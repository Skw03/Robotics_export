# AI Tools Usage Statement

## Tools Used

| Tool | Role in project | Validation method |
| --- | --- | --- |
| Codex coding agent | Code inspection, ROS 2 script generation, documentation drafting, test planning | Static checks, command-line dry runs, source review |
| OpenAI API through `course_llm_command.py` | Natural-language task parsing into executable `scene` and `task` presets | 10-command dry-run test, latency logging, fallback comparison |
| `ros2-engineering-skills` | ROS 2, Nav2, launch, Gazebo, action, and evaluation design guidance | Manual mapping to source files and ROS 2 commands |
| Spreadsheet skill | Experiment workbook and chart template generation | Workbook formulas and chart inspection |
| Presentations skill | Midterm and final defense deck generation | Slide review against required course sections |
| Documents skill | Final report draft and AI usage appendix | Render/visual check when LibreOffice is available |

## Cost and Resource Estimate

The local fallback parser has no API cost. The LLM parser only calls the OpenAI API when `OPENAI_API_KEY` is set. For the required 10-command reliability test, expected API usage is small because each command maps to a compact JSON schema. Record the actual token usage from API responses if available; otherwise report wall-clock latency and number of calls.

## Observed Limitations

- LLM parsing may choose a plausible task even when the command is underspecified.
- The current executable task space intentionally exposes only two scenes and two task types.
- API/network failure must not block the robot demo; therefore `course_llm_command.py` falls back to keyword parsing and records the error.
- Chinese command quality depends on prompt clarity and whether the command includes scene/task cues.

## Prompt and Schema

The semantic layer uses a constrained JSON schema:

```json
{
  "scene": "warehouse | office",
  "task": "delivery | patrol",
  "confidence": 0.0,
  "rationale": "short explanation"
}
```

The system prompt instructs the model to select only executable presets and not invent route names. This keeps LLM output at the semantic planning layer rather than replacing Nav2, costmaps, controllers, or behavior trees.

## Verification

Verify AI-generated code and content by:

1. Running Python syntax checks on new scripts.
2. Running LLM dry-run tests with and without `OPENAI_API_KEY`.
3. Running actual ROS 2 action dispatch after the scenario action server is available.
4. Comparing logged outputs against expected `scene` and `task` labels.
5. Reviewing failure cases in the final report instead of hiding them.

