---
id: 1
title: Update Constitution
stage: constitution
date_iso: 2025-12-08
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "governance", "spec-driven"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - .specify/memory/constitution.md
  - history/prompts/constitution/1-update-constitution.constitution.prompt.md
tests_yaml: |
  - none
outcome: |
  Successfully updated the project constitution with comprehensive principles for AI/Spec-Driven book creation
evaluation: |
  The constitution now contains 6 core principles covering accuracy, clarity, AI validation, reproducibility, transparency, and continuous improvement, plus technical constraints and governance rules.
---

# Update Constitution

## User Input
```
Project: AI/Spec-Driven Unified Book Creation using Spec-Kit Plus and Claude Code

Objective:
To design, write, and deploy a complete, production-quality technical book using AI-native software development practices, powered by Spec-Kit Plus and Claude Code, and published through Docusaurus on GitHub Pages.

Core Principles:

* Accuracy through direct verification of primary and trusted technical sources
* Clarity and accessibility for a technical audience (students, developers, and AI engineers)
* AI-native development with human-in-the-loop validation
* Reproducibility of content, structure, and workflow
* Transparency of sources, tools, and generation process
* Continuous improvement through iterative refinement

Key Standards:

* All factual and technical claims must be verifiable through credible sources
* All explanations must be beginner-friendly while remaining technically correct
* AI-generated content must be reviewed, edited, and validated by humans
* Code examples must be runnable, minimal, and clearly explained
* Citations must be consistently formatted
* No hallucinated concepts, tools, or references are allowed
* All chapters must follow a consistent documentation structure defined by Spec-Kit Plus

Source & Citation Rules:

* Minimum 60% sources from official documentation, research papers, or authoritative technical platforms
* Remaining sources may include trusted blogs, GitHub repositories, and industry case studies
* Plagiarism tolerance: 0%
* All reused ideas must be rewritten originally with proper attribution

Writing & Content Quality:

* Language: Clear, precise, and structured
* Tone: Professional, educational, and practical
* Target readability: Intermediate technical level
* Each chapter must include:

  * Concept explanation
  * Practical example
  * Use case or real-world application
  * Summary

Technical Constraints:

* Platform: Docusaurus
* Deployment: GitHub Pages
* Tooling:

  * Spec-Kit Plus for specification-driven structure
  * Claude Code for AI-assisted content generation
* Version control must be maintained via GitHub
* All content must be written in Markdown
* The book must build successfully with zero build errors

Project Constraints:

* Book length: Determined by topic depth, no artificial padding
* Chapters must be modular and independently readable
* No copyrighted material may be copied directly
* All diagrams must be either original or properly attributed

Success Criteria:

* Fully deployed live book on GitHub Pages
* All chapters pass human factual verification
* Zero plagiarism detected
* Clean GitHub repository with documented workflow
* Clear demonstration of AI-native software development using Spec-driven processes
* Judges can reproduce the book-generation process using the provided specs

Ethical & AI Usage Guidelines:

* AI is used as a co-author, not a replacement for human judgment
* All misleading, biased, or unverifiable content must be removed
* Safety, privacy, and responsible AI principles must be respected at all stages

Final Acceptance Conditions:

* The book builds successfully
* The deployed site is publicly accessible
* The content reflects real understanding, not just AI output
* The project clearly demonstrates the power of Spec-Kit Plus + Claude Code working together
```

## Response
The constitution has been successfully updated with the following principles:

1. **Accuracy through Direct Verification**: All factual and technical claims must be verifiable through credible sources; All explanations must be beginner-friendly while remaining technically correct.

2. **Clarity and Accessibility**: Content must be clear, precise, and structured with professional, educational, and practical tone for a technical audience (students, developers, and AI engineers).

3. **AI-Native Development with Human-in-the-Loop Validation**: AI-generated content must be reviewed, edited, and validated by humans; No hallucinated concepts, tools, or references are allowed.

4. **Reproducibility of Content, Structure, and Workflow**: All chapters must follow a consistent documentation structure defined by Spec-Kit Plus; Version control must be maintained via GitHub.

5. **Transparency of Sources, Tools, and Generation Process**: Source & Citation Rules: Minimum 60% sources from official documentation, research papers, or authoritative technical platforms; All reused ideas must be rewritten originally with proper attribution.

6. **Continuous Improvement Through Iterative Refinement**: Content must be continuously improved through iterative refinement.

The constitution also includes Technical Constraints, Development Workflow and Quality Standards, and Governance rules as specified in the user requirements. The version is set to 1.0.0 with today's date (2025-12-08) as the last amended date.