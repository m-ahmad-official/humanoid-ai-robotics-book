---
id: 1
title: Amend Constitution
stage: constitution
date: 2025-12-23
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: user
command: sp.constitution
labels: [constitution, documentation]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```text
Project: AI-Spec–Driven Book with Embedded RAG Chatbot

Purpose:
Author and publish a technical book using Spec-Kit Plus and Claude Code, deployed via Docusaurus to GitHub Pages, with an embedded RAG chatbot that answers questions from the book and user-selected text.

Core Principles:
- Spec-first, AI-native development
- Technical accuracy and correctness
- Clear, developer-focused explanations
- Fully reproducible from repository

Standards:

Book:
- Framework: Docusaurus
- Deployment: GitHub Pages
- Modular chapters driven by Spec-Kit Plus
- Runnable or clearly labeled code examples
- Mermaid diagrams where useful

RAG Chatbot:
- Embedded in Docusaurus site
- Supports full-book and user-selected text Q&A
- Stack:
  - OpenAI Agents/ChatKit SDK
  - FastAPI backend
  - Neon Serverless Postgres
  - Qdrant Cloud (Free Tier)
- Documented chunking, embeddings, and retrieval pipeline
- Secure key management via environment variables

Code & Docs:
- TypeScript and Python preferred
- Clear repo structure and READ
```

## Response snapshot

I've successfully updated the project constitution based on your requirements. Here's the final summary:

## Constitution Update Summary

**New Version**: 1.1.0 (Minor version bump from 1.0.0)
**Bump Rationale**: Added new principles and sections specific to the AI-Spec–Driven Book with Embedded RAG Chatbot project

**Changes Made**:
- Updated project name to "AI-Spec–Driven Book with Embedded RAG Chatbot"
- Added 6 core principles reflecting your project requirements:
  1. Spec-First AI-Native Development
  2. Technical Accuracy and Correctness
  3. Developer-Focused Explanations
  4. Fully Reproducible from Repository
  5. Book Standards and Quality
  6. RAG Chatbot Integration Standards
- Added sections for Code & Documentation Standards and Technology Stack Requirements
- Updated Development Workflow and Quality Gates to match project requirements
- Set current date (2025-12-23) as both ratification and last amended dates

**Files Updated**:
- `.specify/memory/constitution.md` - Updated with new constitution content

**Templates Verification**:
- Checked plan-template.md, spec-template.md, and tasks-template.md for alignment
- No modifications needed to templates as they use general placeholders that remain compatible

**Suggested Commit Message**: `docs: amend constitution to v1.1.0 (AI book project principles and RAG chatbot standards)`

The updated constitution now fully reflects your project's core principles of spec-first AI-native development, technical accuracy, developer-focused explanations, and reproducibility, along with specific standards for the Docusaurus book and RAG chatbot implementation.

## Outcome

- ✅ Impact: Updated project constitution to reflect AI book project requirements
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with new principles
- 🔁 Next prompts: None needed
- 🧠 Reflection: Constitution now properly reflects the project's specific requirements for AI-native development, book publishing with Docusaurus, and RAG chatbot integration.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A