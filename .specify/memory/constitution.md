<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.1.0
Modified principles:
- Spec-First AI-Native Development
- Technical Accuracy and Correctness
- Developer-Focused Explanations
- Reproducible Repository
Added sections: Book Standards, RAG Chatbot Standards, Code & Docs Standards
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ⚠ pending
Follow-up TODOs: None
-->

# AI-Spec–Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-First AI-Native Development
All development must follow a specification-first approach using Spec-Kit Plus and Claude Code; Every feature and component must be defined in specifications before implementation; AI tools must be leveraged for code generation, review, and testing throughout the development lifecycle.

### Technical Accuracy and Correctness
All content and code examples must be technically accurate and verified; Code examples must be runnable or clearly labeled as illustrative; All technical claims must be verifiable and backed by appropriate sources or testing.

### Developer-Focused Explanations
All content must be written with developers as the primary audience; Explanations must be clear, practical, and include real-world use cases; Documentation must include sufficient context for developers to understand and apply concepts.

### Fully Reproducible from Repository
All book content, build processes, and deployment configurations must be fully reproducible from the repository; Anyone with access to the repository must be able to build and deploy the same site; Development and deployment environments must be clearly documented and containerized where appropriate.

### Book Standards and Quality
Content must be built using Docusaurus framework and deployed to GitHub Pages; Chapters must be modular and driven by Spec-Kit Plus specifications; Mermaid diagrams must be used where they enhance understanding of complex concepts.

### RAG Chatbot Integration Standards
The embedded RAG chatbot must support both full-book and user-selected text Q&A; The stack must use OpenAI Agents/ChatKit SDK, FastAPI backend, Neon Serverless Postgres, and Qdrant Cloud; The chunking, embeddings, and retrieval pipeline must be fully documented with secure key management.

## Additional Standards

### Code and Documentation Standards
- TypeScript and Python are the preferred languages for all implementation
- Clear repository structure with comprehensive README documentation
- All code must include proper error handling and security considerations
- Environment variables must be used for secure key management

### Technology Stack Requirements
- Book Framework: Docusaurus
- Deployment: GitHub Pages
- Chatbot Backend: FastAPI
- Database: Neon Serverless Postgres
- Vector Storage: Qdrant Cloud (Free Tier)
- AI Integration: OpenAI Agents/ChatKit SDK

## Development Workflow

### Implementation Process
- All features must start with specification in Spec-Kit Plus
- Claude Code must be used for AI-assisted development
- Code examples must be tested and verified before inclusion
- All changes must maintain full reproducibility from the repository

### Quality Gates
- All code examples must be runnable or clearly labeled
- Technical accuracy must be verified by at least one additional reviewer
- Deployment process must be documented and tested
- Security considerations must be addressed for all external integrations

## Governance

All development must adhere to the specified technology stack and architectural decisions; Changes to core principles require explicit approval and documentation; All pull requests must verify compliance with both technical and content standards; The constitution supersedes all other development practices and guidelines.

**Version**: 1.1.0 | **Ratified**: 2025-12-23 | **Last Amended**: 2025-12-23