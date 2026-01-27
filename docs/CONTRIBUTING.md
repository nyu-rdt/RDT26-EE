# Contributing to RDT26-EE

Guide for working on the rover firmware. If you're new to Git or GitHub, this document will walk you through everything you need.

---

## Getting Started

### First-Time Setup

1. **Install Git:** Download from [git-scm.com](https://git-scm.com/)

2. **Configure Git** (run once after installing):
   ```bash
   git config --global user.name "Your Name"
   git config --global user.email "your.email@nyu.edu"
   ```

3. **Clone the repository:**
   ```bash
   git clone https://github.com/nyu-rdt/RDT26-EE.git
   cd RDT26-EE
   ```

4. **Install VS Code extensions:**
   ```bash
   code --install-extension platformio.platformio-ide
   code --install-extension ms-vscode.cpptools
   code --install-extension eamodio.gitlens
   ```
   
   Optional but helpful: `Error Lens`, `Serial Monitor`, `GitHub Copilot`

---

## Git & GitHub Workflow

### Understanding Git vs GitHub

- **Git** = Version control software on your computer. Tracks changes to files.
- **GitHub** = Website that hosts Git repositories online. Where the team shares code.

### Key Concepts

| Term | What it means |
|------|---------------|
| **Repository (repo)** | The project folder with all its history |
| **Commit** | A saved snapshot of your changes |
| **Branch** | A separate line of development |
| **Push** | Upload your commits to GitHub |
| **Pull** | Download new commits from GitHub |
| **Pull Request (PR)** | A request to merge your branch into main |
| **Merge** | Combining one branch into another |

### Workflow

#### Step 1: Start with the latest code
```bash
git checkout main
git pull origin main
```

#### Step 2: Create a branch for your work
```bash
git checkout -b add-encoder-support
```
Name it something descriptive: `add-encoders`, `fix-can-timeout`, `test-motors`

#### Step 3: Make your changes
Edit files, write code, test it.

#### Step 4: Save your changes (commit)
```bash
git add .                                    # Stage all changed files
git commit -m "add encoder reading function" # Save with a message
```

Write commits that explain what you did:
```bash
# Good
git commit -m "add encoder support for wheel tracking"
git commit -m "fix motor direction for rear left"

# Bad
git commit -m "stuff"
git commit -m "asdfasdf"
```

#### Step 5: Push to GitHub
```bash
git push -u origin add-encoder-support
```

#### Step 6: Create a Pull Request
1. Go to GitHub.com → the repository
2. You'll see a banner saying "Compare & pull request" - click it
3. Add a title and brief description of what you changed
4. Click "Create pull request"
5. Ask a teammate to review it

#### Step 7: Merge after review
Once approved, click "Merge pull request" on GitHub.

### Common Situations

**"I need to save my work but it's not done yet"**
```bash
git commit -m "wip: still working on encoders"
```
Just clean up the message before merging.

**"I want to see what I've changed"**
```bash
git status        # See which files changed
git diff          # See the actual changes
```

**"I made a mistake in my last commit message"**
```bash
git commit --amend -m "new message here"
```

**"I need to undo my uncommitted changes"**
```bash
git checkout -- filename    # Undo changes to one file
git checkout -- .           # Undo all changes (careful!)
```

**"Someone else pushed changes and now I can't push"**
```bash
git pull origin main        # Get their changes first
# Fix any conflicts if there are any
git push
```

### The Golden Rule

**Pull before you start working.** Run `git pull` at the beginning of each session to avoid conflicts.

---

## Code Organization

```
RDT_2025_2026_TEENSY4_1/
├── include/          # Header files (.h)
├── src/              # Implementation (.cpp)
├── lib/              # External libraries
└── test/             # Test code (see testing.md)
```

Keep pin definitions in `config.h`. If you wire something new, add it there.

---

## Testing on Hardware

Before merging code that controls motors or actuators:
1. Test it on the actual hardware
2. Make sure E-Stop still works
3. Have someone else present when testing motors for the first time

See [testing.md](testing.md) for how to write and run tests.

---

## PlatformIO Commands

```bash
pio run                  # Build the code
pio run -t upload        # Build and upload to Teensy
pio device monitor       # Open serial monitor
pio test                 # Run tests
pio run -t clean         # Clean build files
```

---

## Getting Help

- **Stuck on Git?** Ask a teammate or check [git-scm.com/doc](https://git-scm.com/doc)
- **Code questions?** Post in Slack or comment on the relevant PR
- **Hardware issues?** Talk to the electrical leads

Don't be afraid to ask questions - everyone was new at some point!
