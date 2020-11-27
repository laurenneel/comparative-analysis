# Python Notes

# Debugging Notes

- step into
    - Allows you to dive into the internal workings of a given function
- step over
    - Allow you to execute the line and go to the next line (all the code gets ran)
- then green "continue" button
    - will jump to the next breakpoints
    - Click the "gutter" to add a breakpoint (red dot)
- step out 
    - Jumps out of a function if you step into it
    
# Git & GitHub

Terminology:

- Git = the version control program
- Github = the site where you store the versioned/saved code
- Commit = saving your file(s) at a specific instance in time
- Changes = things that you've changed
- Staged Changed = things you're about to commit (you must stage things before committing)

1) Initializing a new git repo for a _new_ project (ran once for each new project that you create)
```
git init # initializes an empty git repo on your hard drive
```

2) Hook up repository to Github account
```
Get the SSH URL from the repository (repo) - git@github.com:laurenneel/comparative-analysis.git
git remote add origin git@github.com:laurenneel/comparative-analysis.git

```