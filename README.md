# safe-robots

## Setup
### [Pre-commit](https://pre-commit.com/)
Git hook scripts are useful for identifying simple issues before submission to code review. Hooks are run on every commit to automatically point out issues in code. Pre-commit is a multi-language package manager for pre-commit hooks. You specify a list of hooks you want and pre-commit manages the installation and execution of any hook written in any language before every commit. The first time pre-commit runs on a file it will automatically download, install, and run the hook.

#### Installation
Before you can run hooks, you need to have the pre-commit package manager installed. Using pip:
```terminal
$ pip install pre-commit
```
#### Updating hooks automatically
You can update your hooks to the latest version automatically by running:

```terminal
$ pre-commit autoupdate
```
By default, this will bring the hooks to the latest tag on the default branch.

#### Usage
- To install pre-commit into your git hooks
```terminal
$ pre-commit install
```
- To manually run all pre-commit hooks on a repository:
```terminal
$ run pre-commit run --all-files
or
$ run pre-commit run --all-files <hook_id>
```

### [Compiledb](https://github.com/nickdiego/compiledb)
Tool for generating Clang's JSON Compilation Database file for GNU make-based build systems.
#### Installation
```terminal
$ pip install compiledb
```

### [Cpplint](https://github.com/cpplint/cpplint)
Cpplint is a command-line tool to check C/C++ files for style issues following Google's C++ style guide. Cpplint is developed and maintained by Google Inc. at google/styleguide, also see the wikipedia entry
#### Installation
```terminal
$ pip install cpplint
```
