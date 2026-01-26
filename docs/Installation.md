# Installation

Rerun support requires Apache Arrow to be installed first.

## macOS

On macOS we prefer the system Arrow because it builds much faster than
building Arrow from source.

Install Arrow with Homebrew:

```sh
brew install apache-arrow
```

If CMake cannot find Arrow, set `Arrow_DIR`:

```sh
export Arrow_DIR="$(brew --prefix apache-arrow)/lib/cmake/Arrow"
```

You can force building Arrow from source by configuring with:

```sh
cmake -S . -B build -DRERUN_DOWNLOAD_AND_BUILD_ARROW=ON
```
