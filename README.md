# ros1-pkg-template
This is the template repository for the ROS1 package with [Industrial CI](https://github.com/ros-industrial/industrial_ci).
## Before use
Adjust the `ROS_DISTRO` tag in [ci.yml](https://github.com/Alpaca-zip/ros1-pkg-template/blob/main/.github/workflows/ci.yml) to suit your ROS environment:
```yaml
jobs:
  clang_format_check:
    runs-on: ubuntu-latest
    steps:
      - name: checkout
        uses: actions/checkout@v2
        with:
          ref: ${{ github.head_ref }}
      - name: run industrial_ci
        uses: 'ros-industrial/industrial_ci@master'
        env:
          ROS_DISTRO: noetic # Replace noetic for your chosen distro.
          CLANG_FORMAT_CHECK: file
          CLANG_FORMAT_VERSION: "10"
```
## Industrial CI
This template provides a comprehensive CI pipeline for ROS project using GitHub Actions and the ros-industrial/industrial_ci action.
- `Clang Format Check` : Ensures code formatting adheres to defined standards using [clang-format](https://clang.llvm.org/docs/ClangFormat.html).
- `Clang Tidy Check` : Validates code for potential issues and adherence to coding standards using [Clang-Tidy](https://clang.llvm.org/extra/clang-tidy/).
- `Black Check` : Ensures code formatting adheres to PEP standards using [Black](https://black.readthedocs.io/en/stable/).
- `Pylint Check` : Runs [Pylint](https://pylint.readthedocs.io/en/stable/) for static code analysis on Python code.
- `Catkin Lint Check` : Validates our ROS package structure and dependencies with [catkin_lint](https://fkie.github.io/catkin_lint/).
- `Build Check` : Performs a build for both main and testing ROS repositories to ensure the codebase builds correctly.
## To pass all checks...
### clang-format
**1. install**
```
$ sudo apt-get install clang-format
```
**2. Format with clang-format**
```
$ roscd <this_package>
$ find . -type f \( -name "*.cpp" -or -name "*.hpp" -or -name "*.h" \) -exec clang-format -i -style=file {} \;
```
### Clang-Tidy
**1. install**
```
$ sudo apt-get install clang clang-tidy
```
**2. Format with Clang-Tidy**
```
$ cd <path_to_your_workspace> && catkin build <this_package> --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
$ clang-tidy -p build/<this_package>/ <path_to_this_package>/src/<your_cpp> -fix
```
  Recommended tool : [catkin_tidy](https://github.com/nyxrobotics/catkin_tidy)
### Black
**1. install**
```
$ pip install black
```
**2. Format with Black**
```
$ roscd <this_package>
$ find . -type f -name "*.py" -exec black {} \;
```
### Pylint
**1. install**
```
$ pip install pylint
```
**2. Run Pylint**
```
$ roscd <this_package>
$ find . -type f -name "*.py" -exec pylint {} \;
```
### catkin_lint
**1. install**
```
$ sudo apt-get install catkin-lint
```
**2. Run catkin_lint**
```
$ catkin_lint --pkg <this_package>
```
