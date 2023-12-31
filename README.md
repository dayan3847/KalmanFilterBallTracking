# Bayes Estimator

<img src="https://uady.mx/assets/img/logo_uady.svg" width="15%" alt="uady">

### <font color='orange'>Universidad Autónoma de Yucatán</font> _Facultad de Matemáticas_

**Teacher:** Dr. Arturo Espinosa Romero <[eromero@correo.uady.mx](mailto:eromero@correo.uady.mx)>

**Student:** Ing. Dayan Bravo Fraga <[dayan3847@gmail.com](mailto:dayan3847@gmail.com)>

# Ball Tracking *Kalman Filter*

## Legend

<img alt="ball_green" src="doc/ex_ball_green.png" width="45%"/>

* Blue: **Measurement with `findContours` and *RANSAC*
* Green: Correction with *Kalman Filter*

## Environments

* **Tennis ball:** `ball_tennis` (default)
* **Orange ball:** `ball_orange`
* **Red ball:** `ball_red`
* **Green ball:** `ball_green` (in legend)

<img alt="ball_green" src="doc/ex_ball_tennis.png" width="30%"/><img alt="ball_green" src="doc/ex_ball_orange.png" width="30%"/><img alt="ball_green" src="doc/ex_ball_red.png" width="30%"/>

## Kalman Filter Types

* **Extended_6x5 Kalman Filter:** `0` (default)
* **Implicit Extended_6x5 Kalman Filter:** `1`

## Installation

This project depends on the following library:

### Circle: (by Arturo Espinosa)

* https://github.com/arturoemx/Circle

### Clone

```sh
git clone https://github.com/arturoemx/Circle lib/arturoemx/Circle
```

### Note:

Last
commit: [1b3e61fd0203164265b804276546465898eaa3b7](https://github.com/arturoemx/Circle/commit/1b3e61fd0203164265b804276546465898eaa3b7)

## Install with CMake

### Requirements (Ubuntu 20.04)

#### [CMake](https://cmake.org/)

```sh
sudo apt install cmake -y
```

Check:

```sh
cmake --version
```

#### [Ninja Build](https://ninja-build.org/)

```sh
sudo apt install ninja-build -y
```

You can see: https://lindevs.com/install-ninja-build-system-on-ubuntu/

Check:

```sh
ninja --version
```

### Reload CMake Project

```sh
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_MAKE_PROGRAM=ninja -G Ninja -S ./ -B ./cmake-build-debug
```

### Build

```sh
cmake --build ./cmake-build-debug --target KalmanFilterBallTracking -j 6
```

## Install with Make (by Arturo Espinosa)

```sh
make
```

You can see: [Makefile](Makefile)

## Run

`./KalmanFilterBallTracking [ENVIRONMENT] [KALMAN_FILTER_TYPE];`

## Examples:

Tennis ball with Implicit Extended_6x5 Kalman Filter:

```sh
./cmake-build-debug/KalmanFilterBallTracking ball_tennis 1;
```

Orange ball with Extended_6x5 Kalman Filter:

```sh
./cmake-build-debug/KalmanFilterBallTracking ball_orange 0;
```

## Analysis and Matrices

### Derivatives Analysis

* [Position](doc/analysis/analysis_x_velocity_acceleration.ipynb) [<img alt="Open In Colab" src="https://colab.research.google.com/assets/colab-badge.svg" width="80"/>](https://colab.research.google.com/github/dayan3847/KalmanFilterBallTracking/blob/master/doc/analysis/analysis_x_velocity_acceleration.ipynb)
* [Radium](doc/analysis/analysis_r_velocity_acceleration.ipynb) [<img alt="Open In Colab" src="https://colab.research.google.com/assets/colab-badge.svg" width="80"/>](https://colab.research.google.com/github/dayan3847/KalmanFilterBallTracking/blob/master/doc/analysis/analysis_r_velocity_acceleration.ipynb)

### Matrices

* [Extended_6x5 Kalman Filter](doc/analysis/ball_tracking_kalman_filter_extended.ipynb) [<img alt="Open In Colab" src="https://colab.research.google.com/assets/colab-badge.svg" width="80"/>](https://colab.research.google.com/github/dayan3847/KalmanFilterBallTracking/blob/master/doc/analysis/ball_tracking_kalman_filter_extended.ipynb)
* [Implicit Extended_6x5 Kalman Filter](doc/analysis/ball_tracking_kalman_filter_extended_implicit.ipynb) [<img alt="Open In Colab" src="https://colab.research.google.com/assets/colab-badge.svg" width="80"/>](https://colab.research.google.com/github/dayan3847/KalmanFilterBallTracking/blob/master/doc/analysis/ball_tracking_kalman_filter_extended_implicit.ipynb)

## Colaborators

* [Arturo Espinosa](https://github.com/arturoemx)
* [Dayan Bravo](https://github.com/dayan3847)
* [Mario Herrera](https://github.com/mario-infor)
