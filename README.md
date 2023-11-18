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

* **Extended Kalman Filter:** `0` (default)
* **Implicit Extended Kalman Filter:** `1`

## Run

`./KalmanFilterBallTracking [ENVIRONMENT] [KALMAN_FILTER_TYPE];`

## Examples:

Tennis ball with Implicit Extended Kalman Filter:


```sh
cmake -DCMAKE_BUILD_TYPE=Debug -S ./cmake-build-debug2
```

```sh
./kalman_filter_ball_tracking ball_tennis 1;
```

Orange ball with Extended Kalman Filter:

```sh
./KalmanFilterBallTracking ball_orange 0;
```

## Installation

This project depends on the following libraries:

* https://github.com/arturoemx/Circle
* https://github.com/arturoemx/Curvature

The following command downloads the necessary libraries to the `lib` folder:

```sh
sh download_libraries.sh
```

## Analysis and Matrices

* [Extended Kalman Filter](doc/ball_tracking_kalman_filter_extended.ipynb)
* [Implicit Extended Kalman Filter](doc/ball_tracking_kalman_filter_extended_implicit.ipynb)

## Colaborators

* [Arturo Espinosa](https://github.com/arturoemx)
* [Dayan Bravo](https://github.com/dayan3847)
* [Mario Herrera](https://github.com/mario-infor)
