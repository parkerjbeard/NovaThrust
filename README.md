Welcome to NovaThrust!

NovaThrust is a cutting-edge simulation toolkit engineered for the next generation of aerospace applications. Developed with precision and performance in mind, it integrates state-of-the-art computational techniques and mathematical models to tackle some of the most challenging problems in aerospace engineering. From the dynamics of new propulsion systems to the aerodynamics of high-speed flight, NovaThrust is your gateway to exploring and innovating in the field of aerospace technology.

## Features

- **Multi-Tube PDE Simulation**: Dive into the world of advanced propulsion with our multi-tube pulse detonation engine simulations. NovaThrust models the intricate wave dynamics and energy transformations within these engines, providing insights that help in optimizing engine design and performance.

- **PDE Flight Simulation**: Utilize partial differential equations to simulate and study the behavior of aircraft under various flight conditions. Our tools allow you to adjust parameters and observe the effects on flight dynamics, offering a powerful method for enhancing aircraft design and safety.

- **Performance Metrics**: With NovaThrust, not only do you get to simulate, but you also get to measure. Our comprehensive suite of performance metrics allows you to evaluate every aspect of your simulation, from computational speed to accuracy, ensuring that you have all the data you need to make informed decisions.

## Features

- **Multi-Tube PDE Simulation**: Simulate the complex interactions in multi-tube pulse detonation engines.
- **PDE Flight Simulation**: Explore the dynamics of flight using partial differential equations.
- **Performance Metrics**: Evaluate the performance of simulations with detailed metrics.

## Prerequisites

Before you can build and run NovaThrust, you need to have the following installed on your system:
- CMake 3.10 or higher
- A C++ compiler supporting C++17 standard
- OpenBLAS
- FFTW (The Fastest Fourier Transform in the West)
- Eigen3 (version 3.3 or higher)

## Installation

1. **Clone the repository:**
   ```
   git clone https://github.com/parkerjbeard/NovaThrust.git
   ```
2. **Navigate to the cloned directory:**
   ```
   cd NovaThrust
   ```

3. **Configure the project with CMake:**
   ```
   cmake .
   ```

4. **Build the project:**
   ```
   cmake --build .
   ```

## Usage

After building the project, you can run the provided examples:
- `./NovaThrust`
- `./MultiTubePDESimulation`
- `./PDEFlightSimulation`
- `./PerformanceMetricsExample`

## Customization

You can set the `OpenBLAS_HOME` environment variable to specify the installation path of OpenBLAS if it is not in the default location.

## Contributing

Contributions to NovaThrust are welcome! Please refer to the contributing guidelines for more information on how to submit pull requests, report issues, and so on.

## License

NovaThrust is released under the MIT License. See the LICENSE file for more details.

## Acknowledgments

This project utilizes the following open-source libraries:
- OpenBLAS
- FFTW
- Eigen

Thank you to all the contributors who help in maintaining and improving these libraries.

Enjoy using NovaThrust for your simulation needs!

## Roadmap

Looking ahead, NovaThrust is committed to expanding its capabilities and offering more advanced features to enhance user experience and simulation accuracy. Here are some of the exciting developments we have planned:

- **More Types of Rockets**: We plan to include simulations for a broader range of rocket types, including liquid, solid, and hybrid propulsion systems. This will allow users to explore and compare different rocket technologies under various operational conditions.

- **Hardware Connectivity**: Future versions will support hardware-in-the-loop (HIL) simulations, enabling users to connect NovaThrust with actual control hardware. This feature aims to provide a more realistic simulation environment for testing and validation of control strategies.

- **Python Bindings and Enhanced Visualization**: To make NovaThrust more accessible and versatile, we will introduce Python bindings. This will allow users to script simulations and analyses in Python. Additionally, these bindings will support enhanced visualization tools, making it easier to interpret simulation results and perform complex analyses.

These features are designed to make NovaThrust a more powerful tool in the arsenal of aerospace engineers and researchers, pushing the boundaries of what can be achieved in aerospace simulation.

Enjoy using NovaThrust for your simulation needs!