# Boid Simulation 
<div align="center">
  <img src="boids.png" width="400" height="400">
</div>

## Description
This Python script simulates the complex behavior of a flock of birds, known as "Boids", using a few simple rules: separation, alignment, and cohesion, along with wandering behavior. The simulation utilizes the Pygame library for visualization and runs in real-time, displaying Boids as they interact with each other according to the defined parameters and behaviors.
One can change the three different forces during simulation by adjusting the sliders.
<div align="center">
  
[![Boids moving](https://i.gyazo.com/f37a7d8c3102542ca7804a0fe605fd69.gif)](https://gyazo.com/f37a7d8c3102542ca7804a0fe605fd69)
</div>

## Dependencies
- Python 3.x
- Pygame
- Pygame GUI

## Installation
To run the simulation, you need to have Python installed along with the Pygame library. You can install Pygame using pip:

```
pip install pygame
pip install pygame_gui
```

## Usage
After installing the dependencies, you can run the simulation by executing the script in your terminal:

```
python boids.py
```

This will open a new window where you can see the Boids moving around the screen.

## Configuration
The script contains several parameters that you can tweak to change the behavior of the Boids:

- `NUM_BOIDS`: The number of Boids in the simulation.
- `BOID_SIZE`: The size of each Boid on the screen.
- `SPEED`: The speed at which Boids move.
- `MAX_FORCE`: The maximum force that can be applied to a Boid for it to change direction.
- `BOID_FRICTION`: The resistance the Boids experience, akin to friction, slowing them down over time.
- `WANDER_RADIUS`: The radius within which a Boid will wander randomly.
- `SEPARATION`: The degree to which Boids try to maintain distance from one another.
- `SEPARATION_RADIUS`: The radius within which Boids will begin to experience separation force.
- `ALIGNMENT`: The degree to which Boids try to align themselves with the average direction of nearby Boids.
- `ALIGNMENT_RADIUS`: The radius within which Boids will begin to experience alignment force.
- `COHESION`: The degree to which Boids try to move towards the average position of nearby Boids.
- `COHESION_RADIUS`: The radius within which Boids will begin to experience cohesion force.

Feel free to experiment with these values to observe different behaviors in the Boid flock.

## How it Works
- The script initializes a Pygame window and populates it with a number of Boid objects.
- Each Boid has properties that determine how it interacts with others:
  - **Separation**: Boids try to avoid crowding their neighbors.
  - **Alignment**: Boids try to align their direction with the average heading of their neighbors.
  - **Cohesion**: Boids try to move towards the average position of their neighbors.
- These forces are calculated for each Boid, and a resultant force changes the Boid's direction.
- A "wandering" behavior is also implemented to make the Boid's movement more natural and less uniform.
- Boids are drawn on the screen as simple triangles, the orientation of which depends on their current velocity.

## Key Components
- `class Simulation`: Handles the initialization and running of the simulation, managing events, and updating the Pygame screen.
- `class PhysicsObject`: An abstract class representing an object following simple physics laws.
- `class Boid(PhysicsObject)`: Represents a Boid, inheriting from PhysicsObject, and containing methods for each behavioral rule (separation, alignment, cohesion, and wandering).

## Exiting the Simulation
You can exit the simulation by closing the Pygame window or pressing Ctrl+C in the terminal.

## License
This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgements
- Craig Reynolds for the original concept of Boids.
- The course assistant in TDT4137 Kognitive arkitekturer that the setup.
