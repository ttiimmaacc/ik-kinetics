# IK Kinetics

A React Three Fiber-based demonstration of inverse kinematics (IK) using the FABRIK algorithm to simulate realistic leg movement.

## Overview

IK Kinetics is an interactive 3D visualization that demonstrates how inverse kinematics can be used to create natural-looking limb movements. The project uses the FABRIK (Forward And Backward Reaching Inverse Kinematics) algorithm with constraints, rest pose biasing, and automated stepping behavior.

![IK Kinetics Demo](public/demo-screenshot.png)

## Features

- **Interactive IK System**: Real-time response to target position changes
- **Constraint-based Movement**: Joint angle constraints for realistic motion
- **Rest Pose Biasing**: Limbs naturally return to a default pose when possible
- **Dynamic Stepping**: Automatic stepping when targets exceed maximum reach
- **3D Visualization**: Complete with joints, segments, and interactive controls
- **Ground Interaction**: Target positions automatically mapped to ground level

## Technical Implementation

The project implements several advanced techniques:

- **FABRIK Algorithm**: Custom implementation with iterative solving
- **Natural Constraints**: Joint limits prevent unnatural poses
- **Stable Orientation**: Special handling to avoid rotation artifacts
- **Raycasting**: Ground detection for accurate foot placement

## Getting Started

### Prerequisites

- Node.js (version 16 or higher)
- npm or yarn

### Installation

1. Clone the repository:
```bash
git clone [repository-url]
cd ik-kinetics
```

2. Install dependencies:
```bash
npm install
# or
yarn
```

3. Start the development server:
```bash
npm start
# or
yarn start
```

4. Open your browser to `http://localhost:3000`

## Usage

- **Moving the Body**: Use the transform controls (yellow gizmo) to move the body sphere
- **Leg Target**: The yellow sphere attached to the body determines where the leg tries to reach
- **Observing IK**: Watch how the leg segments (brown boxes) adjust automatically
- **Step Behavior**: When the target exceeds maximum reach, the foot will step to the new position

## Project Structure

- **IKDemo.jsx**: Main container component with 3D environment setup
- **LegWithIK.jsx**: Core implementation of the FABRIK algorithm and leg behavior
- **Ground.jsx**: Simple ground plane for interaction
- **SkeletonHelper.jsx**: Helper component for visualizing joints and connections

## Technologies

- [React](https://reactjs.org/) (v19.0.0)
- [Three.js](https://threejs.org/) (v0.160.0)
- [@react-three/fiber](https://github.com/pmndrs/react-three-fiber) (v9.0.4)
- [@react-three/drei](https://github.com/pmndrs/drei) (v10.0.1)
- [Leva](https://github.com/pmndrs/leva) (v0.10.0) for parameter controls

## How It Works

The FABRIK algorithm works by alternating between forward and backward passes through the kinematic chain:

1. **Forward Pass**: Starting from the target, each joint is positioned toward the root
2. **Backward Pass**: Starting from the root, each joint is positioned toward the target
3. **Constraints**: After each pass, constraints are applied to maintain natural poses
4. **Rest Pose Bias**: A subtle force pulls joints toward a predefined rest pose

The stepping behavior activates when the target is beyond the maximum reach of the leg, creating a simple yet effective locomotion system.

## License

[MIT License](LICENSE)

## Acknowledgments

- The FABRIK algorithm was first described by Andreas Aristidou and Joan Lasenby
- Thanks to the React Three Fiber community for their excellent 3D libraries
