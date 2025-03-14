import React, { useRef, useState, useEffect, useMemo } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import { TransformControls } from "@react-three/drei";
import * as THREE from "three";
import { FABRIK } from "./FABRIK";

const LEG_CONFIGURATIONS = [
  // Front left
  {
    id: "frontLeft",
    bodyOffset: new THREE.Vector3(-0.4, 0, -0.4),
    targetOffset: new THREE.Vector3(-1.5, 0, -0.8),
    initialJoints: [
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(-1, 0.5, 0),
      new THREE.Vector3(-2.5, 0, 0),
      new THREE.Vector3(-3.5, 0, 0),
    ],
    color: "#331100",
    phaseOffset: 0, // First diagonal pair
  },
  // Front right
  {
    id: "frontRight",
    bodyOffset: new THREE.Vector3(0.4, 0, -0.4),
    targetOffset: new THREE.Vector3(1.5, 0, -0.8),
    initialJoints: [
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(1, 0.5, 0),
      new THREE.Vector3(2.5, 0, 0),
      new THREE.Vector3(3.5, 0, 0),
    ],
    color: "#442200",
    phaseOffset: 0.5, // Second diagonal pair
  },
  // Back left
  {
    id: "backLeft",
    bodyOffset: new THREE.Vector3(-0.4, 0, 0.4),
    targetOffset: new THREE.Vector3(-1.5, 0, 0.8),
    initialJoints: [
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(-1, 0.5, 0),
      new THREE.Vector3(-2.5, 0, 0),
      new THREE.Vector3(-3.5, 0, 0),
    ],
    color: "#331100",
    phaseOffset: 0.5, // Second diagonal pair
  },
  // Back right
  {
    id: "backRight",
    bodyOffset: new THREE.Vector3(0.4, 0, 0.4),
    targetOffset: new THREE.Vector3(1.5, 0, 0.8),
    initialJoints: [
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(1, 0.5, 0),
      new THREE.Vector3(2.5, 0, 0),
      new THREE.Vector3(3.5, 0, 0),
    ],
    color: "#442200",
    phaseOffset: 0, // First diagonal pair
  },
];

const SpiderLeg = ({ 
  bodyRef, 
  legConfig, 
  scene, 
  isMoving, 
  targetDirection,
  movementPhase,
  groundDetector
}) => {
  const segmentRefs = useRef(Array(3).fill().map(() => React.createRef()));
  const jointRefs = useRef(Array(4).fill().map(() => React.createRef()));
  const footTargetRef = useRef();
  const footPositionRef = useRef(new THREE.Vector3(0, 0, 0));
  const stepProgressRef = useRef(0);
  const isSteppingRef = useRef(false);
  const distanceLineRef = useRef();
  const sphereRef = useRef();
  const fabrikSolver = useRef(null);

  const legData = useMemo(() => ({
    joints: Array(4).fill().map(() => new THREE.Vector3()),
    segmentLengths: [0.8, 1, 0.9],
    targetPos: new THREE.Vector3(),
    maxStretch: 1.8,
    stepDuration: 10,
    bodyOffset: legConfig.bodyOffset,
  }), [legConfig]);

  useEffect(() => {
    // Initialize FABRIK solver with the configuration for this leg
    fabrikSolver.current = new FABRIK(legConfig.initialJoints, legData.segmentLengths);
    
    // Initialize foot position
    footPositionRef.current.copy(legConfig.initialJoints[3]);
  }, [legConfig, legData.segmentLengths]);

  // Main function to calculate leg positions based on FABRIK solution
  const updateLegPositions = (basePos, footPos) => {
    if (!fabrikSolver.current) return;

    // Set the root position
    fabrikSolver.current.joints[0].copy(basePos);

    // Also update the rest pose root position since it's attached to the body
    fabrikSolver.current.restPose[0].copy(basePos);

    // Calculate vector from body to target
    const toTarget = new THREE.Vector3().subVectors(footPos, basePos);
    const distanceToTarget = toTarget.length();

    // Adjust rest pose based on target direction for a more natural reach
    if (distanceToTarget > 0.1) {
      const direction = toTarget.clone().normalize();

      // Generate a rest pose that points in the target direction
      const restDirection = direction.clone().multiplyScalar(0.7);

      // Knee joint - position based on direction but raised
      const kneePos = basePos
        .clone()
        .add(restDirection.clone().multiplyScalar(legData.segmentLengths[0]));
      kneePos.y += 0.5; // Raise knee joint
      fabrikSolver.current.restPose[1].copy(kneePos);

      // Ankle position
      const anklePos = kneePos
        .clone()
        .add(restDirection.clone().multiplyScalar(legData.segmentLengths[1]));
      fabrikSolver.current.restPose[2].copy(anklePos);

      // Foot position (end effector)
      const footRestPos = anklePos
        .clone()
        .add(restDirection.clone().multiplyScalar(legData.segmentLengths[2]));
      fabrikSolver.current.restPose[3].copy(footRestPos);
    }

    // Solve IK to reach foot position
    fabrikSolver.current.solve(footPos);

    // Copy joint positions to our data for visualization
    for (let i = 0; i < fabrikSolver.current.joints.length; i++) {
      legData.joints[i].copy(fabrikSolver.current.joints[i]);
    }
  };

  // Stable orientation function that prevents y-axis flipping
  function stableOrient(object, startPoint, endPoint) {
    const direction = new THREE.Vector3()
      .subVectors(endPoint, startPoint)
      .normalize();

    // Calculate the yaw (horizontal angle around Y axis)
    const yaw = Math.atan2(direction.x, direction.z);

    // Calculate the pitch (vertical angle)
    const horizontalLength = Math.sqrt(
      direction.x * direction.x + direction.z * direction.z
    );
    const pitch = -Math.atan2(direction.y, horizontalLength);

    // Set rotation using Euler angles with specific order
    const euler = new THREE.Euler(pitch, yaw, 0, "YXZ");
    object.quaternion.setFromEuler(euler);
  }

  useFrame(() => {
    if (!bodyRef.current || !footTargetRef.current) return;

    const body = bodyRef.current;
    const target = footTargetRef.current;

    // Get target position in world space
    const targetWorldPos = new THREE.Vector3();
    bodyRef.current.getWorldPosition(targetWorldPos);
    
    // Add the target offset to get the desired foot target position
    targetWorldPos.add(legConfig.targetOffset);

    // Project the target onto the ground using raycasting
    const groundPoint = groundDetector.getGroundPoint(targetWorldPos);
    if (groundPoint) {
      legData.targetPos.copy(groundPoint);
      
      // Update target visualization
      if (sphereRef.current) {
        sphereRef.current.position.copy(legData.targetPos);
      }

      // Update distance line visualization
      if (distanceLineRef.current) {
        const points = [footPositionRef.current, legData.targetPos];
        distanceLineRef.current.geometry.setFromPoints(points);
      }

      // Calculate distance to target
      const distanceToTarget = footPositionRef.current.distanceTo(legData.targetPos);

      // Determine if this leg should be stepping based on movement phase and phase offset
      const shouldStep = isMoving && 
        (Math.sin(2 * Math.PI * (movementPhase + legConfig.phaseOffset)) > 0.5) && 
        !isSteppingRef.current;

      // Stepping logic when distance exceeds max stretch or based on movement phase
      if (distanceToTarget > legData.maxStretch || isSteppingRef.current || shouldStep) {
        if (!isSteppingRef.current) {
          isSteppingRef.current = true;
          stepProgressRef.current = 0;
        }

        stepProgressRef.current++;
        if (stepProgressRef.current >= legData.stepDuration) {
          isSteppingRef.current = false;
          footPositionRef.current.copy(legData.targetPos);
        } else {
          // Add a bit of height to the step trajectory
          const t = stepProgressRef.current / legData.stepDuration;
          const stepHeight = Math.sin(t * Math.PI) * 0.5; // Add arc to step
          
          // Interpolate from current foot position to target
          const targetPos = new THREE.Vector3().lerpVectors(
            footPositionRef.current,
            legData.targetPos,
            t
          );
          
          // Add height to create an arc
          targetPos.y += stepHeight;
          footPositionRef.current.copy(targetPos);
        }
      }

      // Get body position and add offset for this specific leg
      const basePos = new THREE.Vector3();
      body.getWorldPosition(basePos);
      basePos.add(legData.bodyOffset);

      // Calculate leg positions based on FABRIK
      updateLegPositions(basePos, footPositionRef.current);

      // Update joint spheres for visualization
      jointRefs.current.forEach((ref, i) => {
        if (ref.current) {
          ref.current.position.copy(legData.joints[i]);
        }
      });

      // Update the leg segments
      segmentRefs.current.forEach((ref, i) => {
        if (ref.current) {
          const start = legData.joints[i];
          const end = legData.joints[i + 1];

          // Position segment at midpoint between joints
          const midpoint = new THREE.Vector3().lerpVectors(start, end, 0.5);
          ref.current.position.copy(midpoint);

          // Calculate segment length
          const length = start.distanceTo(end);

          // Use stable orientation for all segments
          stableOrient(ref.current, start, end);

          // Set scale - only change length, not width/height
          ref.current.scale.set(0.2, 0.2, length);
        }
      });
    }
  });

  return (
    <group>
      {/* Foot target (invisible) */}
      <mesh ref={footTargetRef} position={legConfig.targetOffset}>
        <sphereGeometry args={[0.1]} />
        <meshStandardMaterial transparent opacity={0} />
      </mesh>

      {/* Target visualization sphere */}
      <mesh ref={sphereRef}>
        <sphereGeometry args={[0.1]} />
        <meshStandardMaterial color="blue" transparent opacity={0.3} />
      </mesh>

      {/* Joint visualizations */}
      {jointRefs.current.map((ref, i) => (
        <mesh key={`joint-${legConfig.id}-${i}`} ref={ref} castShadow>
          <sphereGeometry args={[0.1]} />
          <meshStandardMaterial
            color={
              i === 0
                ? "#ff0000" // Hip - red
                : i === 3
                ? "#00ff00" // Foot - green
                : "#ffaa00" // Middle joints - orange
            }
          />
        </mesh>
      ))}

      {/* Leg segments */}
      {segmentRefs.current.map((ref, i) => (
        <mesh key={`segment-${legConfig.id}-${i}`} ref={ref} castShadow>
          <boxGeometry args={[1, 1, 1]} />
          <meshStandardMaterial color={legConfig.color} />
        </mesh>
      ))}

      {/* Distance visualization line */}
      <line ref={distanceLineRef}>
        <bufferGeometry />
        <lineBasicMaterial color="red" />
      </line>
    </group>
  );
};

// Create a ground detector utility
class GroundDetector {
  constructor(scene) {
    this.scene = scene;
  }

  getGroundPoint(position, direction = new THREE.Vector3(0, -1, 0)) {
    const raycaster = new THREE.Raycaster(
      new THREE.Vector3(position.x, position.y + 0.5, position.z), 
      direction
    );
    
    const intersects = raycaster.intersectObjects(
      this.scene.children.filter(
        (child) => child.name && child.name.startsWith("ground")
      )
    );

    if (intersects.length > 0) {
      return intersects[0].point;
    }
    
    return null;
  }
}

const SpiderController = () => {
  const { scene } = useThree();
  const bodyRef = useRef();
  const targetGizmoRef = useRef();
  const [isAutoWalking, setIsAutoWalking] = useState(false);
  const bodyHeightOffsetRef = useRef(1.0);
  const bodyMovementSpeedRef = useRef(0.06);
  const prevTargetPositionRef = useRef(new THREE.Vector3());
  const isTargetMovingRef = useRef(false);
  const moveBodyToTargetRef = useRef(false);
  const movementPhaseRef = useRef(0);
  const groundDetector = useMemo(() => new GroundDetector(scene), [scene]);

  // Check if target gizmo has stopped moving
  const checkTargetMovement = () => {
    if (!targetGizmoRef.current) return;

    const targetPosition = new THREE.Vector3();
    targetGizmoRef.current.getWorldPosition(targetPosition);

    // If position changed
    if (!targetPosition.equals(prevTargetPositionRef.current)) {
      isTargetMovingRef.current = true;
      prevTargetPositionRef.current.copy(targetPosition);
    }
    // If position stopped changing
    else if (isTargetMovingRef.current) {
      isTargetMovingRef.current = false;
      moveBodyToTargetRef.current = true; // Start moving body
    }
  };

  // Move body toward target
  const moveBodyTowardTarget = () => {
    if (!moveBodyToTargetRef.current || !bodyRef.current || !targetGizmoRef.current)
      return false;

    const bodyPosition = new THREE.Vector3();
    bodyRef.current.getWorldPosition(bodyPosition);

    const targetPosition = new THREE.Vector3();
    targetGizmoRef.current.getWorldPosition(targetPosition);

    // Calculate direction on XZ plane only
    const direction = new THREE.Vector3(
      targetPosition.x - bodyPosition.x,
      0,
      targetPosition.z - bodyPosition.z
    );

    // Check if we've reached the target
    const distanceToTarget = direction.length();
    if (distanceToTarget < 0.1) {
      moveBodyToTargetRef.current = false;
      return false;
    }

    // Move toward target
    direction.normalize().multiplyScalar(bodyMovementSpeedRef.current);
    bodyRef.current.position.x += direction.x;
    bodyRef.current.position.z += direction.z;
    return true;
  };

  // Adapt body height to terrain
  const adaptBodyToTerrain = () => {
    if (!bodyRef.current) return;

    const bodyPosition = new THREE.Vector3();
    bodyRef.current.getWorldPosition(bodyPosition);

    // Get ground height at body position
    const groundPoint = groundDetector.getGroundPoint(
      new THREE.Vector3(bodyPosition.x, bodyPosition.y + 5, bodyPosition.z)
    );
    
    if (groundPoint) {
      const groundHeight = groundPoint.y;
      const targetHeight = groundHeight + bodyHeightOffsetRef.current;

      // Smoothly adjust height
      bodyRef.current.position.y = THREE.MathUtils.lerp(
        bodyRef.current.position.y,
        targetHeight,
        0.1
      );
    }
  };

  useFrame(() => {
    // Update target movement detection
    checkTargetMovement();
    
    // Move body if needed and update isMoving state
    const isMoving = moveBodyTowardTarget();
    
    // Update body height based on terrain
    adaptBodyToTerrain();
    
    // Update movement phase for coordinated leg stepping
    if (isMoving) {
      movementPhaseRef.current = (movementPhaseRef.current + 0.01) % 1.0;
    }
  });

  return (
    <group>
      {/* Main target control */}
      <TransformControls object={targetGizmoRef} mode="translate" size={0.5}>
        <mesh ref={targetGizmoRef} position={[0, 1.9, 2]}>
          <sphereGeometry args={[0.2]} />
          <meshStandardMaterial color="yellow" transparent opacity={0.7} />
        </mesh>
      </TransformControls>

      {/* Spider body */}
      <mesh ref={bodyRef} position={[0, 1.9, 0]} castShadow>
        <boxGeometry args={[0.8, 0.5, 1.6]} />
        <meshStandardMaterial color="#8B4513" />
      </mesh>

      {/* Spider legs */}
      {LEG_CONFIGURATIONS.map(legConfig => (
        <SpiderLeg
          key={legConfig.id}
          bodyRef={bodyRef}
          legConfig={legConfig}
          scene={scene}
          isMoving={moveBodyToTargetRef.current}
          targetDirection={new THREE.Vector3()}
          movementPhase={movementPhaseRef.current}
          groundDetector={groundDetector}
        />
      ))}
    </group>
  );
};

export default SpiderController;