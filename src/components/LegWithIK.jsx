import React, { useRef, useMemo, useEffect } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import { TransformControls } from "@react-three/drei";
import * as THREE from "three";

class FABRIK {
  constructor(joints, lengths) {
    this.joints = joints.map((j) => new THREE.Vector3().copy(j));
    this.originalJoints = joints.map((j) => new THREE.Vector3().copy(j));
    this.restPose = joints.map((j) => new THREE.Vector3().copy(j));
    this.lengths = lengths;
    this.totalLength = lengths.reduce((sum, length) => sum + length, 0);

    // Define angle constraints for each joint (in radians)
    this.constraints = [
      { min: -Math.PI / 12, max: Math.PI / 1 }, // <-- Hip joint
      { min: -Math.PI / 12, max: Math.PI / 3 }, // <-- Knee joint
      { min: -Math.PI / 0, max: Math.PI / 2 }, // <-- Ankle joint
    ];

    // Rest pose bias strength (0-1)
    // Higher values make the leg return to rest pose more strongly
    this.restPoseBias = 0.1;
  }

  // Apply constraints to angle between segments
  applyConstraints() {
    for (let i = 1; i < this.joints.length - 1; i++) {// <-- loop through each joint (except the first and last)
      const prev = this.joints[i - 1];
      const current = this.joints[i]; // <-- For each joint, we look at the previous, current & next joints
      const next = this.joints[i + 1];

      // Get vectors representing the segments
      const v1 = new THREE.Vector3().subVectors(current, prev); // <-- vector from previous to current joint
      const v2 = new THREE.Vector3().subVectors(next, current); // <-- vector from current to next joint

      // Calculate angle between segments
      const angle = v1.angleTo(v2);

      // If angle is within constraints, continue
      if (
        angle >= this.constraints[i - 1].min &&
        angle <= this.constraints[i - 1].max
      ) {
        continue;
      }

      // Adjust the position of the next joint
      const constrainedAngle =
        angle < this.constraints[i - 1].min
          ? this.constraints[i - 1].min
          : this.constraints[i - 1].max;

      // Create rotation axis (perpendicular to the plane formed by v1 and v2)
      const axis = new THREE.Vector3().crossVectors(v1, v2).normalize();

      // Create rotation matrix
      const m = new THREE.Matrix4().makeRotationAxis(
        axis,
        constrainedAngle - angle
      );

      // Apply rotation to the second vector
      v2.applyMatrix4(m);
      v2.normalize().multiplyScalar(this.lengths[i]);

      // Set the new position of the next joint
      next.copy(current).add(v2);
    }
  }

  // Apply bias toward rest pose
  applyRestPoseBias(biasStrength) {
    // Skip the root joint (index 0) since it's fixed
    for (let i = 1; i < this.joints.length; i++) {
      // Calculate vector from current position to rest position
      const toRest = new THREE.Vector3().subVectors(
        this.restPose[i],
        this.joints[i]
      );

      // Apply partial movement toward rest pose
      const bias = toRest.multiplyScalar(biasStrength);
      this.joints[i].add(bias);

      // Ensure segment length is maintained after adding bias
      if (i > 0) {
        const prev = this.joints[i - 1];
        const current = this.joints[i];

        const direction = new THREE.Vector3()
          .subVectors(current, prev)
          .normalize();
        current.copy(prev).add(direction.multiplyScalar(this.lengths[i - 1]));
      }
    }
  }

  solve(target, tolerance = 0.01, maxIterations = 10) {
    const targetVector = new THREE.Vector3().copy(target);
    const rootPos = new THREE.Vector3().copy(this.joints[0]);
    const distanceToTarget = rootPos.distanceTo(targetVector);

    // Calculate target reachability
    const targetReachable = this.totalLength >= distanceToTarget;

    // Determine rest pose bias strength based on distance to target
    // When target is close to max reach, reduce bias to allow stretching
    let currentBias = this.restPoseBias;
    if (targetReachable) {
      // Gradually reduce bias as we approach max reach
      const reachRatio = distanceToTarget / this.totalLength;
      currentBias = this.restPoseBias * (1 - Math.pow(reachRatio, 2));
    } else {
      // Very low bias when target is unreachable to allow maximum extension
      currentBias = this.restPoseBias * 0.1;
    }

    // If target is unreachable, stretch the chain as far as possible
    if (!targetReachable) {
      const direction = new THREE.Vector3()
        .subVectors(targetVector, rootPos)
        .normalize();

      // Position joints in a straight line toward target
      let currentPos = rootPos.clone();
      for (let i = 1; i < this.joints.length; i++) {
        currentPos.add(direction.clone().multiplyScalar(this.lengths[i - 1]));
        this.joints[i].copy(currentPos);
      }

      // Apply constraints to make the pose more natural
      this.applyConstraints();

      // Apply mild rest pose bias for unreachable targetssss
      this.applyRestPoseBias(currentBias);
      return;
    }

    let iterations = 0;
    while (iterations < maxIterations) {
      // Forward reaching - set end effector to target
      this.joints[this.joints.length - 1].copy(targetVector);

      // Work backward to the root
      for (let i = this.joints.length - 2; i >= 0; i--) {
        const currentToNext = new THREE.Vector3().subVectors(
          this.joints[i + 1],
          this.joints[i]
        );

        // Get the direction and scale to segment length
        currentToNext.normalize().multiplyScalar(this.lengths[i]);

        // Position current joint based on next joint
        this.joints[i].copy(this.joints[i + 1]).sub(currentToNext);
      }

      // Backward reaching - fix the root position
      this.joints[0].copy(rootPos);

      // Work forward to end effector
      for (let i = 0; i < this.joints.length - 1; i++) {
        const currentToNext = new THREE.Vector3().subVectors(
          this.joints[i + 1],
          this.joints[i]
        );

        // Get the direction and scale to segment length
        currentToNext.normalize().multiplyScalar(this.lengths[i]);

        // Position next joint based on current joint
        this.joints[i + 1].copy(this.joints[i]).add(currentToNext);
      }

      // Apply constraints to maintain natural poses
      this.applyConstraints();

      // Apply rest pose bias
      this.applyRestPoseBias(currentBias);

      // Check if we're close enough to the target
      if (
        this.joints[this.joints.length - 1].distanceTo(targetVector) < tolerance
      ) {
        break;
      }

      iterations++;
    }
  }

  reset() {
    for (let i = 0; i < this.joints.length; i++) {
      this.joints[i].copy(this.originalJoints[i]);
    }
  }
}

const LegWithIK = () => {
  const { scene } = useThree();
  const segmentRefs = useRef(
    Array(3)
      .fill()
      .map(() => React.createRef())
  );
  const jointRefs = useRef(
    Array(4)
      .fill()
      .map(() => React.createRef())
  );
  const bodyRef = useRef();
  const targetRef = useRef();
  const footPositionRef = useRef(new THREE.Vector3(0, 0, 0));
  const distanceLineRef = useRef();
  const stepProgressRef = useRef(0);
  const isSteppingRef = useRef(false);
  const sphereRef = useRef();

  const fabrikSolver = useRef(null);

  const legData = useMemo(
    () => ({
      joints: Array(4)
        .fill()
        .map(() => new THREE.Vector3()),
      segmentLengths: [0.8, 1, 0.9],
      targetPos: new THREE.Vector3(),
      maxStretch: 1.5, // <-- Maximum distance before leg steps
      stepDuration: 15, 
      bodyOffset: new THREE.Vector3(-0.25, 0, 0),
    }),
    []
  );

  useEffect(() => {
    // Initial joint positions forming a typical spider leg shape
    // This will also serve as our rest pose
    const initialJoints = [
      new THREE.Vector3(0, 0, 0),   // <-- Hip
      new THREE.Vector3(-1, 0.5, 0), // <-- Knee (slightly up)
      new THREE.Vector3(-2.5, 0, 0), // <-- Ankle
      new THREE.Vector3(-3.5, 0, 0), // <-- Foot
    ];
    fabrikSolver.current = new FABRIK(initialJoints, legData.segmentLengths);

    // Initialize foot position
    footPositionRef.current.copy(initialJoints[3]);

    // Update rest pose when needed
    const updateRestPose = () => {
      if (fabrikSolver.current) {
        // Define our ideal rest pose - can be adjusted as needed
        const restPose = [
          new THREE.Vector3(0, 0, 0),   // <--  Hip (fixed)
          new THREE.Vector3(-1, 0.5, 0), // <--  Knee - slightly raised
          new THREE.Vector3(-2.5, 0, 0), // <--  Ankle
          new THREE.Vector3(-3.5, 0, 0), // <--  Foot
        ];

        for (let i = 0; i < restPose.length; i++) {
          fabrikSolver.current.restPose[i].copy(restPose[i]);
        }
      }
    };

    updateRestPose();
  }, []);

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
    // This makes the rest pose "lean" toward the target direction
    if (distanceToTarget > 0.1) {
      const direction = toTarget.clone().normalize();

      // Generate a rest pose that points in the target direction
      // but maintains the general shape of the leg
      const restDirection = direction.clone().multiplyScalar(0.7); // Scaled direction vector

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
    // Get direction vector from start to end
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
    // YXZ order means rotation will be applied in the order: yaw (Y), pitch (X), roll (Z)
    const euler = new THREE.Euler(pitch, yaw, 0, "YXZ");
    object.quaternion.setFromEuler(euler);
  }

  useFrame(() => {
    if (!bodyRef.current || !targetRef.current) return;

    const body = bodyRef.current;
    const target = targetRef.current;

    // Get target position in world space
    const targetWorldPos = new THREE.Vector3();
    target.getWorldPosition(targetWorldPos);

    // Raycast to find ground intersection
    const raycaster = new THREE.Raycaster(
      targetWorldPos,
      new THREE.Vector3(0, -0.8, 0)
    );
    const intersects = raycaster.intersectObjects(
      scene.children.filter((child) => child.name === "ground")
    );

    if (intersects.length > 0) {
      // Update target position to where ray hits ground
      legData.targetPos.copy(intersects[0].point);
      target.position.y = body.position.y - intersects[0].distance;

      // Update distance line visualization
      if (distanceLineRef.current) {
        const points = [footPositionRef.current, legData.targetPos];
        distanceLineRef.current.geometry.setFromPoints(points);
      }

      // Update sphere position to match intersection point
      if (sphereRef.current) {
        sphereRef.current.position.copy(legData.targetPos);
      }

      // Calculate distance to target
      const distanceToTarget = footPositionRef.current.distanceTo(
        legData.targetPos
      );

      // Stepping logic when distance exceeds max stretch
      if (distanceToTarget > legData.maxStretch || isSteppingRef.current) {
        if (!isSteppingRef.current) {
          isSteppingRef.current = true;
          stepProgressRef.current = 0;
        }

        stepProgressRef.current++;
        if (stepProgressRef.current >= legData.stepDuration) {
          isSteppingRef.current = false; // <-- Here's where the step ends
          footPositionRef.current.copy(legData.targetPos); // <-- Sudden position change
        } else {
          const t = stepProgressRef.current / legData.stepDuration;
          footPositionRef.current.lerpVectors(
            footPositionRef.current,
            legData.targetPos,
            t
          );
        }
      }

      // Get body position and apply offset
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

          // Orient to point from start to end
          // ref.current.lookAt(end);

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
      {/* Body with transform controls */}
      <TransformControls object={bodyRef} mode="translate" size="0.5">
        <mesh ref={bodyRef} position={[0, 1.9, 0]} castShadow>
          <sphereGeometry args={[0.3]} />
          <meshStandardMaterial color="#8B4513" />

          {/* Target point attached to body */}
          <mesh ref={targetRef} position={[-1.5, 0, 0]}>
            <sphereGeometry args={[0.1]} />
            <meshStandardMaterial color="yellow" />
          </mesh>
        </mesh>
      </TransformControls>

      {/* Sphere at raycast intersection (foot target) */}
      <mesh ref={sphereRef}>
        <sphereGeometry args={[0.2]} />
        <meshStandardMaterial color="blue" transparent opacity={0.5} />
      </mesh>

      {/* Joint visualizations */}
      {jointRefs.current.map((ref, i) => (
        <mesh key={`joint-${i}`} ref={ref} castShadow>
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
        <mesh key={`segment-${i}`} ref={ref} castShadow>
          <boxGeometry args={[1, 1, 1]} />
          <meshStandardMaterial
            color={
              i === 0
                ? "#331100" // Femur - darkest brown
                : i === 1
                ? "#442200" // Tibia - darker brown
                : "#AA6622" // Tarsus - brown
            }
          />
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

export default LegWithIK;
