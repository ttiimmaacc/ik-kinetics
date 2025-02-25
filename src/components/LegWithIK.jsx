import React, { useRef, useMemo, useEffect } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import { TransformControls } from "@react-three/drei";
import * as THREE from "three";

class FABRIK {
  constructor(joints, lengths) {
    this.joints = joints.map((j) => new THREE.Vector3().copy(j));
    this.lengths = lengths;
    this.totalLength = lengths.reduce((sum, length) => sum + length, 0);
  }

  solve(target, tolerance = 0.01, maxIterations = 10) {
    const targetVector = new THREE.Vector3().copy(target);

    if (this.totalLength < targetVector.distanceTo(this.joints[0])) {
      // Target is unreachable
      const direction = targetVector.clone().sub(this.joints[0]).normalize();
      for (let i = 1; i < this.joints.length; i++) {
        this.joints[i]
          .copy(this.joints[i - 1])
          .add(direction.clone().multiplyScalar(this.lengths[i - 1]));
      }
      return;
    }

    let iterations = 0;
    while (iterations < maxIterations) {
      // Forward reaching
      this.joints[this.joints.length - 1].copy(targetVector);
      for (let i = this.joints.length - 2; i >= 0; i--) {
        const direction = this.joints[i]
          .clone()
          .sub(this.joints[i + 1])
          .normalize();
        this.joints[i]
          .copy(this.joints[i + 1])
          .add(direction.multiplyScalar(this.lengths[i]));
      }

      // Backward reaching
      this.joints[0] = new THREE.Vector3().copy(this.joints[0]);
      for (let i = 1; i < this.joints.length; i++) {
        const direction = this.joints[i]
          .clone()
          .sub(this.joints[i - 1])
          .normalize();
        this.joints[i]
          .copy(this.joints[i - 1])
          .add(direction.multiplyScalar(this.lengths[i - 1]));
      }

      // Check if we've reached the target within tolerance
      if (
        this.joints[this.joints.length - 1].distanceTo(targetVector) < tolerance
      ) {
        break;
      }

      iterations++;
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
  const bodyRef = useRef();
  const targetRef = useRef();
  const footPositionRef = useRef(new THREE.Vector3(0, 0, 0));
  const distanceLineRef = useRef();
  const stepProgressRef = useRef(0);
  const isSteppingRef = useRef(false);

  const sphereRef = useRef(); // Ref for the sphere

  const fabrikSolver = useRef(null);

  const legData = useMemo(
    () => ({
      segments: Array(3)
        .fill()
        .map(() => new THREE.Vector3()),
      segmentLengths: [1, 1.5, 1],
      targetPos: new THREE.Vector3(),
      maxStretch: 2,
      stepDuration: 20,
      bodyOffset: new THREE.Vector3(-0.25, 0, 0),
    }),
    []
  );

  useEffect(() => {
    const initialJoints = [
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(2.5, 0, 0),
      new THREE.Vector3(3.3, 0, 0),
    ];
    fabrikSolver.current = new FABRIK(initialJoints, legData.segmentLengths);
  }, []);

  const calculateLegPositions = (basePos, footPos) => {
    if (!fabrikSolver.current) return;

    // Adjust foot position
    const adjustedFootPos = footPos.clone();
    const footOffset = new THREE.Vector3(-1, 0, 0); // Adjust this offset as needed
    adjustedFootPos.add(footOffset);

    // Solve using FABRIK algorithm
    const target = adjustedFootPos.clone();
    fabrikSolver.current.joints[0].copy(basePos);
    fabrikSolver.current.solve(target);

    // Apply custom adjustments
    // First segment adjustment
    const firstSegmentAngle = Math.PI / 6;
    const firstSegmentDir = new THREE.Vector3(
      adjustedFootPos.x - basePos.x,
      0,
      adjustedFootPos.z - basePos.z
    ).normalize();
    firstSegmentDir.applyAxisAngle(
      new THREE.Vector3(0, 0, 0),
      firstSegmentAngle
    );
    legData.segments[0]
      .copy(basePos)
      .add(firstSegmentDir.multiplyScalar(legData.segmentLengths[0]));

    // Last segment adjustment
    const lastSegmentLength = legData.segmentLengths[2];
    const lastSegmentDir = new THREE.Vector3(0, 1, 0).normalize();
    legData.segments[2]
      .copy(adjustedFootPos)
      .add(lastSegmentDir.multiplyScalar(lastSegmentLength));

    // Middle segment adjustment
    legData.segments[1].copy(legData.segments[2]).sub(legData.segments[0]);
    legData.segments[1].normalize().multiplyScalar(legData.segmentLengths[1]);
    legData.segments[1].add(legData.segments[0]);

    // Ensure the foot position matches the target
    legData.segments[2].copy(adjustedFootPos);
  };

  useFrame(() => {
    if (!bodyRef.current || !targetRef.current) return;

    const body = bodyRef.current;
    const target = targetRef.current;

    const targetWorldPos = new THREE.Vector3();
    target.getWorldPosition(targetWorldPos);

    // Raycast to find intersection with ground
    const raycaster = new THREE.Raycaster(
      targetWorldPos,
      new THREE.Vector3(0, -1, 0)
    );
    const intersects = raycaster.intersectObjects(
      scene.children.filter((child) => child.name === "ground")
    );

    if (intersects.length > 0) {
      legData.targetPos.copy(intersects[0].point);
      target.position.y = body.position.y - intersects[0].distance;

      if (distanceLineRef.current) {
        const points = [footPositionRef.current, legData.targetPos];
        distanceLineRef.current.geometry.setFromPoints(points);
      }

      // Update sphere position to match intersection point
      if (sphereRef.current) {
        sphereRef.current.position.copy(legData.targetPos);
      }

      const distanceToTarget = footPositionRef.current.distanceTo(
        legData.targetPos
      );

      if (distanceToTarget > legData.maxStretch || isSteppingRef.current) {
        if (!isSteppingRef.current) {
          isSteppingRef.current = true;
          stepProgressRef.current = 0;
        }

        stepProgressRef.current++;
        if (stepProgressRef.current >= legData.stepDuration) {
          isSteppingRef.current = false;
          footPositionRef.current.copy(legData.targetPos);
        } else {
          const t = stepProgressRef.current / legData.stepDuration;
          footPositionRef.current.lerpVectors(
            footPositionRef.current,
            legData.targetPos,
            t
          );
        }
      }

      const basePos = new THREE.Vector3();
      body.getWorldPosition(basePos);
      basePos.add(legData.bodyOffset);

      calculateLegPositions(basePos, footPositionRef.current);

      segmentRefs.current.forEach((ref, i) => {
        if (ref.current) {
          const start = i === 0 ? basePos : legData.segments[i - 1];
          const end = legData.segments[i];

          ref.current.position.copy(start.clone().add(end).multiplyScalar(0.5));
          ref.current.lookAt(end);
          ref.current.scale.set(0.2, 0.2, start.distanceTo(end));
        }
      });
    }
  });

  return (
    <group>
      {/* Body with transform controls */}
      <TransformControls object={bodyRef} mode="translate">
        <mesh ref={bodyRef} position={[0, 2.5, 0]}>
          <sphereGeometry args={[0.3]} />
          <meshStandardMaterial color="#F26157" />

          {/* Target point attached to body */}
          <mesh ref={targetRef} position={[0, 0, 0]}>
            <sphereGeometry args={[0.1]} />
            <meshStandardMaterial color="yellow" />
          </mesh>
        </mesh>
      </TransformControls>

      {/* Sphere at raycast intersection */}
      <mesh ref={sphereRef}>
        <sphereGeometry args={[0.2]} />
        <meshStandardMaterial color="blue" />
      </mesh>

      {/* Leg segments */}
      {segmentRefs.current.map((ref, i) => (
        <mesh key={i} ref={ref}>
          <boxGeometry args={[1, 1, 1]} />
          <meshStandardMaterial color={i === 1 ? "#FF0000" : "#666666"} />
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
