import * as THREE from "three";

export class FABRIK {
  constructor(joints, lengths) {
    this.joints = joints.map((j) => new THREE.Vector3().copy(j));
    this.originalJoints = joints.map((j) => new THREE.Vector3().copy(j));
    this.restPose = joints.map((j) => new THREE.Vector3().copy(j));
    this.lengths = lengths;
    this.totalLength = lengths.reduce((sum, length) => sum + length, 0);

    // Define angle constraints for each joint (in radians)
    this.constraints = [
      { min: -Math.PI / 1, max: Math.PI / 2 }, // <-- Hip joint (-9 degrees to 360 degrees)
      { min: -Math.PI / 12, max: Math.PI / 3 }, // <-- Knee joint (15 degrees of backward bending)
      { min: -Math.PI / 0, max: Math.PI / 2 }, // <-- Ankle joint (-infinity to 90 degrees)
    ];

    // Define directional constraints for each joint (in radians)
    this.directionalConstraints = [
      null, // <-- Hip joint
      null, // <-- Knee joint
      {
        axis: new THREE.Vector3(1, 0, 1), // Local Z axis
        minAngle: -Math.PI / 3, // Limit backward bending
        maxAngle: Math.PI / 2, // Allow forward bending
      },
    ];

    // Rest pose bias strength (0-1)
    // Higher values make the leg return to rest pose more strongly
    this.restPoseBias = 0.12;
  }

  // Apply constraints to angle between segments
  applyConstraints() {
    for (let i = 1; i < this.joints.length - 1; i++) {
      // <-- loop through each joint (except the first and last)
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

  // Apply directional constraints to prevent unnatural joint rotations
  applyDirectionalConstraints() {
    for (let i = 1; i < this.joints.length - 1; i++) {
      // Start with ankle (index 2)
      if (!this.directionalConstraints[i]) continue;

      // Get the joint positions
      const prev = this.joints[i - 1]; // Knee
      const current = this.joints[i]; // Ankle
      const next = this.joints[i + 1]; // Foot

      // Calculate segment vectors
      const segmentUp = new THREE.Vector3()
        .subVectors(current, prev)
        .normalize();
      const segmentDown = new THREE.Vector3()
        .subVectors(next, current)
        .normalize();

      // Create a local coordinate system for the joint
      const forward = new THREE.Vector3().copy(segmentUp);
      const right = new THREE.Vector3(0, 1, 0).cross(forward).normalize();
      const up = new THREE.Vector3().crossVectors(forward, right).normalize();

      // Project the downward segment onto the forward-up plane
      const projectedVector = new THREE.Vector3();
      projectedVector.copy(segmentDown);
      const rightComponent = right.dot(segmentDown);
      projectedVector.sub(right.clone().multiplyScalar(rightComponent));
      projectedVector.normalize();

      // Calculate angle between forward and projected vector in the forward-up plane
      const angle = Math.atan2(
        projectedVector.dot(up),
        projectedVector.dot(forward)
      );

      // Apply constraints if needed
      const constraint = this.directionalConstraints[i];
      if (angle < constraint.minAngle || angle > constraint.maxAngle) {
        // Constrain the angle
        const constrainedAngle =
          angle < constraint.minAngle
            ? constraint.minAngle
            : constraint.maxAngle;

        // Create a new vector at the constrained angle
        const constrainedVector = new THREE.Vector3()
          .copy(forward)
          .multiplyScalar(Math.cos(constrainedAngle))
          .addScaledVector(up, Math.sin(constrainedAngle));

        // Restore the right component to maintain the 3D orientation
        constrainedVector.addScaledVector(right, rightComponent);
        constrainedVector.normalize();

        // Adjust the next joint position
        next.copy(current).addScaledVector(constrainedVector, this.lengths[i]);
      }
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

      // Apply directional constraints to prevent unnatural ankle bending
      this.applyDirectionalConstraints();

      // Apply mild rest pose bias for unreachable targets
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

      // Apply directional constraints to prevent unnatural ankle bending
      this.applyDirectionalConstraints();

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