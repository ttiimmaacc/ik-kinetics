import React, { useRef, useMemo, useEffect } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import * as THREE from "three";

// Simple skeleton helper component to visualize IK chain
const SkeletonHelper = ({ fabrikSolver, bodyRef, bodyOffset }) => {
  const jointRefs = useRef(
    Array(4) // 4 joints in the system
      .fill()
      .map(() => React.createRef())
  );
  
  const boneRefs = useRef(
    Array(3) // 3 bones connecting the joints
      .fill()
      .map(() => React.createRef())
  );
  
  const jointPositions = useRef([
    new THREE.Vector3(),
    new THREE.Vector3(),
    new THREE.Vector3(),
    new THREE.Vector3()
  ]);
  
  useFrame(() => {
    if (!fabrikSolver.current || !bodyRef.current) return;
    
    // Get base position from body
    const basePos = new THREE.Vector3();
    bodyRef.current.getWorldPosition(basePos);
    basePos.add(bodyOffset);
    
    // Copy joint positions from the FABRIK solver
    jointPositions.current[0].copy(fabrikSolver.current.joints[0]);
    jointPositions.current[1].copy(fabrikSolver.current.joints[1]);
    jointPositions.current[2].copy(fabrikSolver.current.joints[2]);
    jointPositions.current[3].copy(fabrikSolver.current.joints[3]);
    
    // Update joint spheres
    jointRefs.current.forEach((ref, i) => {
      if (ref.current) {
        ref.current.position.copy(jointPositions.current[i]);
      }
    });
    
    // Update bones connecting joints
    boneRefs.current.forEach((ref, i) => {
      if (ref.current) {
        const start = jointPositions.current[i];
        const end = jointPositions.current[i + 1];
        
        // Position at midpoint
        const midpoint = start.clone().add(end).multiplyScalar(0.5);
        ref.current.position.copy(midpoint);
        
        // Orient toward end joint
        ref.current.lookAt(end);
        
        // Scale to match joint distance
        const length = start.distanceTo(end);
        ref.current.scale.set(0.03, 0.03, length);
      }
    });
  });
  
  return (
    <group>
      {/* Joint spheres */}
      {jointRefs.current.map((ref, i) => (
        <mesh key={`joint-${i}`} ref={ref}>
          <sphereGeometry args={[0.08]} />
          <meshStandardMaterial 
            color={i === 0 ? "#FF00FF" : i === 3 ? "#00FFFF" : "#FFFF00"} 
            transparent={true}
            opacity={0.7}
          />
        </mesh>
      ))}
      
      {/* Bones connecting joints */}
      {boneRefs.current.map((ref, i) => (
        <mesh key={`bone-${i}`} ref={ref}>
          <cylinderGeometry args={[1, 1, 1, 8]} />
          <meshStandardMaterial 
            color={"#FFFFFF"} 
            transparent={true}
            opacity={0.6}
          />
        </mesh>
      ))}
    </group>
  );
};

export default SkeletonHelper;