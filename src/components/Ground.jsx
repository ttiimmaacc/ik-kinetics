import * as THREE from "three";
import React, { useRef, useEffect } from "react";

const Ground = () => {
  
  
  return (
    <>
      <mesh rotation-x={-Math.PI / 2} name="ground-0" position-y={0} receiveShadow>
        <planeGeometry args={[20, 20]} />
        <meshStandardMaterial color="#ffffff" side={THREE.DoubleSide} />
      </mesh>
      <mesh position={[0, -8, 5]} name="ground-1" receiveShadow>
        <sphereGeometry args={[10]} />
        <meshStandardMaterial color="#cccccc" />
      </mesh>
      <mesh position={[5, -6, 5]} name="ground-2" receiveShadow>
        <sphereGeometry args={[10]} />
        <meshStandardMaterial color="#cccccc" />
      </mesh>
    </>
  );
};

export default Ground;
