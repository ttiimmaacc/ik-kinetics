import React from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';

const IKDemo = () => {
  return (
    <div style={{ width: '100%', height: '100vh', position: 'fixed' }}>
      <Canvas camera={{ position: [5, 5, 5], fov: 50 }}>
        <color attach="background" args={["#ffffff"]} />
        <ambientLight intensity={0.8} />
        <directionalLight position={[5, 5, 5]} intensity={1} />
        <OrbitControls makeDefault />
        <mesh>
          <boxGeometry args={[1, 1, 1]} />
          <meshStandardMaterial color="orange" />
        </mesh>
      </Canvas>
    </div>
  );
};

export default IKDemo;