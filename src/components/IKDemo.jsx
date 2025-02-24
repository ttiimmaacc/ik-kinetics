import React from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Sky } from "@react-three/drei";
import Ground from "./Ground";

const IKDemo = () => {
  return (
    <div style={{ width: "100%", height: "100vh", position: "fixed" }}>
      <Canvas camera={{ position: [10, 5, 10], fov: 50 }}>
        <color attach="background" args={["skyblue"]} />
        <ambientLight intensity={0.4} />
        <directionalLight position={[1, 1, 1]} intensity={3} castShadow/>
        <pointLight castShadow intensity={20} position={[1,3,1]} />
        <Sky sunPosition={[100, 20, 100]} />
        <Ground />
        <OrbitControls makeDefault />
        <mesh castShadow >
          <boxGeometry args={[1, 1, 1]} />
          <meshStandardMaterial color="orange" />
        </mesh>
      </Canvas>
    </div>
  );
};

export default IKDemo;
