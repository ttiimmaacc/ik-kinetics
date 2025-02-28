import React from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Sky, GizmoHelper, GizmoViewport, GizmoViewcube } from "@react-three/drei";
import Ground from "./Ground";
import LegWithIK from "./LegWithIK";

const IKDemo = () => {
  return (
    <div style={{ width: "100%", height: "100vh", position: "fixed" }}>
      <Canvas camera={{ position: [10, 5, 10], fov: 50 }}>
        <color attach="background" args={["skyblue"]} />
        <ambientLight intensity={0.8} />
        <directionalLight position={[1, 1, 1]} intensity={4} castShadow />
        <pointLight castShadow intensity={20} position={[1, 3, 1]} />
        <Sky sunPosition={[100, 20, 100]} />
        <Ground />
        <LegWithIK />
        <OrbitControls makeDefault />
        <gridHelper position={[0,0,0]} />
        <GizmoHelper
        alignment="bottom-right" // widget alignment within scene
        margin={[80, 80]} // widget margins (X, Y) 
        >
        <GizmoViewport axisColors={['red', 'green', 'blue']} labelColor="black" />
         <GizmoViewcube />
        </GizmoHelper>
      </Canvas>
    </div>
  );
};

export default IKDemo;
