import * as THREE from 'three';

const Ground = () => {
  return (
    <mesh rotation-x={-Math.PI / 2} position-y={0} name="ground" receiveShadow>
      <planeGeometry args={[1000,1000]} />
      <meshStandardMaterial color="#ffffff" side={THREE.DoubleSide} />
    </mesh>
  );
};

export default Ground;