// Helper component to manage raycast visualization
import React from 'react';

const RaycastManager = ({ legs, legConfigs }) => {
  return (
    <>
      {legConfigs.map((config, index) => (
        <mesh 
          key={`${config.id}-raycast`} 
          ref={legs[index].raycastPointRef}
        >
          <sphereGeometry args={[0.1]} />
          <meshStandardMaterial 
            color={config.side === "left" ? "blue" : "green"} 
            transparent 
            opacity={legs[index].isGroundedRef ? 0.7 : 0.0} 
          />
        </mesh>
      ))}
    </>
  );
};

export default RaycastManager;
