// Base coordinates are Z-up. The project GLB uses Y-up and the opposite J1 display direction.
export const toWebPosition = ([x, y, z]) => [x, z, y];

export const toWebJointAngles = ([base, ...arm]) => [-base, ...arm];
