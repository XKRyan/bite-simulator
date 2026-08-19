const pairs = (values) => values.flatMap(([code, value]) => [String(code), String(value)]);

function entity(type, values) {
  return ['0', type, ...pairs(values)];
}

function line(start, end, options = {}) {
  const values = [
    [8, 'CUT'],
    [10, start[0]], [20, start[1]], [30, options.startZ ?? 0],
    [11, end[0]], [21, end[1]], [31, options.endZ ?? 0],
  ];
  if (options.thickness !== undefined) values.push([39, options.thickness]);
  if (options.normal) {
    values.push([210, options.normal[0]], [220, options.normal[1]], [230, options.normal[2]]);
  }
  return entity('LINE', values);
}

function lwpolyline(vertices, options = {}) {
  const values = [
    [8, 'CUT'],
    [90, vertices.length],
    [70, options.closed === false ? 0 : 1],
  ];
  if (options.elevation !== undefined) values.push([38, options.elevation]);
  if (options.thickness !== undefined) values.push([39, options.thickness]);
  if (options.width !== undefined) values.push([43, options.width]);
  vertices.forEach(([x, y], index) => {
    values.push([10, x], [20, y]);
    if (options.bulges?.[index] !== undefined) values.push([42, options.bulges[index]]);
  });
  if (options.normal) {
    values.push([210, options.normal[0]], [220, options.normal[1]], [230, options.normal[2]]);
  }
  return entity('LWPOLYLINE', values);
}

function circle(center, radius, options = {}) {
  const values = [
    [8, 'CUT'],
    [10, center[0]], [20, center[1]], [30, options.z ?? 0],
    [40, radius],
  ];
  if (options.normal) {
    values.push([210, options.normal[0]], [220, options.normal[1]], [230, options.normal[2]]);
  }
  return entity('CIRCLE', values);
}

function dxf(...entities) {
  return [
    '0', 'SECTION',
    '2', 'HEADER',
    '9', '$ACADVER',
    '1', 'AC1015',
    '9', '$INSUNITS',
    '70', '4',
    '0', 'ENDSEC',
    '0', 'SECTION',
    '2', 'ENTITIES',
    ...entities.flat(),
    '0', 'ENDSEC',
    '0', 'EOF',
    '',
  ].join('\n');
}

const closedTriangle = (options = {}) => [
  line([0, 0], [20, 0], options),
  line([20, 0], [0, 10], options),
  line([0, 10], [0, 0], options),
];

export const DXF_FIXTURES = Object.freeze([
  {
    id: 'planar-closed-triangle',
    description: 'A closed three-LINE contour in the world XY plane.',
    text: dxf(...closedTriangle()),
    expected: {
      accepted: true,
      entityCounts: { LINE: 3 },
      planar: true,
      topology: { rings: 1, outerRings: 1, holes: 0 },
    },
  },
  {
    id: 'line-3d',
    description: 'The same closed XY projection with every LINE elevated to Z=2 mm.',
    text: dxf(...closedTriangle({ startZ: 2, endZ: 2 })),
    expected: {
      accepted: false,
      errorCode: 'DXF_NON_PLANAR_Z',
      entityCounts: { LINE: 3 },
      planar: false,
      offendingGroupCodes: [30, 31],
    },
  },
  {
    id: 'open-gap-0.2mm',
    description: 'Three LINEs whose final endpoint stops 0.2 mm short of the first endpoint.',
    text: dxf(
      line([0, 0], [20, 0]),
      line([20, 0], [0, 10]),
      line([0, 10], [0, 0.2]),
    ),
    expected: {
      accepted: false,
      errorCode: 'DXF_OPEN_CONTOUR',
      entityCounts: { LINE: 3 },
      planar: true,
      topology: { rings: 0, openComponents: 1, gapMm: 0.2 },
    },
  },
  {
    id: 'nonzero-elevation',
    description: 'A closed LWPOLYLINE with a non-zero group 38 elevation.',
    text: dxf(lwpolyline([[0, 0], [20, 0], [0, 10]], { elevation: 1.5 })),
    expected: {
      accepted: false,
      errorCode: 'DXF_NONZERO_ELEVATION',
      entityCounts: { LWPOLYLINE: 1 },
      offendingGroupCodes: [38],
      offendingValues: [1.5],
    },
  },
  {
    id: 'nonzero-thickness',
    description: 'A closed LINE contour with non-zero DXF thickness on one edge.',
    text: dxf(
      line([0, 0], [20, 0], { thickness: 0.5 }),
      line([20, 0], [0, 10]),
      line([0, 10], [0, 0]),
    ),
    expected: {
      accepted: false,
      errorCode: 'DXF_NONZERO_THICKNESS',
      entityCounts: { LINE: 3 },
      offendingGroupCodes: [39],
      offendingValues: [0.5],
    },
  },
  {
    id: 'nonzero-width',
    description: 'A closed LWPOLYLINE with a non-zero constant width.',
    text: dxf(lwpolyline([[0, 0], [20, 0], [0, 10]], { width: 0.8 })),
    expected: {
      accepted: false,
      errorCode: 'DXF_NONZERO_WIDTH',
      entityCounts: { LWPOLYLINE: 1 },
      offendingGroupCodes: [43],
      offendingValues: [0.8],
    },
  },
  {
    id: 'nonzero-bulge',
    description: 'A closed LWPOLYLINE with a non-zero bulge on its first vertex.',
    text: dxf(lwpolyline([[0, 0], [20, 0], [0, 10]], { bulges: [0.25] })),
    expected: {
      accepted: false,
      errorCode: 'DXF_UNSUPPORTED_BULGE',
      entityCounts: { LWPOLYLINE: 1 },
      offendingGroupCodes: [42],
      offendingValues: [0.25],
    },
  },
  {
    id: 'tilted-normal',
    description: 'A CIRCLE whose extrusion normal is tilted away from world +Z.',
    text: dxf(circle([10, 10], 5, { normal: [0, 0.6, 0.8] })),
    expected: {
      accepted: false,
      errorCode: 'DXF_UNSUPPORTED_EXTRUSION',
      entityCounts: { CIRCLE: 1 },
      extrusionNormal: [0, 0.6, 0.8],
    },
  },
  {
    id: 'negative-z-normal',
    description: 'A CIRCLE whose extrusion normal points along world -Z.',
    text: dxf(circle([10, 10], 5, { normal: [0, 0, -1] })),
    expected: {
      accepted: false,
      errorCode: 'DXF_UNSUPPORTED_EXTRUSION',
      entityCounts: { CIRCLE: 1 },
      extrusionNormal: [0, 0, -1],
    },
  },
  {
    id: 'malformed-extrusion-number',
    description: 'A CIRCLE with an explicit non-numeric extrusion Z component.',
    text: dxf(circle([10, 10], 5, { normal: [0, 0, 'abc'] })),
    expected: {
      accepted: false,
      errorCode: 'DXF_INVALID_NUMBER',
      entityCounts: { CIRCLE: 1 },
      offendingGroupCodes: [230],
    },
  },
  {
    id: 'malformed-bulge-number',
    description: 'A closed LWPOLYLINE with an explicit non-numeric bulge.',
    text: dxf(lwpolyline([[0, 0], [20, 0], [0, 10]], { bulges: ['abc'] })),
    expected: {
      accepted: false,
      errorCode: 'DXF_INVALID_NUMBER',
      entityCounts: { LWPOLYLINE: 1 },
      offendingGroupCodes: [42],
    },
  },
  {
    id: 'zero-length-line',
    description: 'A valid closed contour accompanied by a zero-length LINE that must not be dropped.',
    text: dxf(...closedTriangle(), line([5, 5], [5, 5])),
    expected: {
      accepted: false,
      errorCode: 'DXF_ZERO_LENGTH',
      entityCounts: { LINE: 4 },
    },
  },
  {
    id: 'outer-with-hole',
    description: 'One outer LWPOLYLINE and one strictly contained hole with the same input winding.',
    text: dxf(
      lwpolyline([[0, 0], [40, 0], [40, 30], [0, 30]]),
      lwpolyline([[10, 10], [30, 10], [30, 20], [10, 20]]),
    ),
    expected: {
      accepted: true,
      entityCounts: { LWPOLYLINE: 2 },
      planar: true,
      topology: {
        rings: 2,
        outerRings: 1,
        holes: 1,
        outerAreaMm2: 1200,
        holeAreaMm2: 200,
        netAreaMm2: 1000,
        inputWindingIsSame: true,
      },
    },
  },
  {
    id: 'two-independent-outers',
    description: 'Two disjoint top-level closed LWPOLYLINE contours.',
    text: dxf(
      lwpolyline([[0, 0], [10, 0], [10, 10], [0, 10]]),
      lwpolyline([[30, 0], [40, 0], [40, 10], [30, 10]]),
    ),
    expected: {
      accepted: false,
      errorCode: 'DXF_MULTIPLE_OUTERS',
      entityCounts: { LWPOLYLINE: 2 },
      planar: true,
      topology: { rings: 2, outerRings: 2, holes: 0 },
    },
  },
  {
    id: 'self-intersecting',
    description: 'A closed bow-tie LWPOLYLINE with one proper self-intersection.',
    text: dxf(lwpolyline([[0, 0], [20, 20], [0, 20], [20, 0]])),
    expected: {
      accepted: false,
      errorCode: 'DXF_SELF_INTERSECTION',
      entityCounts: { LWPOLYLINE: 1 },
      planar: true,
      topology: { selfIntersections: 1 },
    },
  },
]);

export const DXF_FIXTURES_BY_ID = Object.freeze(Object.fromEntries(
  DXF_FIXTURES.map((fixture) => [fixture.id, fixture]),
));
