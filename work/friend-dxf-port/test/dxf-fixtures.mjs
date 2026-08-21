const pairs = (entries) => entries.flatMap(([code, value]) => [String(code), String(value)]);
const entity = (type, entries) => ['0', type, ...pairs(entries)];
const line = (a, b, options = {}) => entity('LINE', [[8, 'CUT'], [10, a[0]], [20, a[1]], [30, options.startZ ?? 0], [11, b[0]], [21, b[1]], [31, options.endZ ?? 0], ...(options.thickness === undefined ? [] : [[39, options.thickness]]), ...(options.normal ? [[210, options.normal[0]], [220, options.normal[1]], [230, options.normal[2]]] : [])]);
const poly = (vertices, options = {}) => entity('LWPOLYLINE', [[8, 'CUT'], [90, vertices.length], [70, options.closed === false ? 0 : 1], ...(options.elevation === undefined ? [] : [[38, options.elevation]]), ...(options.thickness === undefined ? [] : [[39, options.thickness]]), ...(options.width === undefined ? [] : [[43, options.width]]), ...vertices.flatMap(([x, y], i) => [[10, x], [20, y], ...(options.bulges?.[i] === undefined ? [] : [[42, options.bulges[i]]])]), ...(options.normal ? [[210, options.normal[0]], [220, options.normal[1]], [230, options.normal[2]]] : [])]);
const circle = (centre, radius, normal) => entity('CIRCLE', [[8, 'CUT'], [10, centre[0]], [20, centre[1]], [30, 0], [40, radius], ...(normal ? [[210, normal[0]], [220, normal[1]], [230, normal[2]]] : [])]);
const dxf = (...entities) => ['0', 'SECTION', '2', 'HEADER', '9', '$INSUNITS', '70', '4', '0', 'ENDSEC', '0', 'SECTION', '2', 'ENTITIES', ...entities.flat(), '0', 'ENDSEC', '0', 'EOF', ''].join('\n');
const triangle = (options = {}) => [line([0, 0], [20, 0], options), line([20, 0], [0, 10], options), line([0, 10], [0, 0], options)];

export const DXF_FIXTURES = Object.freeze([
  { id: 'planar-closed-triangle', text: dxf(...triangle()), accepted: true },
  { id: 'line-3d', text: dxf(...triangle({ startZ: 2, endZ: 2 })), errorCode: 'DXF_NON_PLANAR_Z' },
  { id: 'open-gap-0.2mm', text: dxf(line([0, 0], [20, 0]), line([20, 0], [0, 10]), line([0, 10], [0, .2])), errorCode: 'DXF_OPEN_CONTOUR' },
  { id: 'nonzero-elevation', text: dxf(poly([[0, 0], [20, 0], [0, 10]], { elevation: 1.5 })), errorCode: 'DXF_NONZERO_ELEVATION' },
  { id: 'nonzero-thickness', text: dxf(...triangle({ thickness: .5 })), errorCode: 'DXF_NONZERO_THICKNESS' },
  { id: 'nonzero-width', text: dxf(poly([[0, 0], [20, 0], [0, 10]], { width: .8 })), errorCode: 'DXF_NONZERO_WIDTH' },
  { id: 'nonzero-bulge', text: dxf(poly([[0, 0], [20, 0], [0, 10]], { bulges: [.25] })), errorCode: 'DXF_UNSUPPORTED_BULGE' },
  { id: 'tilted-normal', text: dxf(circle([10, 10], 5, [0, .6, .8])), errorCode: 'DXF_UNSUPPORTED_EXTRUSION' },
  { id: 'negative-z-normal', text: dxf(circle([10, 10], 5, [0, 0, -1])), errorCode: 'DXF_UNSUPPORTED_EXTRUSION' },
  { id: 'malformed-extrusion-number', text: dxf(circle([10, 10], 5, [0, 0, 'abc'])), errorCode: 'DXF_INVALID_NUMBER' },
  { id: 'malformed-bulge-number', text: dxf(poly([[0, 0], [20, 0], [0, 10]], { bulges: ['abc'] })), errorCode: 'DXF_INVALID_NUMBER' },
  { id: 'zero-length-line', text: dxf(...triangle(), line([5, 5], [5, 5])), errorCode: 'DXF_ZERO_LENGTH' },
  { id: 'outer-with-hole', text: dxf(poly([[0, 0], [40, 0], [40, 30], [0, 30]]), poly([[10, 10], [30, 10], [30, 20], [10, 20]])), accepted: true },
  { id: 'two-independent-outers', text: dxf(poly([[0, 0], [10, 0], [10, 10], [0, 10]]), poly([[30, 0], [40, 0], [40, 10], [30, 10]])), errorCode: 'DXF_MULTIPLE_OUTERS' },
  { id: 'self-intersecting', text: dxf(poly([[0, 0], [20, 20], [0, 20], [20, 0]])), errorCode: 'DXF_SELF_INTERSECTION' },
]);
