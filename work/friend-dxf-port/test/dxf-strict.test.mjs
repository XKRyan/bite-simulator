import assert from 'node:assert/strict';
import test from 'node:test';
import { DXF_FIXTURES } from './dxf-fixtures.mjs';
import { PENETRATION_GATE_METRES, DxfValidationError, GeometryContractError, assertNoInitialOverlap, classifyTriangleEdge, exactSolidRectanglePenetration, matchGeometricContactsToSolver, parseStrictPlanarDxf, prepareSolidTriangulation, validateSingleOuterSolid } from '../src/dxf-strict.mjs';

const fixture = (id) => DXF_FIXTURES.find((entry) => entry.id === id);
const parseSolid = (id) => validateSingleOuterSolid(parseStrictPlanarDxf(fixture(id).text).paths);
const expectCode = (fn, code) => assert.throws(fn, (error) => error instanceof DxfValidationError && error.code === code);
const expectContractCode = (fn, code) => assert.throws(fn, (error) => error instanceof GeometryContractError && error.code === code);
const squareSolid = () => validateSingleOuterSolid([{ type: 'polyline', closed: true, points: [{ x: 0, y: 0 }, { x: 4, y: 0 }, { x: 4, y: 4 }, { x: 0, y: 4 }] }]).solid;

test('ported fixture catalogue is ASCII and has the same strict acceptance decisions', () => {
  assert.equal(DXF_FIXTURES.length, 15); assert.equal(new Set(DXF_FIXTURES.map((entry) => entry.id)).size, 15);
  for (const entry of DXF_FIXTURES) { assert([...entry.text].every((char) => char.charCodeAt(0) <= 0x7f)); if (entry.accepted) assert.doesNotThrow(() => parseSolid(entry.id)); else expectCode(() => parseSolid(entry.id), entry.errorCode); }
});
test('ordinary holes are winding-independent and subtract from canonical solid area', () => {
  const { solid } = parseSolid('outer-with-hole'); assert.equal(solid.rings.length, 2); assert.equal(solid.holes.length, 1); assert.equal(solid.netArea, 1000);
});
test('open 0.2 mm chain is rejected rather than silently bridged', () => expectCode(() => parseSolid('open-gap-0.2mm'), 'DXF_OPEN_CONTOUR'));
test('triangulation preserves hole area and tags Earcut diagonals as non-boundary seams', () => {
  const { solid } = parseSolid('outer-with-hole');
  const earcutForFixture = () => [0, 1, 4, 1, 5, 4, 1, 2, 5, 2, 6, 5, 2, 3, 6, 3, 7, 6, 3, 0, 7, 0, 4, 7];
  const prepared = prepareSolidTriangulation(solid, earcutForFixture, { geometryFloor: 1e-9 }); assert.equal(prepared.triangulatedArea, 1000); assert.equal(prepared.quality.proof, 'contained + non-overlap + exact boundary/internal edge multiplicity + area equality'); assert.equal(classifyTriangleEdge(prepared, 0, 1).internalTriangleEdge, true); assert.equal(classifyTriangleEdge(prepared, 0, 0).boundary, true);
});
test('P0: repeated coverage of half a square cannot pass the former total-area-only check', () => {
  const solid = squareSolid();
  // Each copy has area 8; total 16 equals the square area, while the other half is absent.
  expectCode(() => prepareSolidTriangulation(solid, () => [0, 1, 2, 0, 1, 2], { geometryFloor: 1e-9 }), 'DXF_TRIANGLE_OVERLAP');
});
test('P0: equal-area mesh that fills a hole and omits an equal exterior patch is rejected', () => {
  const { solid } = parseSolid('outer-with-hole');
  const balancedButHoleCovering = [
    0, 1, 4, 1, 5, 4,
    4, 5, 6, 4, 6, 7,
    2, 3, 6, 3, 7, 6, 3, 0, 7, 0, 4, 7,
  ];
  expectCode(() => prepareSolidTriangulation(solid, () => balancedButHoleCovering, { geometryFloor: 1e-9 }), 'DXF_TRIANGLE_HOLE_COVERAGE');
});
test('P0: missing canonical boundary coverage fails before any area-only acceptance', () => {
  expectCode(() => prepareSolidTriangulation(squareSolid(), () => [0, 1, 2], { geometryFloor: 1e-9 }), 'DXF_TRIANGULATION_BOUNDARY_GAP');
});
test('feature floor is an explicit fail-closed contract', () => {
  const thin = validateSingleOuterSolid([{ type: 'polyline', closed: true, points: [{ x: 0, y: 0 }, { x: .05, y: 0 }, { x: .05, y: 1 }, { x: 0, y: 1 }] }]).solid;
  const thinWall = validateSingleOuterSolid([
    { type: 'polyline', closed: true, points: [{ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 10, y: 10 }, { x: 0, y: 10 }] },
    { type: 'polyline', closed: true, points: [{ x: .05, y: .05 }, { x: 9.95, y: .05 }, { x: 9.95, y: 9.95 }, { x: .05, y: 9.95 }] },
  ]).solid;
  expectCode(() => prepareSolidTriangulation(squareSolid(), () => [0, 1, 2, 0, 2, 3]), 'DXF_UNCALIBRATED_GEOMETRY_FLOOR');
  expectCode(() => prepareSolidTriangulation(thin, () => [], { geometryFloor: .1 }), 'DXF_FEATURE_BELOW_GEOMETRY_FLOOR');
  expectCode(() => prepareSolidTriangulation(thinWall, () => [], { geometryFloor: .1 }), 'DXF_FEATURE_BELOW_GEOMETRY_FLOOR');
});
test('initial-overlap contract rejects every positive area by default and only accepts an explicit kernel tolerance', () => {
  assert.deepEqual(assertNoInitialOverlap(0), { intersectionArea: 0, kernelAreaTolerance: 0 });
  expectCode(() => assertNoInitialOverlap(2e-12), 'DXF_INITIAL_SOLID_OVERLAP');
  assert.equal(assertNoInitialOverlap(2e-12, { kernelAreaTolerance: 2e-12 }).kernelAreaTolerance, 2e-12);
});
test('exact penetration is hole-aware and keeps the production 80 um gate as a constant', () => {
  assert.equal(PENETRATION_GATE_METRES, 0.00008); const pose = { position: { x: 0, y: 0 }, angle: 0 };
  const outer = [{ x: 0, y: 0 }, { x: 4, y: 0 }, { x: 4, y: 4 }, { x: 0, y: 4 }]; const hole = [{ x: 1, y: 1 }, { x: 3, y: 1 }, { x: 3, y: 3 }, { x: 1, y: 3 }];
  assert.equal(exactSolidRectanglePenetration([outer], pose, { position: { x: 2, y: 2 }, angle: 0 }, { x: .5, y: .5 }), 1.5);
  assert.equal(exactSolidRectanglePenetration([outer, hole], pose, { position: { x: 2, y: 2 }, angle: 0 }, { x: .25, y: .25 }), 0);
});
test('contact mapping does not confuse geometric impulse indexes with solver witness indexes', () => {
  const mapped = matchGeometricContactsToSolver([{ midpoint: { x: 0, y: 0 } }, { midpoint: { x: 2, y: 0 } }], [-.01, -.02], [{ point: { x: 2, y: 0 }, distance: -.02 }, { point: { x: 0, y: 0 }, distance: -.01 }], 1e-9);
  assert.equal(mapped.geometricToSolver.get(0), 1); assert.equal(mapped.geometricToSolver.get(1), 0); assert.deepEqual(mapped.unmatchedSolverContacts, []);
});
test('contact mapper is fail-closed for unmatched and ambiguous witnesses', () => {
  const geometric = [{ midpoint: { x: 0, y: 0 } }]; const solver = [{ point: { x: 0, y: 0 }, distance: 0 }];
  expectContractCode(() => matchGeometricContactsToSolver(geometric, [0], [], 1e-9), 'GEOMETRY_SOLVER_CONTACT_UNMATCHED');
  assert.deepEqual(matchGeometricContactsToSolver(geometric, [0], [], { tolerance: 1e-9, failClosed: false }).ok, false);
  const ambiguous = matchGeometricContactsToSolver([...geometric, ...geometric], [0, 0], [...solver, ...solver], { tolerance: 1e-9, failClosed: false });
  assert.deepEqual({ ok: ambiguous.ok, code: ambiguous.code }, { ok: false, code: 'GEOMETRY_SOLVER_CONTACT_AMBIGUOUS' });
  expectContractCode(() => matchGeometricContactsToSolver([...geometric, ...geometric], [0, 0], [...solver, ...solver], 1e-9), 'GEOMETRY_SOLVER_CONTACT_AMBIGUOUS');
});
test('P0: only the literal own boolean false can opt into a soft contact result', () => {
  const geometric = [{ midpoint: { x: 0, y: 0 } }];
  const mustThrow = [
    ['missing field', {}],
    ['undefined field', { failClosed: undefined }],
    ['null field', { failClosed: null }],
    ['zero field', { failClosed: 0 }],
    ['empty-string field', { failClosed: '' }],
    ['true field', { failClosed: true }],
  ];
  mustThrow.forEach(([label, options]) => assert.throws(
    () => matchGeometricContactsToSolver(geometric, [0], [], options),
    (error) => error instanceof GeometryContractError,
    `${label} must remain fail-closed`,
  ));
  const invalidOptions = [null, true, '', [], new Date()];
  invalidOptions.forEach((options) => expectContractCode(
    () => matchGeometricContactsToSolver(geometric, [0], [], options),
    'GEOMETRY_SOLVER_CONTACT_INVALID_OPTIONS',
  ));
  expectContractCode(() => matchGeometricContactsToSolver(geometric, [0], [], 0), 'GEOMETRY_SOLVER_CONTACT_UNMATCHED');
  const accessor = {}; Object.defineProperty(accessor, 'failClosed', { get: () => false });
  expectContractCode(() => matchGeometricContactsToSolver(geometric, [0], [], accessor), 'GEOMETRY_SOLVER_CONTACT_INVALID_OPTIONS');
  const soft = matchGeometricContactsToSolver(geometric, [0], [], { failClosed: false });
  assert.deepEqual({ ok: soft.ok, code: soft.code }, { ok: false, code: 'GEOMETRY_SOLVER_CONTACT_UNMATCHED' });
});
