/* BITE LAB — browser-only 2D engineering MVP. Internal units: m, kg, s, N. */
(() => {
  'use strict';

  const $ = (selector) => document.querySelector(selector);
  const $$ = (selector) => [...document.querySelectorAll(selector)];
  const FIXED_DT = 0.0005;
  const MAX_SIM_TIME = 4.0;
  const TRAJECTORY_CHUNK_BUDGET_MS = 32;
  const TRAJECTORY_CHUNK_MAX_TICKS = 64;
  const GRAVITY = 9.80665;
  // Contact cooldown is part of the solver, not a UI-mode switch.  Every
  // entered parameter remains active while its editor is collapsed.
  const TOOTH_CONTACT_COOLDOWN = 0.010;
  const TARGET_CONTACT_COOLDOWN = 0.080;
  const FLOOR_SKIN = 0.0001;
  // CAD placement validation is stricter than the solver's contact skin: an
  // entered weapon/fork must not visibly cross the fixed arena floor.
  const GEOMETRY_CLEARANCE_EPS = 0.000001;
  // CAD collision samples are taken directly from the joined DXF outer
  // boundary.  This is a maximum chord length, not an artificial tip radius.
  const CAD_COLLISION_CHORD = 0.0012;
  // A small positional separation keeps a high-speed CAD edge visibly outside
  // a target after a time-of-impact event. It is deliberately much smaller
  // than any UI geometry dimension and is not reported as material bite.
  const CCD_POSITION_SLOP = 0.00015;
  // Numerical guard only. Reaching it stops the run with a solver-domain
  // warning; it never permits an untested residual motion segment.
  const MAX_CCD_IMPACTS_PER_STEP = 8;
  const RAPIER_FLOOR_HALF_THICKNESS = 0.04;
  const RAPIER_FLOOR_HALF_WIDTH = 50;
  // The production path is a constrained Rapier rigid-body rig: the chassis
  // has one horizontal DOF, while the independent fork and CAD weapon connect
  // through revolute joints with their entered/derived mass properties.
  const USE_RAPIER_RIG = true;
  // Production safety gate. The traceable material-removal branch still needs
  // a transactional rollback -> earliest compressive CAD/MultiPolygon TOI ->
  // coupled solve -> remainder replay before it can make quantitative cutting
  // claims. Until that verification is complete, imported CAD keeps Rapier's
  // non-penetrating boundary and reports zero removed material.
  const TRACEABLE_CUTTING_CONTINUOUS_TOI_ENABLED = false;
  // Convergence resolution, not a response coefficient. Rotational CCD is
  // resolved by limiting the CAD boundary's arc travel in every solve.
  // Initial rotational/linear boundary travel per Rapier substep. This is a
  // convergence seed, not an allowed penetration: the hard 0.08 mm geometry
  // gate below reruns the whole fixed tick at 2× resolution when necessary.
  const RIG_INITIAL_CAD_SWEEP = 0.0003;
  // Rapier 0.20 limits a single dynamic integration step to roughly π/4 of
  // rotation. Subdivide globally (not only near a collider) so requested RPM
  // is never silently truncated before the weapon reaches the target.
  const RAPIER_MAX_ROTATION_PER_STEP = Math.PI / 4;
  // Computational budget only: this does not loosen any contact tolerance.
  // The 0.59:1 / 180 deg regression needs 6720 local steps for one energetic
  // contact tick to converge while retaining the same 0.08 mm geometry gate.
  const RIG_MAX_SUBSTEPS = 8192;
  // Bounded dyadic rollback depth for a fixed tick.  An acute parametric tooth
  // can need one more exact-boundary solve after a 256-step trial lands just
  // outside the unchanged 0.08 mm gate; 10 levels still remain subject to the
  // absolute 8192-substep budget above and never accept a rejected pose.
  const RIG_MAX_REFINEMENT = 10;
  // All physics coordinates are already SI metres, therefore Rapier's
  // units-per-metre scale must stay at 1. Precision is configured explicitly
  // below; shrinking lengthUnit would also shrink corrective velocities and
  // can make energetic contacts appear unable to converge.
  const RAPIER_LENGTH_UNIT = 1;
  const RAPIER_ALLOWED_LINEAR_ERROR = 0.00002;
  const RAPIER_PREDICTION_DISTANCE = 0.00006;
  const RIG_CAD_PENETRATION_TOLERANCE = 0.00008;
  // Weapon/fork colliders use the exact CAD boundary. Predictive contact may
  // still be generated within one resolved CAD sweep, but there is no expanded
  // 0.2 mm virtual tool geometry.
  const RIG_CONTACT_SKIN = 0;
  const RIG_CONTACT_EVENT_MAX_GAP = RAPIER_PREDICTION_DISTANCE + RAPIER_ALLOWED_LINEAR_ERROR;
  const RIG_FLOOR_PENETRATION_TOLERANCE = 0.00008;
  // Operator-splitting error guard for material cutting.  The contact force is
  // applied at the end of each Rapier substep, so the ballistic CAD sweep is a
  // slight overestimate of the distance travelled while the tool decelerates.
  // Refine the whole fixed tick until that kinematic over-travel is no larger
  // than Rapier's configured 0.02 mm linear accuracy.  This is a numerical
  // convergence criterion, not an empirical cutting/energy coefficient.
  const MATERIAL_PATH_INTEGRATION_TOLERANCE = RAPIER_ALLOWED_LINEAR_ERROR;
  // Maximum midpoint deviation between the continuous rigid pose and the
  // piecewise-linear swept boundary used by polygon clipping. This is a purely
  // geometric discretisation error (5 µm at the default SI scale), not a
  // material coefficient or an engagement factor.
  const MATERIAL_SWEEP_GEOMETRY_TOLERANCE = RAPIER_ALLOWED_LINEAR_ERROR * .25;
  const MATERIAL_SWEEP_MAX_RECURSION = 10;
  // Result-validity thresholds for the explicit mechanical-energy ledger.
  // They never change an impulse or material parameter; an episode outside
  // them is labelled numerically unconverged instead of being presented as a
  // quantitative engineering result.
  const ENERGY_LEDGER_ABSOLUTE_TOLERANCE = 0.002;
  const ENERGY_LEDGER_RELATIVE_TOLERANCE = 0.001;
  const MATERIAL_TICK_RELATIVE_CONVERGENCE = 0.01;
  // Revolute-anchor drift is a constraint residual, not CAD penetration. Its
  // hard gate is relative to weapon radius (0.5%), bounded to 0.1–0.5 mm;
  // tool/floor overlap retains the independent strict 0.08 mm gate.
  const RIG_JOINT_ERROR_MIN = 0.0001;
  const RIG_JOINT_ERROR_MAX = 0.0005;
  const RIG_GROUP_TARGET = 0x0001;
  const RIG_GROUP_FLOOR = 0x0002;
  const RIG_GROUP_FORK = 0x0004;
  const RIG_GROUP_WEAPON = 0x0008;
  // The fork is an independent gravity-loaded body joined to the chassis at its
  // own CAD origin.  Target support clearance and the physical arena floor are
  // deliberately different colliders: the target rests on targetSupportY(),
  // while the fork always lands on the real scene datum Y=0.
  const UNIT_SCALES = Object.freeze({ mm: 0.001, in: 0.0254, m: 1 });
  const INSUNITS = Object.freeze({ 1: 'in', 4: 'mm', 6: 'm' });

  // Starter rotating-body mass is an editable geometry estimate, not I/R².
  // The built-in DXF contains one 3,940.6791357 mm² outer loop and twelve
  // nested closed cut-outs totalling 1,578.7716971 mm².  Even-odd net steel
  // area is therefore 2,361.9074385 mm².  At the drawing's 8 mm Z width and
  // 7,850 kg/m³, the plate estimate is 0.1483277871 kg.  Real hubs, fasteners,
  // pulleys and shafts must be included by replacing this value with a measured
  // complete rotating-assembly mass.
  const DEFAULT_WEAPON_NET_AREA_MM2 = 2361.90743852162;
  const DEFAULT_WEAPON_WIDTH_MM = 8;
  const DEFAULT_STEEL_DENSITY = 7850;
  const DEFAULT_WEAPON_MASS = DEFAULT_WEAPON_NET_AREA_MM2 * 1e-6
    * (DEFAULT_WEAPON_WIDTH_MM / 1000) * DEFAULT_STEEL_DENSITY;

  const DEFAULTS = Object.freeze({
    robotPreset: '1.36kg',
    dxfUnit: 'mm',
    // With the supplied fork mirrored in X, its real CAD nose is offsetX + 72 mm.
    // -40 mm leaves the actual one-tooth profile room to reach a floor target.
    shovelOffsetX: 0,
    shovelOffsetY: -53.25,
    // Measured moving-fork mass. Its COM and inertia ratio come from the active
    // CAD solid, or from the uniform slender-rod formula for the ideal line fork.
    forkMass: 0.03,
    shovelMirrorX: true,
    shovelMirrorY: false,
    weaponMirrorX: true,
    weaponMirrorY: false,
    paramWeaponEnabled: false,
    paramForkEnabled: false,
    paramToothCount: 1,
    paramToothPhaseDeg: 0,
    // Parametric tooth: impact-circle radius R and working tooth-line length L.
    // "Phase" follows the user's document: signed rake angle, not clocking.
    paramToothLength: 55,
    paramToothWorkLength: 14,
    paramToothWidth: 6,
    // Ideal zero-thickness fork segment: fixed hinge O=(0,0) to tip T=(D,0).
    paramForkTipDistance: 72,
    // Only the trajectory horizon changes; the physical fixed step remains
    // 0.5 ms. This keeps shorter engineering runs physically identical to the
    // corresponding prefix of a 4 s run.
    simulationDuration: 4,
    weaponInitialAngle: 0,
    tipRadius: 55,
    toothCount: 1,
    // Backward-compatible public/readout field. Its value is overwritten from
    // weaponMass and the active CAD/parameter solid on every metrics refresh.
    weaponInertia: 160,
    // UI unit: kg. This is the sole editable weapon mass-property input.
    weaponMass: DEFAULT_WEAPON_MASS,
    weaponMotorPreset: 'dual2306',
    // Default engineering run starts from zero so motor power/current limits
    // determine how much energy exists at first contact.  "setSpeed" remains
    // available as an explicit already-spun boundary condition.
    weaponStartMode: 'realAcceleration',
    weaponThrottle: 100,
    weaponKv: 1750,
    weaponMotorCount: 2,
    weaponMotorPower: 250,
    weaponMotorCurrent: 18,
    weaponMotorEfficiency: 85,
    weaponOutputFactor: 75,
    voltage: 14.8,
    weaponGearRatio: 2.1,
    weaponEfficiency: 90,
    weaponSceneX: 0,
    // Default axis height = effective weapon radius × 1.05 above fixed floor Y=0.
    weaponSceneY: 57.75,
    driveMotorPreset: 'dualBlitzLite1604Dyno4S',
    driveStartMode: 'realAcceleration',
    driveThrottle: 100,
    driveKv: 2850,
    driveMotorCount: 2,
    // Just 'Cuz publishes 4S dynamometer data for the complete 29.16:1
    // Blitz Lite assembly: 0.84 N·m peak torque before stall and 70 W peak
    // mechanical output per gearmotor.  The solver's torque input is located
    // before its explicit ratio, so 0.84 / 29.16 is the exact output-referred
    // reparameterisation.  It is not claimed as an independently measured
    // bare-rotor stall torque.
    driveMotorPower: 70,
    driveMotorTorque: 0.028807,
    // Zero means no verified phase-current limit; the entered/measured torque
    // remains authoritative. A positive value adds a Kt-derived advanced cap.
    driveMotorCurrent: 0,
    wheelDiameter: 43.2,
    driveGearRatio: 29.16,
    // The cited torque and power are already measured at the gearbox output;
    // 100% here prevents applying a second, invented gearbox-loss discount.
    driveEfficiency: 100,
    robotMass: 1.36,
    slipFactor: 5,
    gripCoefficient: 0.8,
    rollingResistanceCoefficient: 0.02,
    weaponMaterial: 'hardox500',
    shovelMaterial: 'hardox500',
    targetMaterial: 'hdpe',
    // Production default: run the already-validated Rapier non-penetrating
    // rigid upper-bound path. Finite-material mode remains an explicit research
    // selection and still fails closed when its continuous response is absent;
    // the application never switches an operator-selected material run silently.
    contactModel: 'rigid',
    floorMaterial: 'osb',
    frictionFactor: 1,
    floorSceneY: 0,
    targetSceneX: 380,
    targetLength: 140,
    targetThickness: 16,
    targetWidthZ: 40,
    targetMass: 0.0861056,
    targetMassMode: 'geometry',
    targetDensity: 961,
    targetYoungModulus: 1.4,
    targetYieldStrength: 25,
    targetShearStrength: 14.43,
    // Gent & Wang (1996) measured about 4 kJ/m² for HDPE cutting/tearing.
    // This is a traceable quasi-static reference, not a fitted impact factor.
    targetFractureEnergy: 4000,
    targetFractureSource: 'Gent & Wang 1996 普通 HDPE 准静态切割（跨牌号/速率参考）',
    // Minimum undeformed chip thickness is not a universal fraction of edge
    // radius. Zero/blank means no traceable same-tool/material/rate test is
    // available, so the model may plough but may not claim chip removal.
    targetMinChipThickness: 0,
    targetMinChipSource: '',
    weaponWidthZ: DEFAULT_WEAPON_WIDTH_MM,
    shovelWidthZ: 4,
    weaponZOffset: 0,
    shovelZOffset: 0,
    edgeRadius: 0.2,
    targetClearance: 0.1,
    restitution: 0.04,
    impactEfficiency: 3,
    timeScale: 1,
  });

  const MATERIALS = Object.freeze({
    mildSteel: {
      label: 'S235JR 低碳钢板', short: 'S235JR', color: '#5da9ff', restitution: 0.16, friction: 0.28,
      density: 7850, youngModulus: 210, yieldStrength: 235, shearStrength: 235 / Math.sqrt(3),
      fractureEnergy: 0, fractureSource: null,
      model: 'merchant', source: 'Tata Steel S235JR；E/密度为通用钢工程假设', validity: '静态强度；高速应变率与断裂能未给定',
    },
    aluminum: {
      label: '6061-T6/T651 铝板', short: '6061-T6', color: '#c1d1e8', restitution: 0.30, friction: 0.34,
      density: 2700, youngModulus: 68.3, yieldStrength: 276, shearStrength: 276 / Math.sqrt(3),
      fractureEnergy: 0, fractureSource: null,
      model: 'merchant', source: 'Kaiser Aluminum 6061 板材数据', validity: '静态 T6/T651；高速外推需试验',
    },
    hdpe: {
      label: 'ExxonMobil HYA 900 HDPE', short: 'HDPE', color: '#75d9d3', restitution: 0.52, friction: 0.38,
      density: 961, youngModulus: 1.4, yieldStrength: 25, shearStrength: 25 / Math.sqrt(3),
      fractureEnergy: 4000,
      fractureSource: 'Gent & Wang 1996 普通 HDPE 准静态切割约 4 kJ/m²（跨牌号/速率参考）',
      model: 'merchant', source: 'ExxonMobil HYA 900 数据表；Gc 采用 Gent & Wang 1996 的普通 HDPE 参考', validity: 'Gc 跨牌号且为准静态参考；高速应变率、热软化与本牌号断裂试验未建模',
    },
    wood: {
      label: '结构胶合板（方向待定）', short: '胶合板', color: '#d7a46b', restitution: 0.27, friction: 0.42,
      density: 600, youngModulus: 10, yieldStrength: 20.7, shearStrength: 4.1,
      fractureEnergy: 0, fractureSource: null,
      model: 'shear', source: 'USDA Wood Handbook 一般范围下界', validity: '各向异性；未给纹理方向/含水率/Gc，仅作剪切下界估算',
    },
    custom: {
      label: '自定义材料', short: '自定义', color: '#bca6ff', restitution: 0.25, friction: 0.32,
      density: 1000, youngModulus: 1, yieldStrength: 10, shearStrength: 5.77,
      fractureEnergy: 0, fractureSource: null,
      model: 'merchant', source: '用户输入', validity: '需用户提供可追溯的材料与速率条件',
    },
  });

  // Dry, clean, room-temperature engineering estimates only. They limit tangential
  // impulse / sliding; they are deliberately not used as a cutting or failure model.
  const WEAPON_MATERIALS = Object.freeze({
    hardox500: { label: 'Hardox 500', restitution: 0.15, youngModulus: 210, yieldStrength: 1400, source: 'SSAB；屈服为典型值，非保证值' },
    hardox600: { label: 'Hardox 600', restitution: 0.12, youngModulus: 210, yieldStrength: 1650, source: 'SSAB；E 为通用钢假设' },
    peek: { label: 'Victrex 450G PEEK', restitution: 0.35, youngModulus: 4.0, yieldStrength: 98, source: 'Victrex 450G 数据表' },
    plaPlus: { label: 'eSUN PLA+（XY）', restitution: 0.25, youngModulus: 2.888, yieldStrength: 53.34, source: 'eSUN TDS；打印方向显著影响结果' },
    tc4: { label: 'TC4 / Ti-6Al-4V Grade 5', restitution: 0.18, youngModulus: 114, yieldStrength: 828, source: 'TIMET TIMETAL 6-4' },
    custom: { label: '自定义', restitution: 0.25, youngModulus: 210, yieldStrength: 500, source: '用户输入/待核验' },
  });
  const FLOOR_MATERIALS = Object.freeze({
    osb: { label: '欧松板（OSB）', restitution: 0.16 },
    steelPlate: { label: '钢板', restitution: 0.28 },
    aluminumPlate: { label: '铝板', restitution: 0.23 },
    custom: { label: '自定义', restitution: 0.20 },
  });
  const WEAPON_TARGET_MU = Object.freeze({
    hardox500: { mildSteel: .36, aluminum: .40, hdpe: .24, wood: .43, custom: .35 },
    hardox600: { mildSteel: .34, aluminum: .38, hdpe: .22, wood: .41, custom: .33 },
    peek: { mildSteel: .24, aluminum: .26, hdpe: .19, wood: .30, custom: .28 },
    plaPlus: { mildSteel: .40, aluminum: .44, hdpe: .30, wood: .48, custom: .36 },
    tc4: { mildSteel: .33, aluminum: .37, hdpe: .27, wood: .42, custom: .35 },
    custom: { mildSteel: .32, aluminum: .35, hdpe: .30, wood: .38, custom: .32 },
  });
  const TARGET_FLOOR_CONTACT = Object.freeze({
    mildSteel: { osb: { static: .55, kinetic: .42 }, steelPlate: { static: .45, kinetic: .32 }, aluminumPlate: { static: .40, kinetic: .30 } },
    aluminum: { osb: { static: .50, kinetic: .38 }, steelPlate: { static: .42, kinetic: .30 }, aluminumPlate: { static: .48, kinetic: .36 } },
    hdpe: { osb: { static: .30, kinetic: .22 }, steelPlate: { static: .24, kinetic: .17 }, aluminumPlate: { static: .26, kinetic: .19 } },
    wood: { osb: { static: .55, kinetic: .42 }, steelPlate: { static: .38, kinetic: .29 }, aluminumPlate: { static: .42, kinetic: .32 } },
    custom: { osb: { static: .45, kinetic: .34 }, steelPlate: { static: .35, kinetic: .26 }, aluminumPlate: { static: .37, kinetic: .28 } },
  });

  // “2306 / 2212 …” normally describes a stator envelope, not a unique motor.
  // Values that cite a vendor below retain the vendor's stated test condition;
  // belt reduction, output factor and use in a spinner remain user assumptions.
  const WEAPON_MOTOR_PRESETS = Object.freeze({
    dual2306: {
      label: '双 P2306 1750KV（4S 用户降额）',
      weaponKv: 1750, weaponMotorCount: 2, weaponMotorPower: 250, weaponMotorCurrent: 18, weaponMotorEfficiency: 85, weaponOutputFactor: 75, voltage: 14.8, weaponGearRatio: 2.1,
      source: 'T-Motor P2306 V3 1750KV', sourceUrl: 'https://store.tmotor.com/product/p2306-v3-fpv-motor.html',
      condition: '250 W/台、模型相电流18 A/台是4S用户降额情景，不是厂家桨测点。厂家6S表中的34.8 A是电池侧输入，不能直接替代模型相电流；传动和效率仍需实测。',
    },
    singleP2306Official: {
      label: '单 P2306 V3 1750KV（6S官方电输入）',
      weaponKv: 1750, weaponMotorCount: 1, weaponMotorPower: 864.3, weaponMotorCurrent: 18, weaponMotorEfficiency: 85, weaponOutputFactor: 75, voltage: 24.8, weaponGearRatio: 2.1,
      source: 'T-Motor P2306 V3 1750KV', sourceUrl: 'https://store.tmotor.com/product/p2306-v3-fpv-motor.html',
      condition: 'T5143S-3满油门表为24.8 V、34.8 A电池侧、864.3 W电输入、31277 rpm，规格为60 s上限；轴功率与相电流未给出，故模型相电流保留18 A可编辑假设。',
    },
    dualP2306Official: {
      label: '双 P2306 V3 1750KV（6S表值×2）',
      weaponKv: 1750, weaponMotorCount: 2, weaponMotorPower: 864.3, weaponMotorCurrent: 18, weaponMotorEfficiency: 85, weaponOutputFactor: 75, voltage: 24.8, weaponGearRatio: 2.1,
      source: '两台 T-Motor P2306 V3 1750KV', sourceUrl: 'https://store.tmotor.com/product/p2306-v3-fpv-motor.html',
      condition: '每台采用厂家24.8 V、864.3 W电输入表值，两台合计电输入1728.6 W；厂家没有双电机武器台架数据，电池、ESC、散热、轴功率及同轴耦合均须实测。模型相电流仍保留18 A/台假设。',
    },
    velox2306Conflict: {
      label: 'V2306 1500KV“1969.9 W”（冲突，仅查看）', apply: false,
      source: 'T-Motor VELOX V2306 V3 1500KV', sourceUrl: 'https://store.tmotor.com/product/v2306-v3-kv1750-fpv-motor.html',
      condition: '官网规格栏把1969.9写作W，但同页F5146-3表中1969.9实际是推力g；31.5 V×34.5 A≈1086.8 W，功率栏为1088.2 W。此项不套用任何数值，避免把排版冲突当成机械轴功率。',
    },
    single2212: {
      label: '单 X2212 V3 1400KV（4S 条件）',
      weaponKv: 1400, weaponMotorCount: 1, weaponMotorPower: 509.12, weaponMotorCurrent: 20, weaponMotorEfficiency: 80, weaponOutputFactor: 70, voltage: 14.8, weaponGearRatio: 2.5,
      source: 'SunnySky X2212 V3 1400KV', sourceUrl: 'https://sunnyskyusa.com/collections/x-v3-motors/products/sunnsky-x2212',
      condition: 'APC7060的4S表格点为14.8 V、34.4 A电池侧、509.12 W电输入、16243 rpm；目录另列37 A/30 s和560 W上限。模型相电流20 A并非该电池电流，仍需实测。',
    },
    single2812: {
      label: '单 XRotor 2812 900KV（6S / 9 s 上限）',
      weaponKv: 900, weaponMotorCount: 1, weaponMotorPower: 1191, weaponMotorCurrent: 30, weaponMotorEfficiency: 80, weaponOutputFactor: 70, voltage: 24, weaponGearRatio: 3.2,
      source: 'Hobbywing XRotor 2812 900KV', sourceUrl: 'https://www.hobbywing.com/en/products/xrotor2812',
      condition: '厂商规格为 4–6S、49.6 A / 9 s、1191 W / 9 s；并非持续功率。模型相电流默认 30 A；3.2:1、效率和输出系数均为可编辑假设。',
    },
    single2807: {
      label: '单 NIDICI 2807 1300KV（24V目录上限）',
      weaponKv: 1300, weaponMotorCount: 1, weaponMotorPower: 959.9, weaponMotorCurrent: 20, weaponMotorEfficiency: 80, weaponOutputFactor: 70, voltage: 24, weaponGearRatio: 3.2,
      source: 'iFlight NIDICI 2807 1300KV', sourceUrl: 'https://shop.iflight.com/NIDICI-2807-FPV-Motor-Pro2314',
      condition: '厂家页列24 V、40.28 A峰值电池侧输入、959.9 W最大电输入，但未公开对应桨、转速、环境或持续时间。模型相电流20 A只是可编辑假设，不能把目录电池电流直接套入Kt。',
    },
    single3115: {
      label: '单 XTO-3115 900KV（6S 条件上限）',
      weaponKv: 900, weaponMotorCount: 1, weaponMotorPower: 1915.9, weaponMotorCurrent: 50, weaponMotorEfficiency: 80, weaponOutputFactor: 70, voltage: 23, weaponGearRatio: 3.2,
      source: 'X-TEAM XTO-3115 系列', sourceUrl: 'https://www.x-teamrc.com/product/xto-3115-brushless-motor-for-multi-rotor-aerial-drone/',
      condition: '官方900KV、10.5×5-3、100%表格点为23.0 V、83.3 A电池侧、1915.9 W电输入、13478 rpm，目录短时上限10 s。模型相电流50 A不是电池电流，轴功率仍须台架验证。',
    },
    custom: { label: '自定义', source: '用户输入', condition: '请按具体电机型号、冷却条件、ESC 相电流与传动实测填写。' },
  });

  const DRIVE_MOTOR_PRESETS = Object.freeze({
    dualBlitzLite1604Dyno4S: {
      label: '双 Blitz Lite 1604 2850KV + 29.16:1（4S 官方测功）',
      voltage: 14.8, driveKv: 2850, driveMotorCount: 2, driveMotorPower: 70,
      driveMotorTorque: 0.028807, driveMotorCurrent: 0, wheelDiameter: 43.2,
      driveGearRatio: 29.16, driveEfficiency: 100,
      source: "Just 'Cuz Robotics Blitz Lite 1604 官方产品页与厂家测功表",
      sourceUrl: 'https://justcuzrobotics.com/products/blitz-lite-1604',
      dataUrl: 'https://docs.google.com/spreadsheets/d/18LwgSo5hn4VzsO00wOYi4daVVrfIAYgCwupOJGyJhQo/edit?gid=0',
      condition: '厂家明确称两套可用于 3 lb 机器人；14.8 V 测功点为整套减速电机 1450 rpm、0.84 N·m 停转前峰值扭矩、70 W 峰值机械输出。模型的 0.028807 N·m 约等于 0.84/29.16，只是为了在显式减速比前重现实测轮端扭矩，不是裸电机独立台架值；效率设 100% 是避免对已包含齿轮损耗的输出数据再折损。43.2 mm 轮径来自同厂 SSP 轮子目录，未参与该测功；场地抓地系数仍须实测。16.4 A 是台架电源侧峰值，未填入相电流上限。',
    },
    repeatCompact1806Catalog: {
      label: 'Repeat Compact 1806 2300KV + 22.6:1（官方目录，扭矩未知）',
      apply: false,
      source: 'Repeat Robotics Compact Brushless 1806 官方产品页',
      sourceUrl: 'https://repeat-robotics.com/products/repeat-compact-1806',
      condition: '可核验目录值只有 1806-2300KV、22.6:1、4S 1500 rpm 空载输出和建议 20A+ ESC；厂家未给出停转扭矩、轴功率或相电流。ESC 额定不是电机相电流台架值，因此该项只供查看，不覆盖当前可计算输入。',
    },
    dual1806: {
      label: '双 1806 3000KV + 20:1（用户候选，待核验）',
      driveKv: 3000, driveMotorCount: 2, driveMotorPower: 80, driveMotorTorque: 0.044, wheelDiameter: 38, driveGearRatio: 20, driveEfficiency: 82, gripCoefficient: 0.8,
      source: '用户给定的 1.36 kg 候选',
      condition: '尚未绑定到可核验的厂家型号、绕组或数据表。4S、80 W、0.044 N·m、20:1 与 38 mm 轮均仅为可编辑演示条件；请用实测扭矩/相电流替换，不能用于选型或安全结论。',
    },
    micro1103: {
      label: '双 1103（尺寸级示例，待实测）',
      driveKv: 8000, driveMotorCount: 2, driveMotorPower: 18, driveMotorTorque: 0.006, wheelDiameter: 22, driveGearRatio: 24, driveEfficiency: 78, gripCoefficient: 0.75,
      source: '尺寸级通用输入', condition: '1103 不是唯一型号；KV、功率、扭矩、减速比和抓地均为演示输入，需由实际传动与轮胎测试覆盖。',
    },
    beetle1404: {
      label: '双 1404（尺寸级示例，待实测）',
      driveKv: 5000, driveMotorCount: 2, driveMotorPower: 40, driveMotorTorque: 0.015, wheelDiameter: 32, driveGearRatio: 24, driveEfficiency: 80, gripCoefficient: 0.8,
      source: '尺寸级通用输入', condition: '1404 不是唯一型号；KV、功率、扭矩、减速比和抓地均为演示输入，需由实际传动与轮胎测试覆盖。',
    },
    heavy6374: {
      label: '四 6374（尺寸级示例，待实测）',
      driveKv: 190, driveMotorCount: 4, driveMotorPower: 2000, driveMotorTorque: 2.6, wheelDiameter: 150, driveGearRatio: 12, driveEfficiency: 88, gripCoefficient: 1.05,
      source: '尺寸级通用输入', condition: '6374 不是唯一型号；大功率行驶还受电池、ESC、热管理、胎面和赛场抓地限制。所有数值均需按实际硬件覆盖。',
    },
    custom: { label: '自定义', source: '用户输入', condition: '请优先填写实测/厂家数据表扭矩，再检查轮端牵引力是否被抓地上限限制。' },
  });

  const ROBOT_PRESETS = Object.freeze({
    '150g': { label: '150 g 微型', robotMass: 0.15, voltage: 7.4, weaponMotorPreset: 'custom', weaponKv: 8000, weaponMotorCount: 1, weaponMotorPower: 35, weaponMotorCurrent: 6, weaponMotorEfficiency: 78, weaponOutputFactor: 70, weaponGearRatio: 1.8, weaponInertia: 5, tipRadius: 25, toothCount: 1, driveMotorPreset: 'micro1103', driveKv: 8000, driveMotorCount: 2, driveMotorPower: 18, driveMotorTorque: 0.006, driveGearRatio: 24, driveEfficiency: 78, wheelDiameter: 22, targetMaterial: 'hdpe', targetLength: 60, targetThickness: 6, targetMass: 0.0051894, targetMassMode: 'geometry', targetCenterY: 0, restitution: 0.5 },
    '220g': { label: '220 g 蚂蚁', robotMass: 0.22, voltage: 7.4, weaponMotorPreset: 'custom', weaponKv: 6500, weaponMotorCount: 1, weaponMotorPower: 55, weaponMotorCurrent: 8, weaponMotorEfficiency: 80, weaponOutputFactor: 72, weaponGearRatio: 1.7, weaponInertia: 8, tipRadius: 28, toothCount: 1, driveMotorPreset: 'micro1103', driveKv: 6500, driveMotorCount: 2, driveMotorPower: 22, driveMotorTorque: 0.007, driveGearRatio: 22, driveEfficiency: 79, wheelDiameter: 24, targetMaterial: 'hdpe', targetLength: 70, targetThickness: 8, targetMass: 0.0107632, targetMassMode: 'geometry', targetCenterY: 0, restitution: 0.5 },
    '454g': { label: '454 g 甲虫', robotMass: 0.454, voltage: 11.1, weaponMotorPreset: 'custom', weaponKv: 3950, weaponMotorCount: 1, weaponMotorPower: 120, weaponMotorCurrent: 13, weaponMotorEfficiency: 82, weaponOutputFactor: 74, weaponGearRatio: 2.5, weaponInertia: 40, tipRadius: 40, toothCount: 1, driveMotorPreset: 'beetle1404', driveKv: 5000, driveMotorCount: 2, driveMotorPower: 40, driveMotorTorque: 0.015, driveGearRatio: 24, driveEfficiency: 80, wheelDiameter: 32, targetMaterial: 'hdpe', targetLength: 100, targetThickness: 10, targetMass: 0.02883, targetMassMode: 'geometry', targetCenterY: 0, restitution: 0.45 },
    '1.36kg': { label: '1.36 kg 羽量', robotMass: 1.36, voltage: 14.8, weaponMotorPreset: 'dual2306', weaponKv: 1750, weaponMotorCount: 2, weaponMotorPower: 250, weaponMotorCurrent: 18, weaponMotorEfficiency: 85, weaponOutputFactor: 75, weaponGearRatio: 2.1, weaponInertia: 160, weaponMass: DEFAULT_WEAPON_MASS, tipRadius: 55, toothCount: 1, driveMotorPreset: 'dualBlitzLite1604Dyno4S', driveKv: 2850, driveMotorCount: 2, driveMotorPower: 70, driveMotorTorque: 0.028807, driveMotorCurrent: 0, driveGearRatio: 29.16, driveEfficiency: 100, wheelDiameter: 43.2, targetMaterial: 'hdpe', targetLength: 140, targetThickness: 16, targetMass: 0.0861056, targetMassMode: 'geometry', targetCenterY: 0, restitution: 0.22 },
    '5lb': { label: '5 lbs / 2.27 kg（4S 示例）', robotMass: 2.268, voltage: 14.8, weaponMotorPreset: 'single2212', weaponKv: 1400, weaponMotorCount: 1, weaponMotorPower: 509.12, weaponMotorCurrent: 20, weaponMotorEfficiency: 80, weaponOutputFactor: 70, weaponGearRatio: 2.5, weaponInertia: 2500, tipRadius: 75, toothCount: 1, driveMotorPreset: 'custom', driveKv: 1750, driveMotorCount: 2, driveMotorPower: 140, driveMotorTorque: 0.09, driveGearRatio: 22, driveEfficiency: 83, wheelDiameter: 50, targetMaterial: 'aluminum', targetLength: 190, targetThickness: 20, targetMass: 0.513, targetMassMode: 'geometry', targetCenterY: 0, restitution: 0.3 },
    '13.6kg': { label: '13.6 kg Feather（6S / 9 s 上限）', robotMass: 13.6, voltage: 24, weaponMotorPreset: 'single2812', weaponKv: 900, weaponMotorCount: 1, weaponMotorPower: 1191, weaponMotorCurrent: 30, weaponMotorEfficiency: 80, weaponOutputFactor: 70, weaponGearRatio: 3.2, weaponInertia: 25000, tipRadius: 130, toothCount: 1, driveMotorPreset: 'custom', driveKv: 1300, driveMotorCount: 2, driveMotorPower: 280, driveMotorTorque: 0.21, driveGearRatio: 19, driveEfficiency: 85, wheelDiameter: 75, targetMaterial: 'mildSteel', targetLength: 350, targetThickness: 30, targetMass: 6.594, targetMassMode: 'geometry', targetCenterY: 0, restitution: 0.16 },
    '110kg': { label: '110 kg Heavyweight（参数几何示例）', robotMass: 110, voltage: 44.4, weaponMotorPreset: 'custom', weaponKv: 190, weaponMotorCount: 2, weaponMotorPower: 5000, weaponMotorCurrent: 130, weaponMotorEfficiency: 88, weaponOutputFactor: 78, weaponGearRatio: 3.5, weaponInertia: 550000, tipRadius: 250, toothCount: 1, paramWeaponEnabled: true, paramToothLength: 250, paramToothCount: 1, paramForkEnabled: true, paramForkTipDistance: 330, driveMotorPreset: 'heavy6374', driveKv: 190, driveMotorCount: 4, driveMotorPower: 2000, driveMotorTorque: 2.6, driveGearRatio: 12, driveEfficiency: 88, wheelDiameter: 150, targetMaterial: 'mildSteel', targetLength: 700, targetThickness: 60, targetMass: 65.94, targetMassMode: 'geometry', targetSceneX: 650, targetCenterY: 0, restitution: 0.12 },
  });

  const state = {
    params: { ...DEFAULTS },
    appliedRobotPreset: DEFAULTS.robotPreset,
    drawings: { shovel: null, weapon: null },
    metrics: null,
    sim: null,
    canvas: null,
    ctx: null,
    view: null,
    camera: { center: null, scale: null, fitScale: null, dragging: null, touchPointers: new Map(), pinch: null },
    axisPicking: false,
    advancedEnabled: false,
    showGrid: true,
    showEnvelope: true,
    lastFrame: 0,
    accumulator: 0,
    toastTimer: null,
    rapier: null,
    rapierError: null,
    rapierBackend: null,
    rapierFallbackReason: null,
    trajectory: null,
    trajectoryGeneration: 0,
    trajectoryTimer: 0,
    activeRigTick: null,
    qaRigBoundaryLimit: null,
    playheadTick: 0,
    eventEntries: [],
  };

  function migrateLegacyPublicParams(values) {
    const incoming = values && typeof values === 'object' ? { ...values } : {};
    // Older saved payloads exposed two controls for one physical fork segment.
    // Preserve their length when the current key is absent, but never keep or
    // echo either retired key in the committed/public parameter object. The old
    // angle has no equivalent: fork attitude is now solved by hinge + gravity.
    if (!Object.prototype.hasOwnProperty.call(incoming, 'paramForkTipDistance')
      && Object.prototype.hasOwnProperty.call(incoming, 'paramForkFaceLength')) {
      incoming.paramForkTipDistance = incoming.paramForkFaceLength;
    }
    delete incoming.paramForkFaceLength;
    delete incoming.paramForkAngleDeg;
    delete incoming.paramForkTipAngleDeg;
    delete incoming.paramForkWidth;
    // Since v1.0 the complete rotating mass is the sole editable mass-property
    // input. Keep accepting old payloads, but never let their stale/manual
    // inertia override the value derived from the active geometry.
    delete incoming.weaponInertia;
    return incoming;
  }

  const point = (x, y) => ({ x, y });
  const add = (a, b) => point(a.x + b.x, a.y + b.y);
  const subtract = (a, b) => point(a.x - b.x, a.y - b.y);
  const scalePoint = (p, s) => point(p.x * s, p.y * s);
  const dot = (a, b) => a.x * b.x + a.y * b.y;
  const cross = (a, b) => a.x * b.y - a.y * b.x;
  const length = (p) => Math.hypot(p.x, p.y);
  const normalise = (p) => { const l = length(p); return l > 1e-9 ? scalePoint(p, 1 / l) : point(1, 0); };
  const perpendicular = (p) => point(-p.y, p.x);
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const number = (v, fallback = 0) => Number.isFinite(Number(v)) ? Number(v) : fallback;
  const positive = (v, fallback) => Number.isFinite(Number(v)) && Number(v) > 0 ? Number(v) : fallback;
  const radians = (degrees) => degrees * Math.PI / 180;
  const lerp = (a, b, t) => point(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
  function rotate(p, angle) { const c = Math.cos(angle); const s = Math.sin(angle); return point(p.x * c - p.y * s, p.x * s + p.y * c); }
  function format(v, digits) {
    if (!Number.isFinite(v)) return '—';
    const abs = Math.abs(v);
    const places = digits ?? (abs >= 1000 ? 0 : abs >= 100 ? 1 : abs >= 10 ? 2 : 3);
    return new Intl.NumberFormat('zh-CN', { maximumFractionDigits: places }).format(v);
  }
  const setText = (id, value) => { const node = document.getElementById(id); if (node) node.textContent = value; };
  const nominalSignedOmega = () => state.metrics.simulationAngularVelocity ?? state.metrics.angularVelocity; // counter-clockwise by default
  const signedOmega = () => (state.sim && Number.isFinite(state.sim.weaponOmega) ? state.sim.weaponOmega : nominalSignedOmega());
  function decodeEmbeddedPaths(records, scale = 0.001) {
    return records.map((record) => {
      const [kind, ...values] = record;
      if (kind === 'l') return { type: 'line', start: point(values[0] * scale, values[1] * scale), end: point(values[2] * scale, values[3] * scale) };
      if (kind === 'c') return { type: 'circle', center: point(values[0] * scale, values[1] * scale), radius: values[2] * scale };
      if (kind === 'a') return { type: 'arc', center: point(values[0] * scale, values[1] * scale), radius: values[2] * scale, startAngle: radians(values[3]), endAngle: radians(values[4]) };
      if (kind === 'p') return { type: 'polyline', closed: Boolean(values[0]), points: values[1].map((p) => point(p[0] * scale, p[1] * scale)) };
      return null;
    }).filter(Boolean);
  }

  function clonePath(path) {
    const output = { ...path };
    if (path.points) output.points = path.points.map((p) => point(p.x, p.y));
    if (path.center) output.center = point(path.center.x, path.center.y);
    if (path.start) output.start = point(path.start.x, path.start.y);
    if (path.end) output.end = point(path.end.x, path.end.y);
    return output;
  }

  function samplePath(path, segments) {
    if (path.type === 'line') return [path.start, path.end];
    if (path.type === 'polyline') return path.points || [];
    if (path.type === 'circle') {
      const count = segments || 64;
      return Array.from({ length: count }, (_, i) => point(path.center.x + Math.cos(i / count * Math.PI * 2) * path.radius, path.center.y + Math.sin(i / count * Math.PI * 2) * path.radius));
    }
    if (path.type === 'arc') {
      let end = path.endAngle; const start = path.startAngle;
      while (end < start) end += Math.PI * 2;
      const span = end - start; const count = segments || Math.max(8, Math.ceil(span / (Math.PI / 24)));
      return Array.from({ length: count + 1 }, (_, i) => { const a = start + span * i / count; return point(path.center.x + Math.cos(a) * path.radius, path.center.y + Math.sin(a) * path.radius); });
    }
    return [];
  }

  function pointNear(a, b, tolerance = .00035) { return Math.abs(a.x - b.x) <= tolerance && Math.abs(a.y - b.y) <= tolerance; }
  function buildSolidLoops(paths) {
    const closed = [];
    const pieces = [];
    paths.forEach((path) => {
      const points = samplePath(path);
      if (points.length < 2) return;
      if (path.type === 'circle' || path.closed || pointNear(points[0], points[points.length - 1])) { closed.push(points); return; }
      pieces.push({ points, used: false });
    });
    // DXF commonly stores a boundary as individual LINE/ARC entities.  Rejoin
    // their endpoints for rendering only; simulation keeps the original CAD
    // entities untouched.
    for (const seed of pieces) {
      if (seed.used) continue;
      seed.used = true;
      const loop = [...seed.points];
      let end = loop[loop.length - 1];
      for (let guard = 0; guard < pieces.length + 2; guard += 1) {
        if (pointNear(end, loop[0]) && loop.length > 3) { closed.push(loop); break; }
        let match = null; let reverse = false;
        for (const candidate of pieces) {
          if (candidate.used) continue;
          const first = candidate.points[0]; const last = candidate.points[candidate.points.length - 1];
          if (pointNear(end, first)) { match = candidate; break; }
          if (pointNear(end, last)) { match = candidate; reverse = true; break; }
        }
        if (!match) break;
        match.used = true;
        const next = reverse ? [...match.points].reverse() : match.points;
        loop.push(...next.slice(1));
        end = loop[loop.length - 1];
      }
    }
    return closed.filter((loop) => loop.length >= 3);
  }
  function solidLoopsFor(drawing) {
    if (!drawing._solidLoops) drawing._solidLoops = buildSolidLoops(drawing.paths);
    return drawing._solidLoops;
  }
  function drawSolidLoops(ctx, drawing, transform, fillStyle) {
    const loops = solidLoopsFor(drawing);
    if (!loops.length) return;
    ctx.save(); ctx.fillStyle = fillStyle; ctx.beginPath();
    loops.forEach((loop) => {
      loop.forEach((source, index) => {
        const screen = screenPoint(transform(source));
        if (index === 0) ctx.moveTo(screen.x, screen.y); else ctx.lineTo(screen.x, screen.y);
      });
      ctx.closePath();
    });
    try { ctx.fill('evenodd'); } catch { ctx.fill(); }
    ctx.restore();
  }

  function boundsForPaths(paths) {
    const samples = paths.flatMap((path) => samplePath(path));
    if (!samples.length) return { minX: -.01, maxX: .01, minY: -.01, maxY: .01, width: .02, height: .02 };
    let minX = Infinity; let maxX = -Infinity; let minY = Infinity; let maxY = -Infinity;
    samples.forEach((p) => { minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x); minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y); });
    return { minX, maxX, minY, maxY, width: maxX - minX, height: maxY - minY };
  }

  function makeDrawing(paths, details) {
    const drawing = {
      role: details.role,
      name: details.name,
      sourceFormat: details.sourceFormat || 'DXF',
      sourceKind: details.sourceKind || 'imported',
      paths: paths.map(clonePath),
      rawText: details.rawText || null,
      rawFileName: details.rawFileName || null,
      detectedUnit: details.detectedUnit || null,
      unit: details.unit || 'm',
      unitScale: details.unitScale || 1,
      entityCount: details.entityCount || paths.length,
      unsupported: details.unsupported || [],
      origin: point(0, 0),
      pivot: point(0, 0),
      bounds: null,
    };
    drawing.bounds = boundsForPaths(drawing.paths);
    if (details.origin) drawing.origin = point(details.origin.x, details.origin.y);
    if (details.pivot) drawing.pivot = point(details.pivot.x, details.pivot.y);
    return drawing;
  }

  function fallbackWeapon() {
    const points = Array.from({ length: 10 }, (_, i) => { const r = i % 2 ? .03 : .055; const a = -Math.PI / 2 + i * Math.PI / 5; return point(Math.cos(a) * r, Math.sin(a) * r); });
    return makeDrawing([{ type: 'polyline', points, closed: true }, { type: 'circle', center: point(0, 0), radius: .008 }], { role: 'weapon', name: '备用武器轮廓', sourceKind: 'builtin', sourceFormat: '备用', pivot: point(0, 0), origin: point(0, 0) });
  }

  function fallbackFork() {
    return makeDrawing([{ type: 'polyline', closed: true, points: [point(-.07, -.006), point(.015, -.006), point(.01, .012), point(-.015, .018), point(-.07, .008)] }], { role: 'shovel', name: '备用叉子轮廓', sourceKind: 'builtin', sourceFormat: '备用', origin: point(0, 0) });
  }

  function defaultDrawing(role) {
    const asset = window.BiteDefaultModels && window.BiteDefaultModels[role === 'shovel' ? 'fork' : 'weapon'];
    if (!asset) return role === 'shovel' ? fallbackFork() : fallbackWeapon();
    const origin = asset.origin ? point(asset.origin[0] * .001, asset.origin[1] * .001) : point(0, 0);
    const pivot = asset.pivot ? point(asset.pivot[0] * .001, asset.pivot[1] * .001) : point(0, 0);
    return makeDrawing(decodeEmbeddedPaths(asset.paths), { role, name: asset.sourceName, sourceKind: 'builtin', sourceFormat: asset.sourceFormat, unit: asset.unit, unitScale: .001, entityCount: asset.paths.length, origin, pivot });
  }

  function mirrorAround(source, origin, mirrorX, mirrorY) {
    return point(origin.x + (source.x - origin.x) * (mirrorX ? -1 : 1), origin.y + (source.y - origin.y) * (mirrorY ? -1 : 1));
  }
  function weaponLocal(source) {
    const drawing = state.drawings.weapon;
    return subtract(mirrorAround(source, drawing.pivot, state.params.weaponMirrorX, state.params.weaponMirrorY), drawing.pivot);
  }
  function weaponBaseSceneOrigin() { return point(number(state.params.weaponSceneX) / 1000, number(state.params.weaponSceneY) / 1000); }
  function simulationEndTime() { return clamp(number(state.params.simulationDuration), FIXED_DT, MAX_SIM_TIME); }
  function weaponSceneOrigin() { return add(weaponBaseSceneOrigin(), point(number(state.sim?.robotTravel), 0)); }
  function weaponWorld(source) { return add(weaponSceneOrigin(), rotate(weaponLocal(source), state.sim.angle)); }
  function forkPivotOffset() { return point(number(state.params.shovelOffsetX) / 1000, number(state.params.shovelOffsetY) / 1000); }
  function forkLocal(source) {
    const drawing = state.drawings.shovel;
    const mirrored = mirrorAround(source, drawing.origin, state.params.shovelMirrorX, state.params.shovelMirrorY);
    return subtract(mirrored, drawing.origin);
  }
  function initialForkPivotAt(sceneOrigin = weaponBaseSceneOrigin()) { return add(sceneOrigin, forkPivotOffset()); }
  function forkWorldOrigin() {
    const stored = state.sim?.forkOrigin;
    return stored && Number.isFinite(stored.x) && Number.isFinite(stored.y)
      ? point(stored.x, stored.y)
      : initialForkPivotAt(weaponSceneOrigin());
  }
  function forkWorldAngle() { return number(state.sim?.forkAngle); }
  function forkWorldFromLocal(local) { return add(forkWorldOrigin(), rotate(local, forkWorldAngle())); }
  function shovelWorldAt(sceneOrigin, source, angle = 0) { return add(initialForkPivotAt(sceneOrigin), rotate(forkLocal(source), angle)); }
  function shovelWorld(source) { return forkWorldFromLocal(forkLocal(source)); }
  function mirrorParameterPoint(source, mirrorX, mirrorY) {
    return point(source.x * (mirrorX ? -1 : 1), source.y * (mirrorY ? -1 : 1));
  }
  function capsuleBoundary(start, end, radius, maxChord = CAD_COLLISION_CHORD) {
    const delta = subtract(end, start); const segmentLength = length(delta);
    if (segmentLength <= 1e-12) {
      const count = Math.max(16, Math.ceil(Math.PI * 2 * radius / Math.max(maxChord, 1e-6)));
      return Array.from({ length: count }, (_, index) => {
        const angle = index / count * Math.PI * 2;
        return add(start, point(Math.cos(angle) * radius, Math.sin(angle) * radius));
      });
    }
    const angle = Math.atan2(delta.y, delta.x);
    const arcSteps = Math.max(6, Math.ceil(Math.PI * radius / Math.max(maxChord, 1e-6)));
    const output = [];
    for (let index = 0; index <= arcSteps; index += 1) {
      const theta = angle - Math.PI / 2 + Math.PI * index / arcSteps;
      output.push(add(end, point(Math.cos(theta) * radius, Math.sin(theta) * radius)));
    }
    for (let index = 0; index <= arcSteps; index += 1) {
      const theta = angle + Math.PI / 2 + Math.PI * index / arcSteps;
      output.push(add(start, point(Math.cos(theta) * radius, Math.sin(theta) * radius)));
    }
    return output;
  }
  function parameterWeaponGeometry() {
    const toothCount = clamp(Math.round(positive(state.params.paramToothCount, 1)), 1, 32);
    const outerRadius = positive(number(state.params.paramToothLength) / 1000, .001);
    const requestedWidth = positive(number(state.params.paramToothWidth) / 1000, .0001);
    const workLength = positive(number(state.params.paramToothWorkLength) / 1000, .0001);
    // The public value is the *installed* physical rake from the supplied
    // reference: positive means the tip leads its root in the actual CCW
    // direction.  A reflection reverses oriented angles, so pre-compensate the
    // construction angle before mirroring instead of silently reversing the
    // user's requested sign (the default installation mirrors X).
    const rake = radians(number(state.params.paramToothPhaseDeg));
    const mirrorDeterminant = (state.params.weaponMirrorX ? -1 : 1) * (state.params.weaponMirrorY ? -1 : 1);
    const constructionRake = mirrorDeterminant * rake;
    const segments = Array.from({ length: toothCount }, (_, toothOrder) => {
      const angle = toothOrder * Math.PI * 2 / toothCount;
      const radial = point(Math.cos(angle), Math.sin(angle));
      const tangent = perpendicular(radial);
      const toothDirection = add(scalePoint(radial, Math.cos(constructionRake)), scalePoint(tangent, Math.sin(constructionRake)));
      // T is the actual tooth tip on the impact circle. F->T is the actual
      // working/rake face, so its length and signed angle have their literal
      // engineering meanings. A triangular back support makes the face a finite
      // solid without inventing a hub or a rounded pseudo-tip.
      const rawTip = scalePoint(radial, outerRadius);
      const rawRoot = subtract(rawTip, scalePoint(toothDirection, workLength));
      const root = mirrorParameterPoint(rawRoot, state.params.weaponMirrorX, state.params.weaponMirrorY);
      const tip = mirrorParameterPoint(rawTip, state.params.weaponMirrorX, state.params.weaponMirrorY);
      // The straight root->tip edge is the working face. Its finite backing
      // sits on the trailing side of that face. Choosing the side after
      // mirroring keeps the solid backing behind the installed CCW face.
      const finalFaceDirection = normalise(subtract(tip, root));
      const finalTangentialDirection = normalise(perpendicular(tip));
      let leadingNormal = perpendicular(finalFaceDirection);
      if (dot(leadingNormal, finalTangentialDirection) < 0) leadingNormal = scalePoint(leadingNormal, -1);
      const supportDirection = scalePoint(leadingNormal, -1);
      const back = add(root, scalePoint(supportDirection, requestedWidth));
      const loop = polygonArea([root, tip, back]) >= 0 ? [root, tip, back] : [root, back, tip];
      return {
        toothOrder,
        root,
        start: root,
        end: tip,
        tip,
        back,
        workLength,
        supportThickness: requestedWidth,
        loop,
      };
    });
    return {
      source: 'parametric', label: '参数测试齿形', toothCount, outerRadius, workLength, rake, constructionRake, edgeRadius: 0, width: requestedWidth, segments,
      materialRemovalDefined: false,
      materialValidity: '参数齿严格定义打击圆齿尖、直线迎击面、牙长与有限三角背撑；连续塑性剐蹭、沟槽历史和最早接触 TOI 尚未通过验证。选择材料模式时会在首次承载接触前回滚停止，不会把完全刚体冲量冒充材料结果；理想刚体仅是显式上限对照。',
    };
  }
  function parameterForkGeometry() {
    const tipDistance = positive(number(state.params.paramForkTipDistance) / 1000, .001);
    // The idealised parameter fork is intentionally one zero-thickness segment.
    // Its attitude is not an input: the fixed hinge and floor support determine
    // the assembled angle.  Mass properties are assigned separately below, so
    // this geometric idealisation does not imply a massless fork.
    const root = point(0, 0);
    const tip = point(tipDistance, 0);
    const points = [root, tip];
    return {
      source: 'parametric', label: '参数理想线叉', root, tip, points, loop: points,
      tipDistance, zeroThickness: true,
    };
  }
  function activeWeaponRadius() {
    return state.params.paramWeaponEnabled ? parameterWeaponGeometry().outerRadius : getMaxWeaponRadius();
  }
  function activeWeaponToothCount() {
    if (state.params.paramWeaponEnabled) return parameterWeaponGeometry().toothCount;
    const geometry = weaponCadCollisionGeometry();
    return geometry.hasClosedOutline ? geometry.teeth.length : Math.max(1, Math.round(positive(state.params.toothCount, 1)));
  }
  function activeForkBodyPoints() {
    if (state.params.paramForkEnabled) return parameterForkGeometry().points;
    return state.drawings.shovel.paths.flatMap(samplePathForCollision).map(forkBodyLocal);
  }
  function activeForkSolidLoop() {
    if (state.params.paramForkEnabled) return parameterForkGeometry().points;
    const sourceLoop = largestCollisionLoopFor(state.drawings.shovel);
    return sourceLoop ? sourceLoop.map(forkBodyLocal) : [];
  }
  function normaliseSignedAngle(angle) {
    return Math.atan2(Math.sin(angle), Math.cos(angle));
  }
  function initialForkGroundPose(localPoints = activeForkSolidLoop(), pivot = initialForkPivotAt(weaponBaseSceneOrigin())) {
    const points = (localPoints || []).filter((candidate) => Number.isFinite(candidate.x) && Number.isFinite(candidate.y));
    const minimumPointCount = state.params.paramForkEnabled ? 2 : 3;
    if (points.length < minimumPointCount) return { valid: false, angle: 0, tip: null, reason: state.params.paramForkEnabled ? '参数叉没有可装配的有限线段' : '叉子没有可装配的闭合侧视轮廓' };
    const explicitTip = state.params.paramForkEnabled ? parameterForkGeometry().tip : null;
    const maxX = Math.max(...points.map((candidate) => candidate.x));
    const minX = Math.min(...points.map((candidate) => candidate.x));
    const frontBand = clamp((maxX - minX) * .03, .0015, .004);
    const tipCandidates = explicitTip
      ? [explicitTip]
      : points.filter((candidate) => candidate.x >= maxX - frontBand)
        .sort((left, right) => right.x - left.x || left.y - right.y);
    const floor = groundY(); const solutions = [];
    tipCandidates.forEach((tip) => {
      const radius = length(tip);
      if (!(radius > GEOMETRY_CLEARANCE_EPS)) return;
      const requiredLocalY = floor - pivot.y;
      if (Math.abs(requiredLocalY) > radius + GEOMETRY_CLEARANCE_EPS) return;
      const phase = Math.atan2(tip.y, tip.x);
      const principal = Math.asin(clamp(requiredLocalY / radius, -1, 1));
      [principal - phase, Math.PI - principal - phase].forEach((rawAngle) => {
        const angle = normaliseSignedAngle(rawAngle);
        const worldY = points.map((candidate) => pivot.y + rotate(candidate, angle).y);
        const tipY = pivot.y + rotate(tip, angle).y;
        const minY = Math.min(...worldY);
        if (Math.abs(tipY - floor) <= 1e-8 && minY >= floor - GEOMETRY_CLEARANCE_EPS) {
          solutions.push({ valid: true, angle, tip: point(tip.x, tip.y), tipY, minY });
        }
      });
    });
    if (!solutions.length) {
      return {
        valid: false,
        angle: 0,
        tip: explicitTip || tipCandidates[0] || null,
        reason: '固定铰点与当前叉子轮廓无法在不穿过地面的前提下让前端叉尖接触 Y = 0 mm；请调整铰点 Y、叉尖距离或叉形',
      };
    }
    // The physically assembled pose is the valid support solution requiring
    // the least rotation from the drawing's local datum.  This avoids choosing
    // the upside-down sine branch while keeping the hinge position invariant.
    solutions.sort((left, right) => Math.abs(left.angle) - Math.abs(right.angle));
    return solutions[0];
  }
  function parameterGeometryStatus() {
    if (state.params.paramWeaponEnabled) {
      const rawCount = Number(state.params.paramToothCount);
      const rawRadius = Number(state.params.paramToothLength) / 1000;
      const rawWidth = Number(state.params.paramToothWidth) / 1000;
      const rawWorkLength = Number(state.params.paramToothWorkLength) / 1000;
      const rawPhase = Number(state.params.paramToothPhaseDeg);
      if (!Number.isInteger(rawCount) || rawCount < 1 || rawCount > 32) return { valid: false, reason: '测试齿数必须是 1–32 的整数；求解器不会静默取整' };
      if (!Number.isFinite(rawRadius) || rawRadius <= 0 || !Number.isFinite(rawWidth) || rawWidth <= 0 || !Number.isFinite(rawWorkLength) || rawWorkLength <= 0) return { valid: false, reason: '测试齿的打击圆半径、牙长与侧视实体宽度必须是大于 0 mm 的有限数值' };
      // |rake| >= 90° makes the entered working face cease to point inward.
      // Smaller angles are accepted by geometry; 0–15° is only a design
      // guideline from the supplied document, not a fitted physics limit.
      if (!Number.isFinite(rawPhase) || rawPhase <= -90 || rawPhase >= 90) return { valid: false, reason: '齿相位角（前角 / rake）的绝对值必须小于 90°；它不是第 0 齿圆周方位' };
      const geometry = parameterWeaponGeometry();
      if (geometry.segments.some((segment) => segment.loop.some((candidate) => length(candidate) > geometry.outerRadius + GEOMETRY_CLEARANCE_EPS))) return { valid: false, reason: '当前牙长、前角或背撑厚度让齿根越出打击圆；请减小牙长/厚度或调整前角' };
      for (let left = 0; left < geometry.segments.length; left += 1) {
        for (let right = left + 1; right < geometry.segments.length; right += 1) {
          const a = geometry.segments[left]; const b = geometry.segments[right];
          if (closedLoopsOverlap(a.loop, b.loop)) {
            return { valid: false, reason: `第 ${left + 1} 与第 ${right + 1} 个测试齿实体重叠；请减少齿数/牙长/背撑厚度或增大打击圆` };
          }
        }
      }
    }
    if (state.params.paramForkEnabled) {
      const tipDistance = Number(state.params.paramForkTipDistance) / 1000;
      if (!Number.isFinite(tipDistance) || tipDistance <= 0) return { valid: false, reason: '叉子铰点至叉尖距离 D 必须是大于 0 mm 的有限数值' };
      const geometry = parameterForkGeometry();
      if (geometry.points.length !== 2 || !geometry.points.every((candidate) => Number.isFinite(candidate.x) && Number.isFinite(candidate.y))) return { valid: false, reason: '参数叉无法形成有限的铰点至叉尖线段' };
    }
    return { valid: true, reason: null };
  }
  function getMaxWeaponRadius() {
    const drawing = state.drawings.weapon;
    if (!drawing?.paths?.length) return .001;
    // Use the same fine CAD sampling as collision. The display sampler can miss
    // an arc apex between coarse points, which is not acceptable when this
    // value sets the fixed-floor clearance and the 1.05x default height.
    const cacheKey = `${drawing.paths.length}:${drawing.pivot.x.toFixed(7)}:${drawing.pivot.y.toFixed(7)}`;
    if (drawing._maxWeaponRadius?.cacheKey === cacheKey) return drawing._maxWeaponRadius.value;
    const value = drawing.paths.flatMap((path) => samplePathForCollision(path))
      .reduce((max, p) => Math.max(max, length(weaponLocal(p))), .001);
    drawing._maxWeaponRadius = { cacheKey, value };
    return value;
  }
  function hasClosedWeaponCadOutline() { return Boolean(state.drawings.weapon && weaponCadCollisionGeometry().hasClosedOutline); }
  function derivedTipRadiusMillimetres() { return Math.max(1, Math.round(getMaxWeaponRadius() * 1000)); }
  function synchroniseDerivedWeaponRadius() {
    if (!state.params.paramWeaponEnabled && hasClosedWeaponCadOutline()) state.params.tipRadius = derivedTipRadiusMillimetres();
  }
  function samplePathForCollision(path, maxChord = CAD_COLLISION_CHORD) {
    if (path.type === 'line') return [path.start, path.end];
    if (path.type === 'polyline') {
      const source = path.points || []; const output = [];
      const appendSegment = (start, end, first) => {
        const count = Math.max(1, Math.ceil(length(subtract(end, start)) / maxChord));
        for (let i = first ? 0 : 1; i <= count; i += 1) output.push(lerp(start, end, i / count));
      };
      for (let i = 1; i < source.length; i += 1) appendSegment(source[i - 1], source[i], i === 1);
      if (path.closed && source.length > 2) appendSegment(source[source.length - 1], source[0], !output.length);
      return output;
    }
    if (path.type === 'circle') {
      const count = Math.max(16, Math.ceil(Math.PI * 2 * path.radius / maxChord));
      return Array.from({ length: count }, (_, i) => point(path.center.x + Math.cos(i / count * Math.PI * 2) * path.radius, path.center.y + Math.sin(i / count * Math.PI * 2) * path.radius));
    }
    if (path.type === 'arc') {
      let end = path.endAngle; const start = path.startAngle;
      while (end < start) end += Math.PI * 2;
      const span = end - start;
      // Keep small fillets recognisably curved too: a 0.5 mm tooth-tip arc
      // still needs angular samples even though its chord is short.
      const count = Math.max(2, Math.ceil(path.radius * span / maxChord), Math.ceil(span / (Math.PI / 18)));
      return Array.from({ length: count + 1 }, (_, i) => {
        const angle = start + span * i / count;
        return point(path.center.x + Math.cos(angle) * path.radius, path.center.y + Math.sin(angle) * path.radius);
      });
    }
    return [];
  }
  function buildCollisionLoops(paths) {
    const closed = []; const pieces = [];
    paths.forEach((path) => {
      const points = samplePathForCollision(path);
      if (points.length < 2) return;
      if (path.type === 'circle' || path.closed || pointNear(points[0], points[points.length - 1])) { closed.push(points); return; }
      pieces.push({ points, used: false });
    });
    for (const seed of pieces) {
      if (seed.used) continue;
      seed.used = true;
      const loop = [...seed.points]; let end = loop[loop.length - 1];
      for (let guard = 0; guard < pieces.length + 2; guard += 1) {
        if (pointNear(end, loop[0]) && loop.length > 3) { closed.push(loop); break; }
        let match = null; let reverse = false;
        for (const candidate of pieces) {
          if (candidate.used) continue;
          const first = candidate.points[0]; const last = candidate.points[candidate.points.length - 1];
          if (pointNear(end, first)) { match = candidate; break; }
          if (pointNear(end, last)) { match = candidate; reverse = true; break; }
        }
        if (!match) break;
        match.used = true;
        const next = reverse ? [...match.points].reverse() : match.points;
        loop.push(...next.slice(1)); end = loop[loop.length - 1];
      }
    }
    return closed.map((loop) => pointNear(loop[0], loop[loop.length - 1]) ? loop.slice(0, -1) : loop).filter((loop) => loop.length >= 3);
  }
  function polygonArea(points) {
    return points.reduce((area, current, index) => {
      const next = points[(index + 1) % points.length];
      return area + current.x * next.y - current.y * next.x;
    }, 0) / 2;
  }
  function circularIndex(index, lengthValue) { return (index % lengthValue + lengthValue) % lengthValue; }
  function weaponCadCollisionGeometry() {
    const drawing = state.drawings.weapon;
    const cacheKey = `${drawing.paths.length}:${drawing.pivot.x.toFixed(7)}:${drawing.pivot.y.toFixed(7)}`;
    if (drawing._cadCollisionGeometry?.cacheKey === cacheKey) return drawing._cadCollisionGeometry;
    const loops = buildCollisionLoops(drawing.paths);
    const outline = loops.reduce((largest, loop) => !largest || Math.abs(polygonArea(loop)) > Math.abs(polygonArea(largest)) ? loop : largest, null);
    // If a drawing has no closed external outline, stay conservative: use its
    // furthest real entity point as one contact feature instead of inventing a
    // circular tooth or treating holes/construction lines as a solid boundary.
    const fallback = drawing.paths.flatMap((path) => samplePathForCollision(path));
    const rawOutline = outline?.length ? outline : fallback;
    const local = rawOutline.map((source) => subtract(source, drawing.pivot));
    const radii = local.map(length);
    const maxRadius = Math.max(...radii, .001);
    const sortedRadii = [...radii].sort((a, b) => a - b);
    const medianRadius = sortedRadii[Math.floor(sortedRadii.length / 2)] || maxRadius;
    const radialProminence = maxRadius - medianRadius;
    const hasResolvedTooth = radialProminence >= Math.max(.0005, maxRadius * .015);
    const peaks = [];
    if (local.length && (hasResolvedTooth || !outline?.length)) {
      const neighborhood = clamp(Math.round(local.length * .016), 2, 12);
      // A round disk rim can contain hundreds of equal local maxima.  A tooth
      // must be near the global CAD protrusion, not merely a high point on the
      // underlying disk.  This preserves unequal real teeth while rejecting
      // the 45 mm rim in the supplied 55 mm single-tooth drawing.
      const permittedDrop = clamp((maxRadius - medianRadius) * .22, .0008, maxRadius * .08);
      const minimumPeak = maxRadius - permittedDrop;
      for (let index = 0; index < local.length; index += 1) {
        if (radii[index] < minimumPeak) continue;
        let isPeak = true;
        for (let offset = 1; offset <= neighborhood; offset += 1) {
          if (radii[circularIndex(index - offset, local.length)] > radii[index] + 1e-7 || radii[circularIndex(index + offset, local.length)] > radii[index] + 1e-7) { isPeak = false; break; }
        }
        if (isPeak) peaks.push(index);
      }
    }
    // Collapse a rounded apex into one CAD tooth and retain separate peaks for
    // genuinely separate teeth.  The input is never copied around the axis.
    const teeth = [];
    const minSeparation = Math.max(3, Math.round(local.length * .045));
    peaks.sort((a, b) => radii[b] - radii[a]).forEach((index) => {
      if (teeth.some((feature) => Math.min(Math.abs(feature.index - index), local.length - Math.abs(feature.index - index)) < minSeparation)) return;
      teeth.push({ index, local: local[index], radius: radii[index] });
    });
    if (!teeth.length && local.length && !outline?.length) {
      const index = radii.indexOf(maxRadius);
      teeth.push({ index, local: local[index], radius: radii[index] });
    }
   const collisionIndexes = new Set();
   if (outline?.length) {
     teeth.forEach((feature) => {
        // Determine the physical tooth boundary from its two adjacent radial
        // valleys. This is CAD topology, not a fixed 3–9 mm sampling band:
        // the whole protruding lobe can strike, while a round backing rim is
        // only a blocking surface.
        const findValley = (direction) => {
          let previousIndex = feature.index; let valleyIndex = feature.index;
          let descending = false;
          const minimumRise = Math.max(.00025, feature.radius * .003);
          for (let step = 1; step < local.length / 2; step += 1) {
            const index = circularIndex(feature.index + direction * step, local.length);
            const currentRadius = radii[index]; const previousRadius = radii[previousIndex];
            if (currentRadius <= previousRadius + 1e-7) {
              descending = true;
              if (currentRadius < radii[valleyIndex]) valleyIndex = index;
            } else if (descending && currentRadius > radii[valleyIndex] + minimumRise) break;
            previousIndex = index;
          }
          return valleyIndex;
        };
        const start = findValley(-1); const end = findValley(1);
        feature.startIndex = start; feature.endIndex = end;
        feature.rootRadius = Math.max(radii[start], radii[end]);
        let index = start;
        for (let guard = 0; guard < local.length; guard += 1) {
          collisionIndexes.add(index);
          if (index === end) break;
          index = circularIndex(index + 1, local.length);
        }
     });
   } else if (local.length) collisionIndexes.add(radii.indexOf(maxRadius));
    const geometry = { cacheKey, hasClosedOutline: Boolean(outline?.length), sourceOutline: rawOutline, teeth, collisionIndexes: [...collisionIndexes].sort((a, b) => a - b), radialProminence };
    drawing._cadCollisionGeometry = geometry;
    return geometry;
  }
  function cadToothCuttingLoops() {
    const geometry = weaponCadCollisionGeometry();
    const outline = geometry.sourceOutline || []; const count = outline.length;
    if (!geometry.hasClosedOutline || count < 3) return [];
    return geometry.teeth.map((feature) => {
      const points = []; let index = circularIndex(feature.startIndex, count);
      const end = circularIndex(feature.endIndex, count);
      for (let guard = 0; guard < count; guard += 1) {
        points.push(weaponLocal(outline[index]));
        if (index === end) break;
        index = circularIndex(index + 1, count);
      }
      // The real CAD lobe boundary is closed by the chord between its two
      // adjacent radial valleys.  This deliberately excludes the backing disk:
      // only the identified tooth may remove material, while the remaining
      // weapon outline stays a non-penetrating rigid boundary.
      const cleaned = removeDuplicateLoopPoints(points).points;
      return cleaned.length >= 3 && Math.abs(polygonArea(cleaned)) > 1e-12 ? cleaned : null;
    });
  }
  function cadCollisionSamples() {
    const geometry = weaponCadCollisionGeometry();
    return geometry.collisionIndexes.map((index) => ({ index: `cad:${index}`, source: geometry.sourceOutline[index], local: weaponLocal(geometry.sourceOutline[index]) })).filter((sample) => sample.source && Number.isFinite(sample.local.x) && Number.isFinite(sample.local.y));
  }
  function cadBlockingSamples() {
    if (state.params.paramWeaponEnabled) {
      const geometry = parameterWeaponGeometry(); const samples = [];
      geometry.segments.forEach((segment) => {
        segment.loop.forEach((local, index) => samples.push({
          index: `param-block:${segment.toothOrder}:${index}`,
          source: local,
          local,
          toothOrder: segment.toothOrder,
          isImpactSample: true,
          parametric: true,
        }));
      });
      return samples;
    }
    const geometry = weaponCadCollisionGeometry(); const drawing = state.drawings.weapon;
    const cacheKey = `${geometry.cacheKey}:${state.params.weaponMirrorX}:${state.params.weaponMirrorY}:${targetThickness().toFixed(6)}`;
    if (drawing._cadBlockingSamples?.cacheKey === cacheKey) return drawing._cadBlockingSamples.samples;
    const retained = new Set(geometry.collisionIndexes); const samples = [];
    // The DXF boundary was initially tessellated at CAD_COLLISION_CHORD. For
    // a thinner target, re-sample each edge below half its thickness so a
    // target cannot slip through gaps between blocking points.
    const maxChord = Math.min(CAD_COLLISION_CHORD, Math.max(.00015, targetThickness() * .45));
    const outline = geometry.sourceOutline;
    outline.forEach((source, index) => {
      const next = outline[(index + 1) % outline.length];
      const start = weaponLocal(source); const end = weaponLocal(next);
      if (!Number.isFinite(start.x) || !Number.isFinite(start.y) || !Number.isFinite(end.x) || !Number.isFinite(end.y)) return;
      const count = Math.max(1, Math.ceil(length(subtract(end, start)) / maxChord));
      for (let part = 0; part < count; part += 1) {
        const fraction = part / count;
        // A sub-sample inherits the containing real CAD boundary segment. It
        // never creates a virtual tooth; it only preserves the actual lobe.
        samples.push({ index: `cad-block:${index}:${part}`, source, local: lerp(start, end, fraction), isImpactSample: retained.has(index) });
      }
    });
    drawing._cadBlockingSamples = { cacheKey, samples };
    return samples;
  }
  function getShovelBoundsAt(sceneOrigin, angle = 0) {
    const samples = state.params.paramForkEnabled
      ? parameterForkGeometry().points.map((source) => add(initialForkPivotAt(sceneOrigin), rotate(source, angle)))
      : state.drawings.shovel.paths.flatMap(samplePathForCollision).map((source) => shovelWorldAt(sceneOrigin, source, angle));
    return samples.reduce((b, p) => ({ minX: Math.min(b.minX, p.x), maxX: Math.max(b.maxX, p.x), minY: Math.min(b.minY, p.y), maxY: Math.max(b.maxY, p.y) }), { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity });
  }
  function getShovelBoundsWorld() {
    const samples = activeForkBodyPoints().map(forkWorldFromLocal);
    return samples.reduce((b, p) => ({ minX: Math.min(b.minX, p.x), maxX: Math.max(b.maxX, p.x), minY: Math.min(b.minY, p.y), maxY: Math.max(b.maxY, p.y) }), { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity });
  }
  function getShovelLeadingEdgeAt(sceneOrigin) {
    // Use the same dense entity sampling as CAD weapon collision.  A long DXF
    // polyline or arc must not turn into a single fictitious fork point.
    const samples = state.params.paramForkEnabled
      ? parameterForkGeometry().points.map((source) => add(initialForkPivotAt(sceneOrigin), source))
      : state.drawings.shovel.paths.flatMap(samplePathForCollision).map((source) => shovelWorldAt(sceneOrigin, source, 0));
    if (!samples.length) return { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity, width: 0, height: 0, points: [] };
    const bounds = samples.reduce((b, p) => ({ minX: Math.min(b.minX, p.x), maxX: Math.max(b.maxX, p.x), minY: Math.min(b.minY, p.y), maxY: Math.max(b.maxY, p.y) }), { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity });
    bounds.width = bounds.maxX - bounds.minX; bounds.height = bounds.maxY - bounds.minY;
    // The foremost 1.5–4 mm strip, rather than the whole CAD bounding box.
    // It keeps an elevated rear hoop from becoming a fake fork contact point.
    const band = clamp(bounds.width * .03, .0015, .004);
    const points = samples.filter((p) => p.x >= bounds.maxX - band);
    return { ...bounds, points: points.length ? points : samples };
  }
  function groundClearanceStatus() {
    const base = weaponBaseSceneOrigin();
    const weaponRadius = state.params.paramWeaponEnabled
      ? activeWeaponRadius()
      : Math.max(positive(number(state.params.tipRadius) / 1000, .001), getMaxWeaponRadius());
    const weaponBottom = base.y - weaponRadius;
    const forkPivot = initialForkPivotAt(base);
    const forkPoints = activeForkSolidLoop();
    const forkPose = initialForkGroundPose(forkPoints, forkPivot);
    const shovelBottom = forkPose.valid
      ? Math.min(...forkPoints.map((candidate) => forkPivot.y + rotate(candidate, forkPose.angle).y))
      : getShovelBoundsAt(base).minY;
    const floor = groundY();
    return {
      weaponBottom,
      shovelBottom,
      floor,
      forkPose,
      weaponClearance: weaponBottom - floor,
      shovelClearance: shovelBottom - floor,
      valid: forkPose.valid && weaponBottom >= floor - GEOMETRY_CLEARANCE_EPS && shovelBottom >= floor - GEOMETRY_CLEARANCE_EPS,
    };
  }
  function getShovelFrontWorld() { return getShovelBoundsWorld().maxX; }
  function getMaterial() { return MATERIALS[state.params.targetMaterial] || MATERIALS.custom; }
  function getWeaponMaterial() { return WEAPON_MATERIALS[state.params.weaponMaterial] || WEAPON_MATERIALS.custom; }
  function getShovelMaterial() { return WEAPON_MATERIALS[state.params.shovelMaterial] || WEAPON_MATERIALS.custom; }
  function getFloorMaterial() { return FLOOR_MATERIALS[state.params.floorMaterial] || FLOOR_MATERIALS.custom; }
  function targetMaterialProperties() {
    const preset = getMaterial();
    return {
      ...preset,
      density: positive(state.params.targetDensity, preset.density),
      youngModulus: positive(state.params.targetYoungModulus, preset.youngModulus),
      yieldStrength: positive(state.params.targetYieldStrength, preset.yieldStrength),
      shearStrength: positive(state.params.targetShearStrength, preset.shearStrength),
      fractureEnergy: Math.max(0, number(state.params.targetFractureEnergy ?? preset.fractureEnergy)),
      fractureSource: String(state.params.targetFractureSource || preset.fractureSource || '').trim(),
      minChipThickness: Math.max(0, number(state.params.targetMinChipThickness)) / 1000,
      minChipSource: String(state.params.targetMinChipSource || '').trim(),
    };
  }
  function applyTargetMaterialPreset(key = state.params.targetMaterial) {
    const material = MATERIALS[key] || MATERIALS.custom;
    state.params.targetDensity = material.density;
    state.params.targetYoungModulus = material.youngModulus;
    state.params.targetYieldStrength = material.yieldStrength;
    state.params.targetShearStrength = material.shearStrength;
    state.params.targetFractureEnergy = Math.max(0, number(material.fractureEnergy));
    state.params.targetFractureSource = material.fractureSource || '';
    state.params.targetMinChipThickness = 0;
    state.params.targetMinChipSource = '';
  }
  function zOverlap(widthA, centreA, widthB, centreB = 0) {
    const a0 = centreA - widthA / 2; const a1 = centreA + widthA / 2;
    const b0 = centreB - widthB / 2; const b1 = centreB + widthB / 2;
    return Math.max(0, Math.min(a1, b1) - Math.max(a0, b0));
  }
  function effectiveToolWidthZ(role = 'weapon') {
    const targetWidth = positive(number(state.params.targetWidthZ) / 1000, .001);
    const toolWidth = positive(number(state.params[role === 'fork' ? 'shovelWidthZ' : 'weaponWidthZ']) / 1000, .001);
    const toolOffset = number(state.params[role === 'fork' ? 'shovelZOffset' : 'weaponZOffset']) / 1000;
    return zOverlap(toolWidth, toolOffset, targetWidth, 0);
  }
  function frictionScale() { return clamp(number(state.params.frictionFactor, 1), .25, 2); }
  function weaponTargetFriction() {
    const pair = WEAPON_TARGET_MU[state.params.weaponMaterial] || WEAPON_TARGET_MU.custom;
    return clamp((pair[state.params.targetMaterial] ?? pair.custom ?? .32) * frictionScale(), .02, 1.2);
  }
  function shovelTargetFriction() {
    const pair = WEAPON_TARGET_MU[state.params.shovelMaterial] || WEAPON_TARGET_MU.custom;
    return clamp((pair[state.params.targetMaterial] ?? pair.custom ?? .32) * frictionScale(), .02, 1.2);
  }
  function targetFloorFriction() {
    const targetPairs = TARGET_FLOOR_CONTACT[state.params.targetMaterial] || TARGET_FLOOR_CONTACT.custom;
    const pair = targetPairs[state.params.floorMaterial] || targetPairs.osb || { static: .45, kinetic: .34 };
    return { static: clamp(pair.static * frictionScale(), .02, 1.5), kinetic: clamp(pair.kinetic * frictionScale(), .02, 1.5) };
  }

  // Rapier owns the free target, its floor contact, friction and continuous
  // collision detection. The fast, potentially concave DXF weapon remains in
  // the CAD sweep solver below; feeding it into a generic convex collider
  // would lose the actual tooth shape supplied by the drawing.
  async function initialiseRapier() {
    let RAPIER = null;
    try {
      RAPIER = window.BiteRapierReady ? await window.BiteRapierReady : window.BiteRapier;
    } catch (error) {
      state.rapierError = error?.message || 'Rapier 2D 初始化失败';
      state.rapierBackend = window.BiteRapierLoadState?.backend || 'unavailable';
      state.rapierFallbackReason = window.BiteRapierLoadState?.fallbackReason || null;
      return false;
    }
    if (!RAPIER) {
      state.rapierError = 'Rapier 2D 未加载';
      state.rapierBackend = window.BiteRapierLoadState?.backend || 'unavailable';
      state.rapierFallbackReason = window.BiteRapierLoadState?.fallbackReason || null;
      return false;
    }
    try {
      await RAPIER.init();
      state.rapier = RAPIER;
      state.rapierError = null;
      state.rapierBackend = window.BiteRapierLoadState?.backend || 'compat';
      state.rapierFallbackReason = window.BiteRapierLoadState?.fallbackReason || null;
      return true;
    } catch (error) {
      state.rapier = null;
      state.rapierError = error?.message || 'Rapier 2D 初始化失败';
      state.rapierBackend = window.BiteRapierLoadState?.backend || 'unavailable';
      state.rapierFallbackReason = window.BiteRapierLoadState?.fallbackReason || null;
      return false;
    }
  }

 function disposeTargetPhysics(physics) {
   if (!physics) return;
   try { physics.world?.free?.(); } catch (_) { /* best effort cleanup */ }
 }

  // A predicted CCD segment can advance Rapier before the earliest DXF time
  // of impact is known. Save the complete world, not just a body transform,
  // so contact manifolds and CCD state roll back with the physical timeline.
  function snapshotTargetPhysics(physics = state.sim?.physics) {
    if (!physics?.world?.takeSnapshot) return null;
    return {
      bytes: physics.world.takeSnapshot(),
      floorBodyHandle: physics.floorBody?.handle,
      floorColliderHandle: physics.floorCollider?.handle,
      targetBodyHandle: physics.targetBody?.handle,
      targetColliderHandle: physics.targetCollider?.handle,
    };
  }
  function restoreTargetPhysics(snapshot) {
    const RAPIER = state.rapier; const sim = state.sim;
    if (!snapshot?.bytes || !RAPIER?.World?.restoreSnapshot || !sim?.physics) return false;
    try {
      const oldWorld = sim.physics.world;
      const world = RAPIER.World.restoreSnapshot(snapshot.bytes);
      const physics = {
        world,
        floorBody: Number.isInteger(snapshot.floorBodyHandle) ? world.getRigidBody(snapshot.floorBodyHandle) : null,
        floorCollider: Number.isInteger(snapshot.floorColliderHandle) ? world.getCollider(snapshot.floorColliderHandle) : null,
        targetBody: Number.isInteger(snapshot.targetBodyHandle) ? world.getRigidBody(snapshot.targetBodyHandle) : null,
        targetCollider: Number.isInteger(snapshot.targetColliderHandle) ? world.getCollider(snapshot.targetColliderHandle) : null,
      };
      if (!physics.targetBody || !physics.targetCollider) { world.free?.(); return false; }
      sim.physics = physics;
      try { oldWorld?.free?.(); } catch (_) { /* old snapshot world is disposable */ }
      return true;
    } catch (error) {
      state.rapierError = error?.message || 'Rapier CCD 回滚失败';
      return false;
    }
  }

  function createTargetPhysics(target) {
    const RAPIER = state.rapier;
    if (!RAPIER) return null;
    try {
      const world = new RAPIER.World({ x: 0, y: -GRAVITY });
      world.integrationParameters.dt = FIXED_DT;
      const floorBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, targetSupportY()));
      const floorDesc = RAPIER.ColliderDesc.halfspace({ x: 0, y: 1 })
        .setFriction(targetFloorFriction().kinetic)
        .setRestitution(getFloorMaterial().restitution);
      const floorCollider = world.createCollider(floorDesc, floorBody);
      const targetDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(target.pos.x, target.pos.y)
        .setRotation(target.angle)
        .setLinvel(target.vel.x, target.vel.y)
        .setAngvel(target.omega)
        .setCcdEnabled(true);
      if (typeof targetDesc.setAdditionalSolverIterations === 'function') targetDesc.setAdditionalSolverIterations(8);
      const targetBody = world.createRigidBody(targetDesc);
      targetBody.enableCcd?.(true);
      targetBody.setSoftCcdPrediction?.(.002);
      const targetDescShape = RAPIER.ColliderDesc.cuboid(targetLength() / 2, targetThickness() / 2)
        .setMass(effectiveTargetMass())
        .setFriction(targetFloorFriction().kinetic)
        .setRestitution(getMaterial().restitution);
      const targetCollider = world.createCollider(targetDescShape, targetBody);
      return { world, floorBody, floorCollider, targetBody, targetCollider };
    } catch (error) {
      state.rapierError = error?.message || 'Rapier 刚体创建失败';
      return null;
    }
  }

  function rapierInteractionGroups(membership, filter) {
    return ((((membership & 0xffff) << 16) | (filter & 0xffff)) >>> 0);
  }

  function configureRigCollider(desc, { friction = 0, restitution = 0, groups, solverGroups, density = 0, contactSkin = 0 } = {}) {
    const RAPIER = state.rapier;
    desc.setDensity(density).setFriction(friction).setRestitution(restitution);
    if (Number.isFinite(groups)) desc.setCollisionGroups(groups);
    if (Number.isFinite(solverGroups) && typeof desc.setSolverGroups === 'function') desc.setSolverGroups(solverGroups);
    if (typeof desc.setContactSkin === 'function') desc.setContactSkin(Math.max(0, contactSkin));
    const combine = RAPIER?.CoefficientCombineRule;
    if (combine && typeof desc.setFrictionCombineRule === 'function') desc.setFrictionCombineRule(combine.Multiply);
    if (combine && typeof desc.setRestitutionCombineRule === 'function') desc.setRestitutionCombineRule(combine.Max);
    return desc;
  }

  function removeDuplicateLoopPoints(points) {
    const output = []; const sourceIndexes = [];
    points.forEach((candidate, sourceIndex) => {
      if (!output.length || length(subtract(candidate, output[output.length - 1])) > 1e-7) {
        output.push(candidate); sourceIndexes.push(sourceIndex);
      }
    });
    if (output.length > 2 && length(subtract(output[0], output[output.length - 1])) <= 1e-7) {
      output.pop(); sourceIndexes.pop();
    }
    return { points: output, sourceIndexes };
  }

  function largestCollisionLoopFor(drawing) {
    const loops = buildCollisionLoops(drawing.paths);
    return loops.reduce((largest, loop) => !largest || Math.abs(polygonArea(loop)) > Math.abs(polygonArea(largest)) ? loop : largest, null);
  }

  function forkBodyLocal(source) {
    return forkLocal(source);
  }

  function polygonMassMomentsAtOrigin(sourcePoints) {
    const points = removeDuplicateLoopPoints(sourcePoints || []).points;
    if (points.length < 3) return null;
    let twiceArea = 0; let firstX = 0; let firstY = 0; let polarNumerator = 0;
    for (let index = 0; index < points.length; index += 1) {
      const a = points[index]; const b = points[(index + 1) % points.length]; const signedCross = cross(a, b);
      twiceArea += signedCross;
      firstX += (a.x + b.x) * signedCross;
      firstY += (a.y + b.y) * signedCross;
      polarNumerator += signedCross * (dot(a, a) + dot(a, b) + dot(b, b));
    }
    const signedArea = twiceArea / 2; const area = Math.abs(signedArea);
    if (!(area > 1e-14)) return null;
    const centroid = point(firstX / (6 * signedArea), firstY / (6 * signedArea));
    return {
      points,
      signedArea,
      area,
      centroid,
      firstMoment: scalePoint(centroid, area),
      polarAtOrigin: Math.abs(polarNumerator / 12),
    };
  }

  function loopInteriorProbe(loop, moments = polygonMassMomentsAtOrigin(loop)) {
    if (!moments) return null;
    const bounds = loop.reduce((result, candidate) => ({
      minX: Math.min(result.minX, candidate.x), maxX: Math.max(result.maxX, candidate.x),
      minY: Math.min(result.minY, candidate.y), maxY: Math.max(result.maxY, candidate.y),
    }), { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity });
    const span = Math.max(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY, 1e-6);
    const baseOffset = clamp(span * 1e-6, 1e-9, 1e-6);
    const orientation = Math.sign(moments.signedArea) || 1;
    for (let index = 0; index < loop.length; index += 1) {
      const a = loop[index]; const b = loop[(index + 1) % loop.length]; const edge = subtract(b, a); const edgeLength = length(edge);
      if (!(edgeLength > 1e-12)) continue;
      const inward = scalePoint(perpendicular(scalePoint(edge, 1 / edgeLength)), orientation);
      for (const multiplier of [1, 4, 16]) {
        const candidate = add(lerp(a, b, .5), scalePoint(inward, baseOffset * multiplier));
        if (pointInsideSimpleLoop(candidate, loop)) return candidate;
      }
    }
    return pointInsideSimpleLoop(moments.centroid, loop) ? moments.centroid : loop[0];
  }

  function uniformBoundaryMassShape(drawing) {
    let totalLength = 0; let polarLineIntegral = 0;
    (drawing?.paths || []).forEach((path) => {
      const points = samplePathForCollision(path).map(weaponLocal);
      if (points.length < 2) return;
      const segmentCount = points.length - 1 + ((path.type === 'circle' || path.closed) ? 1 : 0);
      for (let index = 0; index < segmentCount; index += 1) {
        const a = points[index % points.length]; const b = points[(index + 1) % points.length]; const span = length(subtract(b, a));
        if (!(span > 1e-12)) continue;
        totalLength += span;
        polarLineIntegral += span * (dot(a, a) + dot(a, b) + dot(b, b)) / 3;
      }
    });
    if (!(totalLength > 1e-12) || !(polarLineIntegral > 0)) return null;
    return { area: 0, centroid: point(0, 0), radiusOfGyrationSquared: polarLineIntegral / totalLength, source: 'cad-uniform-boundary' };
  }

  function activeWeaponMassShape() {
    if (state.params.paramWeaponEnabled) {
      const geometry = parameterWeaponGeometry(); let area = 0; let polarAtOrigin = 0; let firstMoment = point(0, 0);
      const clipping = globalThis.polygonClipping;
      if (clipping?.union) {
        const union = clipping.union(...geometry.segments.map((segment) => clippingPolygonFromLoop(segment.loop)));
        const moments = materialGeometryMoments(union);
        area = moments.area; polarAtOrigin = moments.polarAtOrigin; firstMoment = point(moments.firstX, moments.firstY);
      } else {
        // Normal validated parameter teeth do not overlap. This fallback keeps
        // startup diagnosable if the geometry library is missing, while the
        // normal path above also handles legacy/QA payloads with overlapping teeth.
        geometry.segments.forEach((segment) => {
          const moments = polygonMassMomentsAtOrigin(segment.loop);
          if (!moments) return;
          area += moments.area; polarAtOrigin += moments.polarAtOrigin; firstMoment = add(firstMoment, moments.firstMoment);
        });
      }
      if (area > 1e-14 && polarAtOrigin > 0) {
        return { area, centroid: scalePoint(firstMoment, 1 / area), radiusOfGyrationSquared: polarAtOrigin / area, source: 'parametric-uniform-lamina' };
      }
    } else {
      const drawing = state.drawings.weapon;
      if (drawing?.paths?.length) {
        const geometry = weaponCadCollisionGeometry();
        const cacheKey = `${geometry.cacheKey}:${Boolean(state.params.weaponMirrorX)}:${Boolean(state.params.weaponMirrorY)}`;
        if (drawing._weaponMassShape?.cacheKey === cacheKey) return drawing._weaponMassShape.value;
        const loops = buildCollisionLoops(drawing.paths).map((loop) => removeDuplicateLoopPoints(loop.map(weaponLocal)).points).filter((loop) => loop.length >= 3);
        if (loops.length) {
          const moments = loops.map(polygonMassMomentsAtOrigin);
          const outerIndex = moments.reduce((largest, candidate, index) => candidate && (!moments[largest] || candidate.area > moments[largest].area) ? index : largest, 0);
          const outer = loops[outerIndex]; const probes = loops.map((loop, index) => loopInteriorProbe(loop, moments[index]));
          const included = loops.map((loop, index) => index === outerIndex || Boolean(probes[index] && pointInsideSimpleLoop(probes[index], outer)));
          let area = 0; let polarAtOrigin = 0; let firstMoment = point(0, 0);
          loops.forEach((loop, index) => {
            if (!included[index] || !moments[index] || !probes[index]) return;
            const depth = loops.reduce((count, container, containerIndex) => count + (containerIndex !== index && included[containerIndex] && pointInsideSimpleLoop(probes[index], container) ? 1 : 0), 0);
            const sign = depth % 2 ? -1 : 1;
            area += sign * moments[index].area;
            polarAtOrigin += sign * moments[index].polarAtOrigin;
            firstMoment = add(firstMoment, scalePoint(moments[index].firstMoment, sign));
          });
          if (area > 1e-14 && polarAtOrigin > 0) {
            const value = { area, centroid: scalePoint(firstMoment, 1 / area), radiusOfGyrationSquared: polarAtOrigin / area, source: 'cad-even-odd-uniform-lamina' };
            drawing._weaponMassShape = { cacheKey, value };
            return value;
          }
        }
        const boundary = uniformBoundaryMassShape(drawing);
        if (boundary) return boundary;
      }
    }
    const radius = Math.max(activeWeaponRadius(), .001);
    return { area: 0, centroid: point(0, 0), radiusOfGyrationSquared: radius ** 2 / 2, source: 'radius-disk-fallback' };
  }

  function automaticWeaponMassProperties() {
    const shape = activeWeaponMassShape(); const mass = Number(state.params.weaponMass);
    const validMass = Number.isFinite(mass) && mass > 0;
    const inertia = validMass ? mass * shape.radiusOfGyrationSquared : 0;
    const inertiaInput = inertia * 1e6;
    state.params.weaponInertia = Number(inertiaInput.toPrecision(12));
    return { ...shape, mass, inertia, inertiaInput };
  }

  function forkMassProperties(localPoints = activeForkBodyPoints()) {
    const mass = positive(state.params.forkMass, .03);
    const cleaned = removeDuplicateLoopPoints(localPoints || []).points;
    if (cleaned.length >= 3 && Math.abs(polygonArea(cleaned)) > 1e-12) {
      let twiceArea = 0; let centroidXNumerator = 0; let centroidYNumerator = 0; let polarNumerator = 0;
      for (let index = 0; index < cleaned.length; index += 1) {
        const a = cleaned[index]; const b = cleaned[(index + 1) % cleaned.length]; const signedCross = cross(a, b);
        twiceArea += signedCross;
        centroidXNumerator += (a.x + b.x) * signedCross;
        centroidYNumerator += (a.y + b.y) * signedCross;
        polarNumerator += signedCross * (dot(a, a) + dot(a, b) + dot(b, b));
      }
      const signedArea = twiceArea / 2; const area = Math.abs(signedArea);
      if (area > 1e-12) {
        const centroid = point(centroidXNumerator / (6 * signedArea), centroidYNumerator / (6 * signedArea));
        const polarAtOriginPerArea = Math.abs(polarNumerator / 12) / area;
        const radiusOfGyrationSquared = Math.max(1e-10, polarAtOriginPerArea - dot(centroid, centroid));
        return { mass, com: centroid, inertia: mass * radiusOfGyrationSquared, area, source: 'cad-uniform-lamina' };
      }
    }
    if (cleaned.length >= 2) {
      const start = cleaned[0]; const end = cleaned[cleaned.length - 1]; const span = length(subtract(end, start));
      return { mass, com: lerp(start, end, .5), inertia: Math.max(mass * span ** 2 / 12, 1e-10), area: 0, source: 'uniform-line' };
    }
    throw new Error('叉子有效几何不足，无法推导质心与惯量');
  }

  function createCadWeaponSolidColliders(world, body, loop, settings) {
    const RAPIER = state.rapier;
    const triangulator = globalThis.earcut?.default || globalThis.earcut;
    if (typeof triangulator !== 'function') throw new Error('武器 CAD 实体求解所需的 Earcut 模块未加载');
    const cleaned = removeDuplicateLoopPoints(loop); const { points, sourceIndexes } = cleaned;
    if (points.length < 3) throw new Error('武器 DXF 没有可闭合的外轮廓');

    // A closed imported weapon is a finite solid, not a one-sided polyline.
    // With the old polyline, an energetic target could cross to the unsolved
    // side of the edge; the independent exact-loop audit then saw overlap but
    // repeated subdivision could never produce a restoring manifold.  Earcut
    // supplies non-overlapping convex pieces on the same rigid body while the
    // original outer loop remains the authoritative 0.08 mm geometric gate and
    // source-index ruler.  No position projection or enlarged contact skin is
    // introduced here.
    const flat = points.flatMap((candidate) => [candidate.x, candidate.y]);
    const indices = triangulator(flat, null, 2);
    const boundaryEdgeKeys = points.map((_, index) => {
      const next = (index + 1) % points.length;
      return index < next ? `${index}:${next}` : `${next}:${index}`;
    });
    const colliders = []; const metadata = []; let triangulatedArea = 0;
    for (let offset = 0; offset + 2 < indices.length; offset += 3) {
      const triangleVertexIndexes = [indices[offset], indices[offset + 1], indices[offset + 2]];
      const triangleLoop = triangleVertexIndexes.map((index) => points[index]);
      const area = Math.abs(polygonArea(triangleLoop));
      if (area <= 1e-14) continue;
      const desc = RAPIER.ColliderDesc.triangle(triangleLoop[0], triangleLoop[1], triangleLoop[2]);
      configureRigCollider(desc, settings);
      const collider = world.createCollider(desc, body);
      if (!collider) throw new Error('武器 CAD 三角实体碰撞体创建失败');
      triangulatedArea += area;
      colliders.push(collider);
      metadata.push({
        kind: 'cad-weapon-solid-triangle',
        triangleOrder: metadata.length,
        triangleVertexIndexes,
        triangleLoop,
        boundaryEdgeKeys,
        loop: points,
        sourceIndexes,
      });
    }
    const outlineArea = Math.abs(polygonArea(points));
    const areaTolerance = Math.max(1e-12, outlineArea * 1e-8);
    if (!colliders.length || outlineArea <= 1e-12
      || Math.abs(triangulatedArea - outlineArea) > areaTolerance) {
      throw new Error(`武器 CAD 实体三角化未覆盖原外轮廓：轮廓 ${format(outlineArea * 1e6, 6)} mm²，三角形 ${format(triangulatedArea * 1e6, 6)} mm²`);
    }
    return {
      collider: colliders[0], colliders, metadata, points, sourceIndexes,
      mode: `闭合 CAD 三角复合实体（${colliders.length}）`,
    };
  }

  function createCadForkSolidColliders(world, body, loop, settings) {
    const RAPIER = state.rapier;
    const triangulator = globalThis.earcut?.default || globalThis.earcut;
    if (typeof triangulator !== 'function') throw new Error('叉子 CAD 实体求解所需的 Earcut 模块未加载');
    const cleaned = removeDuplicateLoopPoints(loop); const { points, sourceIndexes } = cleaned;
    if (points.length < 3) throw new Error('叉子 DXF 没有可闭合的外轮廓');

    // A concave fork must be solved as the same finite solid audited by the
    // exact 0.08 mm outer-loop gate. A polyline is only a one-sided boundary:
    // in a floor/fork/target wedge it can leave the target on the unsolved side
    // while the closed-loop audit correctly sees growing overlap. Earcut makes
    // a compound of non-overlapping convex triangles on this one rigid body;
    // it changes neither the entered mass properties nor the revolute joint.
    const flat = points.flatMap((candidate) => [candidate.x, candidate.y]);
    const indices = triangulator(flat, null, 2);
    const colliders = []; const metadata = []; let triangulatedArea = 0;
    for (let offset = 0; offset + 2 < indices.length; offset += 3) {
      const triangleVertexIndexes = [indices[offset], indices[offset + 1], indices[offset + 2]];
      const triangleLoop = triangleVertexIndexes.map((index) => points[index]);
      const area = Math.abs(polygonArea(triangleLoop));
      if (area <= 1e-14) continue;
      const desc = RAPIER.ColliderDesc.triangle(triangleLoop[0], triangleLoop[1], triangleLoop[2]);
      configureRigCollider(desc, settings);
      const collider = world.createCollider(desc, body);
      if (!collider) throw new Error('叉子 CAD 三角实体碰撞体创建失败');
      triangulatedArea += area;
      colliders.push(collider);
      metadata.push({
        kind: 'cad-fork-solid-triangle',
        triangleOrder: metadata.length,
        triangleVertexIndexes,
        triangleLoop,
        loop: points,
        sourceIndexes,
      });
    }
    const outlineArea = Math.abs(polygonArea(points));
    const areaTolerance = Math.max(1e-12, outlineArea * 1e-8);
    if (!colliders.length || outlineArea <= 1e-12
      || Math.abs(triangulatedArea - outlineArea) > areaTolerance) {
      throw new Error(`叉子 CAD 实体三角化未覆盖原外轮廓：轮廓 ${format(outlineArea * 1e6, 6)} mm²，三角形 ${format(triangulatedArea * 1e6, 6)} mm²`);
    }
    return {
      collider: colliders[0], colliders, metadata, points, sourceIndexes,
      mode: `闭合 CAD 三角复合实体（${colliders.length}）`,
    };
  }

  function createParameterWeaponColliders(world, body, settings) {
    const RAPIER = state.rapier; const geometry = parameterWeaponGeometry();
    const colliders = []; const metadata = [];
    geometry.segments.forEach((segment) => {
      const desc = RAPIER.ColliderDesc.triangle(segment.loop[0], segment.loop[1], segment.loop[2]);
      configureRigCollider(desc, settings);
      const collider = world.createCollider(desc, body);
      if (!collider) throw new Error(`第 ${segment.toothOrder + 1} 个测试齿无法建立有限三角背撑实体`);
      colliders.push(collider);
      metadata.push({
        kind: 'parametric-tooth', toothOrder: segment.toothOrder,
        start: { ...segment.start }, end: { ...segment.end }, edgeRadius: 0,
        loop: segment.loop.map((candidate) => point(candidate.x, candidate.y)),
        sourceIndexes: segment.loop.map((_, index) => index),
      });
    });
    return {
      colliders, metadata, loops: geometry.segments.map((segment) => segment.loop),
      mode: `参数迎击面 + ${format(geometry.width * 1000, 3)} mm 三角背撑`,
      materialRemovalDefined: false,
    };
  }

  function createParameterForkCollider(world, body, settings) {
    const RAPIER = state.rapier; const geometry = parameterForkGeometry();
    const desc = RAPIER.ColliderDesc.segment(geometry.root, geometry.tip);
    configureRigCollider(desc, settings);
    const collider = world.createCollider(desc, body);
    if (!collider) throw new Error('参数叉无法建立铰点至叉尖线段碰撞体');
    return {
      colliders: [collider], metadata: [{ kind: 'parametric-fork-segment', start: { ...geometry.root }, end: { ...geometry.tip }, loop: geometry.points }],
      points: geometry.points,
      mode: `参数零厚度线叉 · 长 ${format(geometry.tipDistance * 1000, 3)} mm`,
    };
  }

  function initialLoopTargetOverlapArea(localLoop, bodyPosition, bodyAngle, target) {
    const clipping = globalThis.polygonClipping;
    if (!clipping?.intersection) throw new Error('初始实体相交检查所需的精确多边形模块未加载');
    const targetLocalLoop = localLoop.map((local) => rotate(
      subtract(add(bodyPosition, rotate(local, bodyAngle)), target.pos),
      -target.angle,
    ));
    const intersection = clipping.intersection(
      clippingPolygonFromLoop(targetLocalLoop),
      initialTargetMaterialGeometry(),
    );
    return materialGeometryArea(intersection);
  }

  function initialSegmentCrossesTargetInterior(localStart, localEnd, bodyPosition, bodyAngle, target) {
    const toTarget = (local) => rotate(
      subtract(add(bodyPosition, rotate(local, bodyAngle)), target.pos),
      -target.angle,
    );
    const start = toTarget(localStart); const end = toTarget(localEnd); const delta = subtract(end, start);
    const halfX = Math.max(0, targetLength() / 2 - GEOMETRY_CLEARANCE_EPS);
    const halfY = Math.max(0, targetThickness() / 2 - GEOMETRY_CLEARANCE_EPS);
    let minimum = 0; let maximum = 1;
    const clipAxis = (origin, direction, low, high) => {
      if (Math.abs(direction) <= 1e-14) return origin >= low && origin <= high;
      let enter = (low - origin) / direction; let leave = (high - origin) / direction;
      if (enter > leave) [enter, leave] = [leave, enter];
      minimum = Math.max(minimum, enter); maximum = Math.min(maximum, leave);
      return minimum <= maximum;
    };
    return clipAxis(start.x, delta.x, -halfX, halfX)
      && clipAxis(start.y, delta.y, -halfY, halfY)
      && maximum >= 0 && minimum <= 1;
  }

  function createMaterialBoundaryColliders(world, body, geometry, settings) {
    const RAPIER = state.rapier; const colliders = [];
    (geometry || []).forEach((polygon) => polygon.forEach((ring) => {
      const open = ring.length > 1
        && Math.hypot(ring[0][0] - ring[ring.length - 1][0], ring[0][1] - ring[ring.length - 1][1]) <= 1e-12
        ? ring.slice(0, -1) : ring;
      if (open.length < 2) return;
      const vertices = new Float32Array(open.flatMap(([x, y]) => [x, y]));
      const edges = new Uint32Array(open.flatMap((_, index) => [index, (index + 1) % open.length]));
      const desc = RAPIER.ColliderDesc.polyline(vertices, edges);
      if (!desc) throw new Error('材料边界无法建立闭合折线碰撞体');
      configureRigCollider(desc, settings);
      const collider = world.createCollider(desc, body);
      if (!collider) throw new Error('材料边界碰撞体创建失败');
      colliders.push(collider);
    }));
    return colliders;
  }

  function weaponMassConsistencyError(metrics, forkMass, totalRobotMass) {
    const weaponMass = Number(metrics?.weaponMass);
    if (!Number.isFinite(weaponMass) || weaponMass <= 0) {
      return '武器旋转组件质量必须是大于 0 kg 的有限实测/等效值；转动惯量会由这项质量与当前几何自动计算';
    }
    if (!Number.isFinite(metrics?.inertia) || metrics.inertia <= 0) {
      return '当前武器几何不足，无法由质量自动计算有限的轴心转动惯量；请检查 DXF 闭环/线段或参数齿尺寸';
    }
    if (!(weaponMass + forkMass < totalRobotMass)) {
      return `整机质量预算不自洽：武器旋转组件 ${format(weaponMass, 4)} kg + 叉子 ${format(forkMass, 4)} kg 不小于整机质量 ${format(totalRobotMass, 4)} kg；底盘质量必须严格大于 0 kg`;
    }
    return null;
  }

  function createRapierRig(target, initialAngle, initialOmega) {
    const RAPIER = state.rapier;
    if (!USE_RAPIER_RIG || !RAPIER) return null;
    try {
      const geometryStatus = parameterGeometryStatus();
      if (!geometryStatus.valid) throw new Error(geometryStatus.reason);
      const world = new RAPIER.World({ x: 0, y: -GRAVITY });
      world.lengthUnit = RAPIER_LENGTH_UNIT;
      world.integrationParameters.normalizedAllowedLinearError = RAPIER_ALLOWED_LINEAR_ERROR;
      world.integrationParameters.normalizedPredictionDistance = RAPIER_PREDICTION_DISTANCE;
      world.integrationParameters.dt = FIXED_DT;
      world.maxCcdSubsteps = 16;
      world.numSolverIterations = 16;
      world.numInternalPgsIterations = 4;

      const materialMode = state.params.contactModel === 'material';
      const materialProperties = targetMaterialProperties();
      const materialCuttingRequested = materialMode
        && !state.params.paramWeaponEnabled
        && Math.max(0, number(materialProperties.minChipThickness)) > 0
        && Boolean(materialProperties.minChipSource)
        && Math.max(0, number(materialProperties.fractureEnergy)) > 0
        && Boolean(materialProperties.fractureSource);
      const materialCuttingEnabled = TRACEABLE_CUTTING_CONTINUOUS_TOI_ENABLED
        && materialCuttingRequested;
      // Custom material impulses must not replace Rapier's non-penetration
      // constraint until the complete CAD/parameter-tooth boundary has a
      // validated earliest-contact TOI and remainder replay.  This applies to
      // both imported CAD and parameter teeth; otherwise a post-step response
      // can miss entry, leave the tooth embedded, and inject numerical energy.
      const materialResponseEnabled = materialMode && materialCuttingEnabled;
      if (materialCuttingEnabled) {
        const removableSliceMass = targetMaterialProperties().density
          * targetLength() * targetThickness() * effectiveToolWidthZ('weapon');
        if (effectiveTargetMass() + 1e-9 < removableSliceMass) {
          throw new Error(`靶子输入质量 ${format(effectiveTargetMass(), 5)} kg 小于当前密度、XY 尺寸和武器 Z 重叠所要求的材料切片质量 ${format(removableSliceMass, 5)} kg；质量与材料几何不自洽，不能建立切削刚体`);
        }
      }
      // In the 2.5D material model the uncut Z backing remains the structural
      // body seen by floor/fork, while the weapon sees only the exact boundary
      // of its own Z-overlap slice.  This prevents an 8 mm groove from erasing
      // floor/fork support across a 40 mm target width.
      const structuralFilter = RIG_GROUP_FLOOR | RIG_GROUP_FORK | (materialResponseEnabled ? 0 : RIG_GROUP_WEAPON);
      const targetGroups = rapierInteractionGroups(RIG_GROUP_TARGET, structuralFilter);
      const targetMaterialGroups = rapierInteractionGroups(RIG_GROUP_TARGET, materialResponseEnabled ? RIG_GROUP_WEAPON : 0);
      const floorGroups = rapierInteractionGroups(RIG_GROUP_FLOOR, RIG_GROUP_TARGET);
      const forkGroundGroups = rapierInteractionGroups(RIG_GROUP_FLOOR, RIG_GROUP_FORK);
      const forkGroups = rapierInteractionGroups(RIG_GROUP_FORK, effectiveToolWidthZ('fork') > 1e-9 ? (RIG_GROUP_TARGET | RIG_GROUP_FLOOR) : RIG_GROUP_FLOOR);
      const weaponGroups = rapierInteractionGroups(RIG_GROUP_WEAPON, effectiveToolWidthZ('weapon') > 1e-9 ? RIG_GROUP_TARGET : 0);
      // Material-mode weapon contacts are detected by Rapier but deliberately
      // omitted from its infinite-stiffness contact solver.  The local cutting
      // law below supplies one finite, equal-and-opposite impulse and removes
      // the matching fresh CAD swept volume.  Floor/fork and rigid-reference
      // contacts remain ordinary Rapier constraints.
      const targetSolverGroups = rapierInteractionGroups(
        RIG_GROUP_TARGET,
        structuralFilter,
      );
      const targetMaterialSolverGroups = rapierInteractionGroups(RIG_GROUP_TARGET, 0);
      const floorSolverGroups = rapierInteractionGroups(RIG_GROUP_FLOOR, RIG_GROUP_TARGET);
      const forkGroundSolverGroups = rapierInteractionGroups(RIG_GROUP_FLOOR, RIG_GROUP_FORK);
      const forkSolverGroups = rapierInteractionGroups(RIG_GROUP_FORK, RIG_GROUP_TARGET | RIG_GROUP_FLOOR);
      const weaponSolverGroups = rapierInteractionGroups(RIG_GROUP_WEAPON, materialResponseEnabled ? 0 : RIG_GROUP_TARGET);

      const floorBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, targetSupportY()));
      const floorDesc = configureRigCollider(
        // The arena floor is mathematically unbounded in X.  A finite 100 m
        // cuboid let energetic targets run off its edge while the geometry gate
        // still assumed an infinite support plane, producing a false domain
        // stop followed by free fall in diagnostics.
        RAPIER.ColliderDesc.halfspace({ x: 0, y: 1 }),
        { friction: targetFloorFriction().kinetic, restitution: getFloorMaterial().restitution, groups: floorGroups, solverGroups: floorSolverGroups, contactSkin: RIG_FLOOR_PENETRATION_TOLERANCE },
      );
      const floorCollider = world.createCollider(floorDesc, floorBody);
      const forkGroundBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, groundY()));
      const forkGroundDesc = configureRigCollider(
        RAPIER.ColliderDesc.halfspace({ x: 0, y: 1 }),
        { friction: 1, restitution: 0, groups: forkGroundGroups, solverGroups: forkGroundSolverGroups, contactSkin: 0 },
      );
      const forkGroundCollider = world.createCollider(forkGroundDesc, forkGroundBody);

      const targetBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(target.pos.x, target.pos.y)
        .setRotation(target.angle)
        .setLinvel(target.vel.x, target.vel.y)
        .setAngvel(target.omega)
        .setCcdEnabled(true)
        .setCanSleep(true)
        .setAdditionalMassProperties(effectiveTargetMass(), { x: 0, y: 0 }, Math.max(targetInertia(), 1e-10));
      const targetBody = world.createRigidBody(targetBodyDesc);
      targetBody.enableCcd(true); targetBody.setSoftCcdPrediction?.(RAPIER_PREDICTION_DISTANCE); targetBody.setAdditionalSolverIterations(32);
      const targetDesc = configureRigCollider(
        RAPIER.ColliderDesc.cuboid(targetLength() / 2, targetThickness() / 2),
        { friction: 1, restitution: 0, groups: targetGroups, solverGroups: targetSolverGroups, density: 0 },
      );
      const targetCollider = world.createCollider(targetDesc, targetBody);
      const targetMaterialColliders = materialResponseEnabled
        ? createMaterialBoundaryColliders(world, targetBody, initialTargetMaterialGeometry(), {
          friction: 0,
          restitution: 0,
          groups: targetMaterialGroups,
          solverGroups: targetMaterialSolverGroups,
          density: 0,
        })
        : [];

      const baseOrigin = weaponBaseSceneOrigin();
      const initialDrive = configuredInitialDriveSpeed();
      const totalRobotMass = positive(state.params.robotMass, .01);
      const forkLocalPoints = activeForkSolidLoop();
      const minimumForkPointCount = state.params.paramForkEnabled ? 2 : 3;
      if (forkLocalPoints.length < minimumForkPointCount) throw new Error(state.params.paramForkEnabled ? '参数叉必须有一根有限线段才能进入刚体求解' : '叉子必须有闭合外轮廓才能进入刚体求解');
      const forkProperties = forkMassProperties(forkLocalPoints);
      // A rotor with all of its mass inside radius R must satisfy I <= m R^2.
      // I/R² remains only a consistency lower bound.  It must never be used to
      // manufacture a translational mass because joint reaction and target
      // impulse depend on the complete rotating assembly's real mass.
      const weaponMass = Number(state.params.weaponMass);
      const massError = weaponMassConsistencyError(state.metrics, forkProperties.mass, totalRobotMass);
      if (massError) throw new Error(massError);
      const chassisMass = totalRobotMass - weaponMass - forkProperties.mass;
      const robotBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(baseOrigin.x, baseOrigin.y)
        .setLinvel(initialDrive, 0)
        .setGravityScale(0)
        .enabledTranslations(true, false)
        .lockRotations()
        .setCanSleep(false)
        .setAdditionalMass(chassisMass);
      const robotBody = world.createRigidBody(robotBodyDesc);
      robotBody.setAdditionalSolverIterations(32);

      const weaponBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(baseOrigin.x, baseOrigin.y)
        .setRotation(initialAngle)
        .setLinvel(initialDrive, 0)
        .setAngvel(initialOmega)
        .setGravityScale(0)
        .setCcdEnabled(true)
        .setCanSleep(false)
        // Translation is carried by the revolute joint. The complete rotating
        // mass controls contact response; its pivot inertia is derived from the
        // currently active CAD/parameter shape, never entered independently.
        .setAdditionalMassProperties(weaponMass, { x: 0, y: 0 }, Math.max(state.metrics.inertia, 1e-9));
      const weaponBody = world.createRigidBody(weaponBodyDesc);
      weaponBody.enableCcd(true); weaponBody.setSoftCcdPrediction?.(RAPIER_PREDICTION_DISTANCE); weaponBody.setAdditionalSolverIterations(32);

      const forkPivot = initialForkPivotAt(baseOrigin);
      const initialForkPose = initialForkGroundPose(forkLocalPoints, forkPivot);
      if (!initialForkPose.valid) throw new Error(initialForkPose.reason);
      const forkBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(forkPivot.x, forkPivot.y)
        .setRotation(initialForkPose.angle)
        .setLinvel(initialDrive, 0)
        .setAngvel(0)
        .setGravityScale(1)
        .setCcdEnabled(true)
        .setCanSleep(true)
        .setAdditionalMassProperties(forkProperties.mass, forkProperties.com, Math.max(forkProperties.inertia, 1e-10));
      const forkBody = world.createRigidBody(forkBodyDesc);
      forkBody.enableCcd(true); forkBody.setSoftCcdPrediction?.(RAPIER_PREDICTION_DISTANCE); forkBody.setAdditionalSolverIterations(32);

      const weaponSettings = {
        label: '武器', friction: weaponTargetFriction(), restitution: clamp(number(state.params.restitution), 0, .25), groups: weaponGroups, solverGroups: weaponSolverGroups, contactSkin: RIG_CONTACT_SKIN,
      };
      let weaponAssembly;
      if (state.params.paramWeaponEnabled) {
        weaponAssembly = createParameterWeaponColliders(world, weaponBody, weaponSettings);
      } else {
        const weaponGeometry = weaponCadCollisionGeometry();
        if (!weaponGeometry.hasClosedOutline || weaponGeometry.sourceOutline.length < 3) throw new Error('武器 DXF 必须有闭合外轮廓才能进入刚体求解');
        const weaponSolid = createCadWeaponSolidColliders(world, weaponBody, weaponGeometry.sourceOutline.map(weaponLocal), weaponSettings);
        weaponAssembly = {
          colliders: weaponSolid.colliders,
          metadata: weaponSolid.metadata,
          loops: [weaponSolid.points], mode: weaponSolid.mode, materialRemovalDefined: true,
        };
      }
      const weaponToothLoops = state.params.paramWeaponEnabled ? [] : cadToothCuttingLoops();

      const forkSettings = {
        label: '叉子', friction: shovelTargetFriction(), restitution: 0, groups: forkGroups, solverGroups: forkSolverGroups, contactSkin: RIG_CONTACT_SKIN,
      };
      let forkAssembly;
      if (state.params.paramForkEnabled) {
        forkAssembly = createParameterForkCollider(world, forkBody, forkSettings);
      } else {
        const forkSolid = createCadForkSolidColliders(world, forkBody, forkLocalPoints, forkSettings);
        forkAssembly = {
          colliders: forkSolid.colliders, metadata: forkSolid.metadata,
          points: forkSolid.points, mode: forkSolid.mode,
        };
      }

      if (effectiveToolWidthZ('weapon') > 1e-9) {
        for (const loop of weaponAssembly.loops) {
          const overlap = initialLoopTargetOverlapArea(loop, baseOrigin, initialAngle, target);
          if (overlap > 1e-12) throw new Error(`武器与靶子初始实体重叠 ${format(overlap * 1e6, 6)} mm²；请调整场地坐标或武器初始角度，求解器不会从埋入状态启动`);
        }
      }
      if (effectiveToolWidthZ('fork') > 1e-9) {
        const invalidForkStart = state.params.paramForkEnabled
          ? initialSegmentCrossesTargetInterior(forkAssembly.points[0], forkAssembly.points[1], forkPivot, initialForkPose.angle, target)
          : initialLoopTargetOverlapArea(forkAssembly.points, forkPivot, initialForkPose.angle, target) > 1e-12;
        if (invalidForkStart) throw new Error('叉子与靶子初始实体重叠；请调整靶子场地 X，求解器不会用位置投影把实体推出');
      }
      const initialForkBottom = Math.min(...forkAssembly.points.map((local) => forkPivot.y + rotate(local, initialForkPose.angle).y));
      if (initialForkBottom < groundY() - GEOMETRY_CLEARANCE_EPS) {
        throw new Error(`叉子解析装配姿态低于真实地面 ${format((groundY() - initialForkBottom) * 1000, 5)} mm；求解器不会从埋入状态启动`);
      }
      // This local-space AABB is immutable for the lifetime of the rig. The
      // near-contact broad phase rotates its four corners conservatively instead
      // of rebuilding the full world-space fork outline on every 0.5 ms tick.
      const forkLocalBounds = { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity };
      let forkRadius = 0;
      forkAssembly.points.forEach((local) => {
        forkLocalBounds.minX = Math.min(forkLocalBounds.minX, local.x);
        forkLocalBounds.maxX = Math.max(forkLocalBounds.maxX, local.x);
        forkLocalBounds.minY = Math.min(forkLocalBounds.minY, local.y);
        forkLocalBounds.maxY = Math.max(forkLocalBounds.maxY, local.y);
        forkRadius = Math.max(forkRadius, length(local));
      });
      const weaponColliderMeta = new Map(weaponAssembly.colliders.map((collider, index) => [collider.handle, weaponAssembly.metadata[index]]));
      const forkColliderMeta = new Map(forkAssembly.colliders.map((collider, index) => [collider.handle, forkAssembly.metadata[index]]));

      const weaponJoint = world.createImpulseJoint(
        RAPIER.JointData.revolute({ x: 0, y: 0 }, { x: 0, y: 0 }),
        robotBody,
        weaponBody,
        true,
      );
      const forkJoint = world.createImpulseJoint(
        RAPIER.JointData.revolute({ x: forkPivotOffset().x, y: forkPivotOffset().y }, { x: 0, y: 0 }),
        robotBody,
        forkBody,
        true,
      );
      return {
        rigModel: true,
        materialMode,
        materialResponseEnabled,
        materialCuttingRequested,
        materialCuttingEnabled,
        world,
        floorBody,
        floorCollider,
        forkGroundBody,
        forkGroundCollider,
        targetBody,
        targetCollider,
        targetColliders: [targetCollider, ...targetMaterialColliders],
        targetMaterialColliders,
        robotBody,
        weaponBody,
        weaponJoint,
        forkBody,
        forkJoint,
        weaponCollider: weaponAssembly.colliders[0],
        weaponColliders: weaponAssembly.colliders,
        weaponColliderMeta,
        weaponColliderMetadata: weaponAssembly.metadata,
        forkCollider: forkAssembly.colliders[0],
        forkColliders: forkAssembly.colliders,
        forkColliderMeta,
        forkColliderMetadata: forkAssembly.metadata,
        weaponLoop: weaponAssembly.loops[0],
        weaponLoops: weaponAssembly.loops,
        weaponToothLoops,
        weaponSourceIndexes: weaponAssembly.metadata[0]?.sourceIndexes || [],
        forkLoop: forkAssembly.points,
        forkLocalBounds,
        forkRadius,
        weaponMode: weaponAssembly.mode,
        forkMode: forkAssembly.mode,
        weaponGeometrySource: state.params.paramWeaponEnabled ? 'parametric' : 'cad',
        forkGeometrySource: state.params.paramForkEnabled ? 'parametric' : 'cad',
        materialRemovalDefined: weaponAssembly.materialRemovalDefined,
        targetGroups,
        targetSolverGroups,
        targetMaterialGroups,
        targetMaterialSolverGroups,
        weaponMass,
        forkMass: forkProperties.mass,
        forkMassProperties: forkProperties,
        initialForkAngle: initialForkPose.angle,
        initialForkTip: initialForkPose.tip,
        chassisMass,
      };
    } catch (error) {
      state.rapierError = error?.message || 'Rapier CAD 刚体创建失败';
      return null;
    }
  }

  function snapshotRapierRig(physics) {
    if (!physics?.rigModel || !physics.world?.takeSnapshot) return null;
    return {
      bytes: physics.world.takeSnapshot(),
      handles: {
        floorBody: physics.floorBody.handle,
        floorCollider: physics.floorCollider.handle,
        forkGroundBody: physics.forkGroundBody.handle,
        forkGroundCollider: physics.forkGroundCollider.handle,
        targetBody: physics.targetBody.handle,
        targetCollider: physics.targetCollider.handle,
        targetColliders: (physics.targetColliders || [physics.targetCollider]).map((collider) => collider.handle),
        targetMaterialColliders: (physics.targetMaterialColliders || []).map((collider) => collider.handle),
        robotBody: physics.robotBody.handle,
        weaponBody: physics.weaponBody.handle,
        forkBody: physics.forkBody.handle,
        weaponCollider: physics.weaponCollider.handle,
        weaponColliders: (physics.weaponColliders || [physics.weaponCollider]).map((collider) => collider.handle),
        forkCollider: physics.forkCollider.handle,
        forkColliders: (physics.forkColliders || [physics.forkCollider]).map((collider) => collider.handle),
        weaponJoint: physics.weaponJoint.handle,
        forkJoint: physics.forkJoint.handle,
      },
      mlcpV2ClusterActive: Boolean(physics.mlcpV2ClusterActive),
      mlcpV2HandoffTick: Number.isFinite(physics.mlcpV2HandoffTick) ? physics.mlcpV2HandoffTick : null,
      mlcpV2Audit: clonePlaybackPlain(physics.mlcpV2Audit || null),
      geometryMetadata: {
        // This metadata is immutable for the lifetime of a rig. Keeping the
        // references avoids rebuilding every CAD point at all 8,000 fixed
        // ticks; the mutable Rapier world is still fully snapshotted.
        weapon: physics.weaponColliderMetadata || [],
        fork: physics.forkColliderMetadata || [],
      },
    };
  }

  function restoreRapierRig(physics, snapshot) {
    const RAPIER = state.rapier;
    if (!physics?.rigModel || !snapshot?.bytes || !RAPIER?.World?.restoreSnapshot) return false;
    try {
      const oldWorld = physics.world;
      const world = RAPIER.World.restoreSnapshot(snapshot.bytes);
      const h = snapshot.handles;
      const restored = {
        floorBody: world.getRigidBody(h.floorBody),
        floorCollider: world.getCollider(h.floorCollider),
        forkGroundBody: world.getRigidBody(h.forkGroundBody),
        forkGroundCollider: world.getCollider(h.forkGroundCollider),
        targetBody: world.getRigidBody(h.targetBody),
        targetCollider: world.getCollider(h.targetCollider),
        targetColliders: (h.targetColliders || [h.targetCollider]).map((handle) => world.getCollider(handle)).filter(Boolean),
        targetMaterialColliders: (h.targetMaterialColliders || []).map((handle) => world.getCollider(handle)).filter(Boolean),
        robotBody: world.getRigidBody(h.robotBody),
        weaponBody: world.getRigidBody(h.weaponBody),
        forkBody: world.getRigidBody(h.forkBody),
        weaponCollider: world.getCollider(h.weaponCollider),
        weaponColliders: (h.weaponColliders || [h.weaponCollider]).map((handle) => world.getCollider(handle)).filter(Boolean),
        forkCollider: world.getCollider(h.forkCollider),
        forkColliders: (h.forkColliders || [h.forkCollider]).map((handle) => world.getCollider(handle)).filter(Boolean),
        weaponJoint: world.getImpulseJoint(h.weaponJoint),
        forkJoint: world.getImpulseJoint(h.forkJoint),
      };
      const weaponMetadata = snapshot.geometryMetadata?.weapon || physics.weaponColliderMetadata || [];
      const forkMetadata = snapshot.geometryMetadata?.fork || physics.forkColliderMetadata || [];
      restored.weaponColliderMetadata = weaponMetadata;
      restored.forkColliderMetadata = forkMetadata;
      restored.weaponColliderMeta = new Map(restored.weaponColliders.map((collider, index) => [collider.handle, weaponMetadata[index] || { kind: 'cad-weapon' }]));
      restored.forkColliderMeta = new Map(restored.forkColliders.map((collider, index) => [collider.handle, forkMetadata[index] || { kind: 'cad-fork' }]));
      if (!restored.targetBody || !restored.robotBody || !restored.weaponBody || !restored.forkBody || !restored.targetCollider || !restored.targetColliders.length || !restored.weaponCollider || !restored.weaponColliders.length || !restored.forkCollider || !restored.forkColliders.length || !restored.forkGroundCollider || !restored.forkJoint) {
        world.free?.();
        return false;
      }
      physics.world = world;
      Object.assign(physics, restored);
      physics.mlcpV2ClusterActive = Boolean(snapshot.mlcpV2ClusterActive);
      physics.mlcpV2HandoffTick = Number.isFinite(snapshot.mlcpV2HandoffTick) ? snapshot.mlcpV2HandoffTick : null;
      physics.mlcpV2Audit = clonePlaybackPlain(snapshot.mlcpV2Audit || null);
      window.__MlcpV2Stats = physics.mlcpV2Audit;
      try { oldWorld?.free?.(); } catch (_) { /* restored world is authoritative */ }
      return true;
    } catch (error) {
      state.rapierError = error?.message || 'Rapier 刚体快照恢复失败';
      return false;
    }
  }

  function syncRigStateFromPhysics() {
    const sim = state.sim; const physics = sim?.physics;
    if (!physics?.rigModel) return false;
    const robotPosition = physics.robotBody.translation(); const robotVelocity = physics.robotBody.linvel();
    const weaponPosition = physics.weaponBody.translation(); const forkPosition = physics.forkBody.translation();
    const forkVelocity = physics.forkBody.linvel();
    const targetPosition = physics.targetBody.translation(); const targetVelocity = physics.targetBody.linvel();
    sim.robotTravel = robotPosition.x - weaponBaseSceneOrigin().x;
    sim.driveSpeed = robotVelocity.x;
    sim.angle = physics.weaponBody.rotation();
    sim.weaponOmega = physics.weaponBody.angvel();
    sim.forkOrigin = point(forkPosition.x, forkPosition.y);
    sim.forkVelocity = point(forkVelocity.x, forkVelocity.y);
    sim.forkAngle = physics.forkBody.rotation();
    sim.forkOmega = physics.forkBody.angvel();
    sim.target.pos = point(targetPosition.x, targetPosition.y);
    sim.target.vel = point(targetVelocity.x, targetVelocity.y);
    sim.target.angle = physics.targetBody.rotation();
    sim.target.omega = physics.targetBody.angvel();
    const lowest = targetLowestPoint();
    sim.rigMinFloorClearance = Math.min(number(sim.rigMinFloorClearance, Infinity), lowest - groundY());
    sim.target.grounded = lowest <= targetSupportY() + FLOOR_SKIN * 2 && Math.abs(sim.target.vel.y) < .06;
    sim.targetLaunched = !sim.target.grounded;
    // A revolute joint should keep both origins coincident. A large drift is a
    // solver-domain failure, never repaired with a hidden translation.
    sim.rigJointError = Math.hypot(weaponPosition.x - robotPosition.x, weaponPosition.y - robotPosition.y);
    const expectedForkPivot = add(point(robotPosition.x, robotPosition.y), forkPivotOffset());
    sim.rigForkJointError = Math.hypot(forkPosition.x - expectedForkPivot.x, forkPosition.y - expectedForkPivot.y);
    const forkBottom = Math.min(...physics.forkLoop.map((local) => add(point(forkPosition.x, forkPosition.y), rotate(local, sim.forkAngle)).y));
    sim.rigForkMinFloorClearance = Math.min(number(sim.rigForkMinFloorClearance, Infinity), forkBottom - groundY());
    sim.forkGrounded = forkBottom <= groundY() + FLOOR_SKIN * 2 && Math.abs(forkVelocity.y) < .06;
    return true;
  }

  function physicalDriveForce(speed, dt = FIXED_DT) {
    const metrics = state.metrics;
    const noLoad = Math.max(metrics.commandVehicleSpeed, 0);
    const forwardSpeed = Math.max(0, speed);
    const torqueFraction = noLoad > 1e-9 ? clamp(1 - forwardSpeed / noLoad, 0, 1) : 0;
    const motorForce = Math.max(0, metrics.motorTraction * torqueFraction);
    const powerForce = metrics.drivePowerOut / Math.max(Math.abs(speed), .15);
    const motorApplied = Math.min(motorForce, metrics.gripTraction, powerForce);
    const rollingLimit = positive(state.params.robotMass, .01) * Math.abs(speed) / Math.max(dt, 1e-9);
    const rollingMagnitude = Math.abs(speed) > 1e-8
      ? Math.min(metrics.rollingResistanceForce, rollingLimit)
      : 0;
    const rollingApplied = -Math.sign(speed) * rollingMagnitude;
    const applied = motorApplied + rollingApplied;
    const slip = motorForce > 1e-9 ? clamp((motorForce - metrics.gripTraction) / motorForce, 0, 1) : 0;
    return { applied, motorApplied, rollingApplied, motorForce, slip };
  }

  function applyRigActuation(dt = FIXED_DT) {
    const sim = state.sim; const physics = sim.physics;
    const chassisSpeed = physics.robotBody.linvel().x;
    const drive = physicalDriveForce(chassisSpeed, dt);
    if (Math.abs(drive.applied) > 0) physics.robotBody.addForce({ x: drive.applied, y: 0 }, true);
    sim.driveSlip = drive.slip;
    const actuation = {
      driveForce: drive.motorApplied,
      rollingForce: drive.rollingApplied,
      chassisSpeedBefore: chassisSpeed,
      weaponTorque: 0,
      weaponOmegaBefore: physics.weaponBody.angvel(),
    };

    const motorTorque = weaponMotorTorqueAt(physics.weaponBody.angvel());
    if (Math.abs(motorTorque) > 0) {
      actuation.weaponTorque = motorTorque;
      physics.weaponBody.addTorque(actuation.weaponTorque, true);
    }
    return actuation;
  }

  function weaponMotorTorqueAt(omega) {
    const direction = Math.sign(nominalSignedOmega()) || 1;
    const speed = Math.max(0, direction * omega); const noLoad = Math.max(state.metrics.angularVelocity, 1e-6);
    const commanded = Math.max(0, state.metrics.weaponCommandAngularVelocity);
    if (commanded <= 1e-9 || speed >= commanded) return 0;
    // Ideal brushed/BLDC equivalent with a linear torque-speed line. If the
    // entered mechanical peak power is Pmax, tau_stall = 4 Pmax / omega_0.
    // The advanced current field may impose a stricter torque ceiling.
    const torqueFraction = clamp(1 - speed / noLoad, 0, 1);
    const powerDerivedTorque = 4 * state.metrics.weaponPowerOut / noLoad * torqueFraction;
    // The advanced button only controls whether the engineering inputs are
    // visible.  It must never change the motor represented by an unchanged
    // parameter set.  A positive *phase* current is a constant electromagnetic
    // torque ceiling; zero means that no verified phase-current limit is known,
    // so this branch adds no artificial current cap.  Battery-side prop-test
    // current is deliberately not substituted here.
    const currentLimitedTorque = Number.isFinite(state.metrics.weaponCurrentLimitedTorque)
      ? Math.max(0, state.metrics.weaponCurrentLimitedTorque)
      : Infinity;
    return direction * Math.min(powerDerivedTorque, currentLimitedTorque);
  }

  function rigActuationWork(actuation, dt) {
    if (!actuation) return 0;
    const physics = state.sim.physics;
    const chassisAfter = physics.robotBody.linvel().x;
    const omegaAfter = physics.weaponBody.angvel();
    return actuation.driveForce * (actuation.chassisSpeedBefore + chassisAfter) * .5 * dt
      + actuation.weaponTorque * (actuation.weaponOmegaBefore + omegaAfter) * .5 * dt;
  }

  function rigNearCadContact() {
    const sim = state.sim; const physics = sim.physics;
    const halfDiagonal = Math.hypot(targetLength(), targetThickness()) / 2;
    const targetPosition = physics.targetBody.translation(); const robotPosition = physics.robotBody.translation();
    const forkPosition = physics.forkBody.translation(); const forkAngle = physics.forkBody.rotation();
    const targetVelocity = physics.targetBody.linvel(); const robotVelocity = physics.robotBody.linvel(); const forkVelocity = physics.forkBody.linvel();
    const targetAngularSweep = Math.abs(physics.targetBody.angvel()) * halfDiagonal * FIXED_DT;
    const closingMargin = Math.hypot(targetVelocity.x - robotVelocity.x, targetVelocity.y - robotVelocity.y) * FIXED_DT
      + targetAngularSweep + .004;
    const nearWeapon = Math.hypot(targetPosition.x - robotPosition.x, targetPosition.y - robotPosition.y)
      <= activeWeaponRadius() + halfDiagonal + closingMargin;
    const forkBounds = physics.forkLocalBounds;
    const forkCos = Math.cos(forkAngle); const forkSin = Math.sin(forkAngle);
    // Extrema of the rotated local AABB are exactly the extrema of its four
    // corners. Since that box contains the real fork loop, this can only make
    // the exact gate run early; it can never skip a possible contact.
    const forkMinX = forkPosition.x
      + Math.min(forkCos * forkBounds.minX, forkCos * forkBounds.maxX)
      + Math.min(-forkSin * forkBounds.minY, -forkSin * forkBounds.maxY);
    const forkMaxX = forkPosition.x
      + Math.max(forkCos * forkBounds.minX, forkCos * forkBounds.maxX)
      + Math.max(-forkSin * forkBounds.minY, -forkSin * forkBounds.maxY);
    const forkMinY = forkPosition.y
      + Math.min(forkSin * forkBounds.minX, forkSin * forkBounds.maxX)
      + Math.min(forkCos * forkBounds.minY, forkCos * forkBounds.maxY);
    const forkMaxY = forkPosition.y
      + Math.max(forkSin * forkBounds.minX, forkSin * forkBounds.maxX)
      + Math.max(forkCos * forkBounds.minY, forkCos * forkBounds.maxY);
    const forkRadius = physics.forkRadius ?? Math.max(0, ...physics.forkLoop.map(length));
    const forkAngularSweep = Math.abs(physics.forkBody.angvel()) * forkRadius * FIXED_DT;
    const forkClosingMargin = Math.hypot(targetVelocity.x - forkVelocity.x, targetVelocity.y - forkVelocity.y) * FIXED_DT
      + targetAngularSweep + forkAngularSweep + .004;
    const forkTargetOverlapEnabled = effectiveToolWidthZ('fork') > 1e-9;
    const nearFork = forkTargetOverlapEnabled
      && targetPosition.x + halfDiagonal >= forkMinX - forkClosingMargin
      && targetPosition.x - halfDiagonal <= forkMaxX + forkClosingMargin
      && targetPosition.y + halfDiagonal >= forkMinY - forkClosingMargin
      && targetPosition.y - halfDiagonal <= forkMaxY + forkClosingMargin;
    const nearFloor = targetLowestPoint() - targetSupportY()
      <= Math.abs(targetVelocity.y) * FIXED_DT + targetAngularSweep + .004;
    return {
      nearWeapon,
      nearFork,
      nearFloor,
      nearForkFloor: forkMinY - groundY() <= Math.abs(forkVelocity.y) * FIXED_DT + forkAngularSweep + .004,
      targetHalfDiagonal: halfDiagonal,
      targetAngularSweep,
      forkRadius,
      forkAngularSweep,
      relativeLinearSpeed: Math.max(
        Math.hypot(targetVelocity.x - robotVelocity.x, targetVelocity.y - robotVelocity.y),
        forkTargetOverlapEnabled
          ? Math.hypot(targetVelocity.x - forkVelocity.x, targetVelocity.y - forkVelocity.y)
          : 0,
      ),
    };
  }

  function closestLoopSegmentIndex(localPoint, loop) {
    let bestIndex = -1; let bestDistanceSq = Infinity;
    for (let index = 0; index < loop.length; index += 1) {
      const start = loop[index]; const end = loop[(index + 1) % loop.length]; const delta = subtract(end, start);
      const denominator = dot(delta, delta); const fraction = denominator > 1e-15 ? clamp(dot(subtract(localPoint, start), delta) / denominator, 0, 1) : 0;
      const nearest = add(start, scalePoint(delta, fraction)); const offset = subtract(localPoint, nearest); const distanceSq = dot(offset, offset);
      if (distanceSq < bestDistanceSq) { bestDistanceSq = distanceSq; bestIndex = index; }
    }
    return bestIndex;
  }

  function toothOrderForBoundaryIndex(index) {
    const geometry = weaponCadCollisionGeometry(); const count = geometry.sourceOutline.length;
    if (index < 0 || !geometry.collisionIndexes.includes(index)) return null;
    const inRange = (value, start, end) => start <= end ? value >= start && value <= end : value >= start || value <= end;
    const order = geometry.teeth.findIndex((feature) => inRange(index, circularIndex(feature.startIndex, count), circularIndex(feature.endIndex, count)));
    return order >= 0 ? order : null;
  }

  function classifyWeaponContact(worldPoint, physics, collider) {
    const metadata = physics.weaponColliderMeta?.get(collider?.handle);
    if (metadata?.kind === 'parametric-tooth') {
      const origin = physics.weaponBody.translation();
      const local = rotate(subtract(worldPoint, point(origin.x, origin.y)), -physics.weaponBody.rotation());
      const loop = metadata.loop || physics.weaponLoop;
      const overlapCount = (physics.weaponColliderMetadata || []).filter((entry) => entry.kind === 'parametric-tooth'
        && entry.loop
        && (pointInsideSimpleLoop(local, entry.loop)
          || entry.loop.some((start, index) => pointSegmentDistance(local, start, entry.loop[(index + 1) % entry.loop.length]) <= GEOMETRY_CLEARANCE_EPS))).length;
      return {
        index: `param-tooth:${metadata.toothOrder}`,
        colliderIndex: closestLoopSegmentIndex(local, loop),
        toothOrder: metadata.toothOrder,
        isTooth: true,
        parametric: true,
        parametricOverlapCount: overlapCount,
      };
    }
    const origin = physics.weaponBody.translation(); const local = rotate(subtract(worldPoint, point(origin.x, origin.y)), -physics.weaponBody.rotation());
    const loop = metadata?.loop || physics.weaponLoop;
    const colliderIndex = closestLoopSegmentIndex(local, loop);
    const sourceIndexes = metadata?.sourceIndexes || physics.weaponSourceIndexes;
    const sourceIndex = sourceIndexes?.[colliderIndex] ?? colliderIndex;
    const toothOrder = toothOrderForBoundaryIndex(sourceIndex);
    let triangleOrder = null; let triangleColliderEdgeIndex = null;
    let triangleEdgeVertices = null; let internalTriangleEdge = false;
    if (metadata?.kind === 'cad-weapon-solid-triangle' && metadata.triangleLoop?.length === 3) {
      triangleOrder = metadata.triangleOrder;
      triangleColliderEdgeIndex = closestLoopSegmentIndex(local, metadata.triangleLoop);
      const a = metadata.triangleVertexIndexes[triangleColliderEdgeIndex];
      const b = metadata.triangleVertexIndexes[(triangleColliderEdgeIndex + 1) % 3];
      const edgeKey = a < b ? `${a}:${b}` : `${b}:${a}`;
      triangleEdgeVertices = [a, b];
      internalTriangleEdge = !metadata.boundaryEdgeKeys.includes(edgeKey);
    }
    return {
      index: sourceIndex, colliderIndex, toothOrder, isTooth: toothOrder !== null, parametric: false,
      triangleOrder, triangleColliderEdgeIndex, triangleEdgeVertices, internalTriangleEdge,
    };
  }

  function collectRigContacts(physics) {
    const contacts = [];
    (physics.targetColliders || [physics.targetCollider]).forEach((target) => physics.world.contactPairsWith(target, (other) => {
      let role = null;
      if (physics.weaponColliderMeta?.has(other.handle) || (physics.weaponColliders || [physics.weaponCollider]).some((collider) => collider.handle === other.handle)) role = 'weapon';
      else if (physics.forkColliderMeta?.has(other.handle) || (physics.forkColliders || [physics.forkCollider]).some((collider) => collider.handle === other.handle)) role = 'fork';
      else if (other.handle === physics.floorCollider.handle) role = 'floor';
      if (!role) return;
      // Side-view XY proximity is not a physical contact when the finite tool
      // and target intervals are disjoint in Z. Collision groups already omit
      // this pair; keep restored/stale manifolds out of telemetry and gates too.
      if (role === 'fork' && effectiveToolWidthZ('fork') <= 1e-9) return;
      physics.world.contactPair(target, other, (manifold, flipped) => {
        const solverCount = manifold.numSolverContacts(); const contactCount = manifold.numContacts();
        if (!solverCount && !contactCount) return;
        const rawNormal = manifold.normal();
        const normal = normalise(point(rawNormal.x * (flipped ? -1 : 1), rawNormal.y * (flipped ? -1 : 1)));
        const combinedSkin = Math.max(0, number(target.contactSkin?.())) + Math.max(0, number(other.contactSkin?.()));
        // Keep every manifold point separate.  Summing several impulses and
        // applying the total back at solverContactPoint(0) changes angular
        // momentum and was the direct source of the former energy explosion.
        const colliderPointToWorld = (collider, localPoint) => {
          const origin = collider.translation(); const rotated = rotate(point(localPoint.x, localPoint.y), collider.rotation());
          return point(origin.x + rotated.x, origin.y + rotated.y);
        };
        const rawContactGeometry = Array.from({ length: contactCount }, (_, rawIndex) => {
          const local1 = manifold.localContactPoint1(rawIndex); const local2 = manifold.localContactPoint2(rawIndex);
          const collider1 = flipped ? other : target; const collider2 = flipped ? target : other;
          const point1 = colliderPointToWorld(collider1, local1); const point2 = colliderPointToWorld(collider2, local2);
          return { point1, point2, midpoint: point((point1.x + point2.x) * .5, (point1.y + point2.y) * .5) };
        });
        const solverPoints = Array.from({ length: solverCount }, (_, solverIndex) => {
          const current = manifold.solverContactPoint(solverIndex);
          return { point: point(current.x, current.y), distance: manifold.solverContactDist(solverIndex) };
        });
        const contactMatchTolerance = GEOMETRY_CLEARANCE_EPS;
        const matchCandidates = [];
        rawContactGeometry.forEach((raw, rawIndex) => solverPoints.forEach((solver, solverIndex) => {
          const distanceError = Math.abs(manifold.contactDist(rawIndex) - solver.distance);
          const pointError = Math.hypot(raw.midpoint.x - solver.point.x, raw.midpoint.y - solver.point.y);
          if (distanceError <= contactMatchTolerance && pointError <= contactMatchTolerance) {
            matchCandidates.push({ rawIndex, solverIndex, score: distanceError + pointError });
          }
        }));
        matchCandidates.sort((left, right) => left.score - right.score || left.rawIndex - right.rawIndex || left.solverIndex - right.solverIndex);
        const rawToSolverContact = new Map(); const usedSolverContacts = new Set();
        matchCandidates.forEach((candidate) => {
          if (rawToSolverContact.has(candidate.rawIndex) || usedSolverContacts.has(candidate.solverIndex)) return;
          rawToSolverContact.set(candidate.rawIndex, candidate.solverIndex); usedSolverContacts.add(candidate.solverIndex);
        });
        const count = contactCount;
        for (let index = 0; index < count; index += 1) {
          const contactIndex = index;
          const mappedSolverIndex = rawToSolverContact.get(contactIndex);
          const hasSolverContact = Number.isInteger(mappedSolverIndex);
          const distance = manifold.contactDist(contactIndex);
          const signedTangent = contactCount && hasSolverContact ? manifold.contactTangentImpulse(contactIndex) : 0;
          const impulse = contactCount && hasSolverContact ? Math.abs(manifold.contactImpulse(contactIndex)) : 0;
          let contactPoint = rawContactGeometry[contactIndex]?.midpoint || point(state.sim.target.pos.x, state.sim.target.pos.y);
          let solverPenetration = 0;
          if (hasSolverContact) {
            contactPoint = solverPoints[mappedSolverIndex].point;
            solverPenetration = Math.max(0, -solverPoints[mappedSolverIndex].distance);
          }
          const feature = role === 'weapon' ? classifyWeaponContact(contactPoint, physics, other) : { index: 0, toothOrder: null, isTooth: false };
          // contactDist is original geometry; solverContactDist may include a
          // predictive constraint.  A positive value is a gap, never bite.
          const geometricGap = Number.isFinite(distance) ? Math.max(0, distance) : Infinity;
          contacts.push({
            role, other, point: contactPoint, normal,
            impulse, tangentImpulse: Math.abs(signedTangent), tangentImpulseSigned: signedTangent,
            penetration: Number.isFinite(distance) ? Math.max(0, -distance) : 0,
            geometricGap, solverPenetration, combinedSkin, manifoldPointIndex: index, hasSolverContact,
            // Earcut diagonals are bookkeeping seams, not physical CAD faces.
            // Keep them in the coupled Rapier solve and exact penetration audit,
            // but never promote their zero/overlap manifolds to a tooth/body
            // strike or an open weapon-contact episode.
            suppressContactEvent: Boolean(feature.internalTriangleEdge) || !hasSolverContact,
            ...feature,
          });
        }
      });
    }));
    return contacts;
  }

  function syncTargetToPhysics(target = state.sim?.target, physics = state.sim?.physics) {
    if (!target || !physics?.targetBody) return;
    const body = physics.targetBody;
    body.setTranslation({ x: target.pos.x, y: target.pos.y }, true);
    body.setRotation(target.angle, true);
    body.setLinvel({ x: target.vel.x, y: target.vel.y }, true);
    body.setAngvel(target.omega, true);
  }

  function syncTargetFromPhysics(target = state.sim?.target, physics = state.sim?.physics) {
    if (!target || !physics?.targetBody) return false;
    const body = physics.targetBody;
    const position = body.translation(); const velocity = body.linvel();
    target.pos = point(position.x, position.y);
    target.vel = point(velocity.x, velocity.y);
    target.angle = body.rotation(); target.omega = body.angvel();
    target.grounded = targetLowestPoint() <= targetSupportY() + FLOOR_SKIN * 2;
    return true;
  }

  // Rapier prevents dynamic tunnelling through its fixed floor. This second
  // guard is intentionally tiny and only aligns the numerical contact with the
  // visible Y=0 datum, so anti-aliasing cannot look like a solid crossed it.
  function enforceTargetFloorGuard() {
    const target = state.sim.target; const floor = targetSupportY(); const lowest = targetLowestPoint();
    if (lowest >= floor + GEOMETRY_CLEARANCE_EPS) return false;
    target.pos.y += floor + GEOMETRY_CLEARANCE_EPS - lowest;
    if (target.vel.y < 0) target.vel.y = 0;
    syncTargetToPhysics(target);
    target.grounded = true;
    return true;
  }

  function stepRapierTarget(dt) {
    const physics = state.sim.physics;
    if (!physics?.world) return false;
    physics.world.integrationParameters.dt = dt;
    physics.world.step();
    syncTargetFromPhysics();
    enforceTargetFloorGuard();
    return true;
  }

  function computeMetrics() {
    const p = state.params;
    const voltage = positive(p.voltage, DEFAULTS.voltage);
    const weaponKv = positive(p.weaponKv, DEFAULTS.weaponKv);
    const weaponRpm = weaponKv * voltage / positive(p.weaponGearRatio, DEFAULTS.weaponGearRatio);
    const angularVelocity = weaponRpm * Math.PI * 2 / 60;
    const cadGeometry = state.drawings.weapon ? weaponCadCollisionGeometry() : null;
    // A closed CAD outline is authoritative for both tooth collision and radius.
    const radius = p.paramWeaponEnabled
      ? activeWeaponRadius()
      : (cadGeometry?.hasClosedOutline ? getMaxWeaponRadius() : positive(number(p.tipRadius) / 1000, .001));
    // The complete rotating mass is the only editable mass-property input.
    // Integrate the active 2D solid about the entered pivot, then scale that
    // radius of gyration by the entered mass. The public kg·mm² field is a
    // read-only compatibility/readout value, never an independent constraint.
    const weaponProperties = automaticWeaponMassProperties();
    const { inertiaInput, inertia, mass: weaponMass } = weaponProperties;
    const rotorMassLowerBound = inertia / Math.max(radius ** 2, 1e-12);
    const maximumInertiaInput = weaponMass * radius ** 2 * 1e6;
    const forkMassInput = positive(p.forkMass, DEFAULTS.forkMass);
    const chassisMassBudget = positive(p.robotMass, .01) - weaponMass - forkMassInput;
    const weaponMassConsistent = Number.isFinite(weaponMass)
      && weaponMass > 0
      && Number.isFinite(inertia) && inertia > 0
      && chassisMassBudget > 0;
    const tipSpeed = angularVelocity * radius;
    // Never substitute a capped render speed for the requested physical state.
    // Runtime convergence is decided from the *live* rotor speed and CAD travel
    // per substep; theoretical full speed may be unreachable within spin-up.
    const simulationAngularVelocity = angularVelocity;
    const simulationTipSpeed = tipSpeed;
    const weaponEnergy = .5 * inertia * angularVelocity ** 2;
    const simulationWeaponEnergy = .5 * inertia * simulationAngularVelocity ** 2;
    const weaponCommandFraction = clamp(number(p.weaponThrottle) / 100, 0, 1);
    const weaponCommandAngularVelocity = angularVelocity * weaponCommandFraction;
    const motorCount = Math.max(1, Math.round(positive(p.weaponMotorCount, 1)));
    // Kv is entered in rpm/V.  Kt = 60 / (2πKv) N·m/A for the ideal SI conversion.
    const kT = 60 / (Math.PI * 2 * weaponKv);
    // `advancedEnabled` is presentation state only.  Hidden inputs retain their
    // entered values and therefore must remain in the same physical model.
    const motorEff = clamp(number(p.weaponMotorEfficiency) / 100, .01, 1);
    const outputFactor = clamp(number(p.weaponOutputFactor) / 100, .01, 1);
    const weaponGearEff = clamp(number(p.weaponEfficiency) / 100, .01, 1);
    const phaseCurrent = Math.max(0, number(p.weaponMotorCurrent));
    const weaponCurrentLimitedTorque = phaseCurrent > 0
      ? motorCount * kT * phaseCurrent * positive(p.weaponGearRatio, 1) * motorEff * weaponGearEff * outputFactor
      : Infinity;
    // Backward-compatible readout/API alias. Infinity intentionally formats as
    // "—" and means "unknown phase-current ceiling", not infinite motor torque:
    // the power-derived torque-speed envelope remains active below.
    const weaponShaftTorque = weaponCurrentLimitedTorque;
    const weaponPowerOut = motorCount * positive(p.weaponMotorPower, 1) * motorEff * weaponGearEff * outputFactor;
    const spinupLowerBound = weaponPowerOut > 0 ? weaponEnergy / weaponPowerOut : Infinity;
    const driveCount = Math.max(1, Math.round(positive(p.driveMotorCount, 1)));
    const wheelDiameter = positive(number(p.wheelDiameter) / 1000, .001);
    const driveWheelRpm = positive(p.driveKv, DEFAULTS.driveKv) * voltage / positive(p.driveGearRatio, DEFAULTS.driveGearRatio);
    const theoreticalVehicleSpeed = driveWheelRpm / 60 * Math.PI * wheelDiameter;
    const driveEff = clamp(number(p.driveEfficiency) / 100, .01, 1);
    const slipFactor = clamp(number(p.slipFactor) / 100, 0, .95);
    const estimatedVehicleSpeed = theoreticalVehicleSpeed * driveEff * (1 - slipFactor);
    const enteredWheelTorque = driveCount * positive(p.driveMotorTorque, .001) * positive(p.driveGearRatio, 1) * driveEff;
    const driveCurrent = Math.max(0, number(p.driveMotorCurrent));
    const driveKt = 60 / (Math.PI * 2 * positive(p.driveKv, DEFAULTS.driveKv));
    const currentLimitedWheelTorque = driveCurrent > 0
      ? driveCount * driveKt * driveCurrent * positive(p.driveGearRatio, 1) * driveEff
      : Infinity;
    // As on the weapon side, 0 A means "unknown phase-current limit".  A
    // positive value is always enforced, whether or not its field is visible.
    const wheelTorque = Math.min(enteredWheelTorque, currentLimitedWheelTorque);
    const motorTraction = wheelTorque / Math.max(wheelDiameter / 2, .001);
    const drivePowerOut = driveCount * positive(p.driveMotorPower, 1) * driveEff;
    const gripTraction = clamp(number(p.gripCoefficient), .01, 2) * positive(p.robotMass, .01) * GRAVITY;
    const driveTraction = Math.min(motorTraction, gripTraction);
    const rollingResistanceForce = Math.max(0, number(p.rollingResistanceCoefficient))
      * positive(p.robotMass, .01) * GRAVITY;
    const driveCommandFraction = clamp(number(p.driveThrottle) / 100, 0, 1);
    const commandVehicleSpeed = estimatedVehicleSpeed * driveCommandFraction;
    // Bite frequency follows teeth found on the imported CAD outer boundary.
    // `toothCount` is retained only as a fallback for an unclosed DXF, never
    // as a request to create evenly-spaced virtual teeth.
    const cadToothCount = activeWeaponToothCount();
    const toothFrequency = weaponRpm / 60 * cadToothCount;
    const biteIdeal = toothFrequency > 0 ? theoreticalVehicleSpeed / toothFrequency * 1000 : null;
    const instantaneousDriveSpeed = state.sim?.driveSpeed ?? estimatedVehicleSpeed;
    const targetState = state.sim?.target;
    const targetContact = state.sim?.currentContact?.point;
    const targetForwardSpeed = targetState
      ? targetState.vel.x - targetState.omega * (targetContact ? targetContact.y - targetState.pos.y : 0)
      : 0;
    const relativeApproachSpeed = Math.max(0, instantaneousDriveSpeed - targetForwardSpeed);
    const liveAngularVelocity = state.sim && Number.isFinite(state.sim.weaponOmega) ? Math.abs(state.sim.weaponOmega) : simulationAngularVelocity;
    // An out-of-domain gear-up retains raw kinematic bite, but never receives
    // a collision calculation at the substituted render speed.
    const liveToothFrequency = liveAngularVelocity / (Math.PI * 2) * cadToothCount;
    const biteInstant = liveToothFrequency > 0 ? relativeApproachSpeed / liveToothFrequency * 1000 : null;
    return { voltage, weaponRpm, angularVelocity, simulationAngularVelocity, weaponCommandFraction, weaponCommandAngularVelocity, radius, inertia, inertiaInput, weaponMass, weaponMassPropertySource: weaponProperties.source, weaponMassShapeArea: weaponProperties.area, weaponMassShapeCentroid: weaponProperties.centroid, weaponRadiusOfGyration: Math.sqrt(weaponProperties.radiusOfGyrationSquared), rotorMassLowerBound, maximumInertiaInput, forkMassInput, chassisMassBudget, weaponMassConsistent, tipSpeed, simulationTipSpeed, weaponEnergy, simulationWeaponEnergy, kT, phaseCurrent, weaponCurrentLimitedTorque, weaponShaftTorque, weaponPowerOut, spinupLowerBound, driveWheelRpm, theoreticalVehicleSpeed, estimatedVehicleSpeed, driveCommandFraction, commandVehicleSpeed, instantaneousDriveSpeed, relativeApproachSpeed, motorTraction, gripTraction, driveTraction, drivePowerOut, driveCurrent, currentLimitedWheelTorque, rollingResistanceForce, toothFrequency, liveToothFrequency, biteIdeal, biteInstant, cadToothCount };
  }

  function availableDriveTraction(speed = state.sim?.driveSpeed ?? state.metrics.estimatedVehicleSpeed) {
    const powerLimited = state.metrics.drivePowerOut / Math.max(Math.abs(speed), .10);
    return Math.min(state.metrics.driveTraction, powerLimited);
  }

 function targetLength() { return positive(number(state.params.targetLength) / 1000, .001); }
 function targetThickness() { return positive(number(state.params.targetThickness) / 1000, .001); }
 function targetWidthZ() { return positive(number(state.params.targetWidthZ) / 1000, .001); }
  function targetGeometryMass() { return targetMaterialProperties().density * targetLength() * targetThickness() * targetWidthZ(); }
  function effectiveTargetMass() { return state.params.targetMassMode === 'geometry' ? Math.max(.0001, targetGeometryMass()) : positive(state.params.targetMass, .01); }
  function synchroniseAutomaticTargetMass() {
    if (state.params.targetMassMode !== 'geometry') return false;
    const computed = Math.max(.0001, targetGeometryMass());
    // Keep enough significant digits for small weight classes without exposing
    // floating-point noise in the editable engineering field.
    const rounded = Number(computed.toPrecision(7));
    if (state.params.targetMass === rounded) return false;
    state.params.targetMass = rounded;
    return true;
  }
 function targetInertia() { const m = effectiveTargetMass(); return m * (targetLength() ** 2 + targetThickness() ** 2) / 12; }
 function initialTargetMaterialGeometry() {
   const halfLength = targetLength() / 2; const halfThickness = targetThickness() / 2;
   return [[[[-halfLength, -halfThickness], [halfLength, -halfThickness], [halfLength, halfThickness], [-halfLength, halfThickness], [-halfLength, -halfThickness]]]];
 }
 function cloneMaterialGeometry(geometry) {
   return geometry?.map((polygon) => polygon.map((ring) => ring.map(([x, y]) => [x, y]))) || [];
 }
 function materialGeometryMoments(geometry) {
   let area = 0; let firstX = 0; let firstY = 0; let polarAtOrigin = 0;
   (geometry || []).forEach((polygon) => polygon.forEach((ring) => {
     for (let index = 0; index + 1 < ring.length; index += 1) {
       const [x0, y0] = ring[index]; const [x1, y1] = ring[index + 1];
       const crossValue = x0 * y1 - x1 * y0;
       area += crossValue / 2;
       firstX += (x0 + x1) * crossValue / 6;
       firstY += (y0 + y1) * crossValue / 6;
       polarAtOrigin += crossValue * (
         x0 ** 2 + x0 * x1 + x1 ** 2 + y0 ** 2 + y0 * y1 + y1 ** 2
       ) / 12;
     }
   }));
   if (area < 0) return { area: -area, firstX: -firstX, firstY: -firstY, polarAtOrigin: -polarAtOrigin };
   return { area, firstX, firstY, polarAtOrigin };
 }
  function materialGeometryArea(geometry) { return Math.max(0, materialGeometryMoments(geometry).area); }
  function materialBoundarySegments(geometry) {
    const segments = [];
    (geometry || []).forEach((polygon) => polygon.forEach((ring) => {
      for (let index = 0; index + 1 < ring.length; index += 1) {
        const start = point(ring[index][0], ring[index][1]);
        const end = point(ring[index + 1][0], ring[index + 1][1]);
        if (length(subtract(end, start)) > 1e-14) segments.push({ start, end });
      }
    }));
    return segments;
  }
  function pointSegmentDistance(sample, start, end) {
    const edge = subtract(end, start); const squared = dot(edge, edge);
    if (squared <= 1e-24) return length(subtract(sample, start));
    const fraction = clamp(dot(subtract(sample, start), edge) / squared, 0, 1);
    return length(subtract(sample, add(start, scalePoint(edge, fraction))));
  }

  function pointInsideSimpleLoop(sample, loop) {
    let inside = false;
    for (let index = 0, previous = loop.length - 1; index < loop.length; previous = index, index += 1) {
      const a = loop[index]; const b = loop[previous];
      if ((a.y > sample.y) !== (b.y > sample.y)
        && sample.x < (b.x - a.x) * (sample.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside;
  }

  function closedLoopsOverlap(leftLoop, rightLoop, epsilon = GEOMETRY_CLEARANCE_EPS) {
    const orientation = (a, b, c) => cross(subtract(b, a), subtract(c, a));
    const onSegment = (sample, start, end) => pointSegmentDistance(sample, start, end) <= epsilon;
    const segmentsIntersect = (a, b, c, d) => {
      const abC = orientation(a, b, c); const abD = orientation(a, b, d);
      const cdA = orientation(c, d, a); const cdB = orientation(c, d, b);
      if (abC * abD < -(epsilon ** 2) && cdA * cdB < -(epsilon ** 2)) return true;
      return (Math.abs(abC) <= epsilon && onSegment(c, a, b))
        || (Math.abs(abD) <= epsilon && onSegment(d, a, b))
        || (Math.abs(cdA) <= epsilon && onSegment(a, c, d))
        || (Math.abs(cdB) <= epsilon && onSegment(b, c, d));
    };
    if (leftLoop.some((candidate) => pointInsideSimpleLoop(candidate, rightLoop))
      || rightLoop.some((candidate) => pointInsideSimpleLoop(candidate, leftLoop))) return true;
    return leftLoop.some((start, leftIndex) => rightLoop.some((otherStart, rightIndex) => segmentsIntersect(
      start,
      leftLoop[(leftIndex + 1) % leftLoop.length],
      otherStart,
      rightLoop[(rightIndex + 1) % rightLoop.length],
    )));
  }

  function exactCadLoopTargetPenetration(localLoop, movingBody, targetBody) {
    if (!localLoop?.length || !movingBody || !targetBody) return 0;
    const movingPositionRaw = movingBody.translation(); const targetPositionRaw = targetBody.translation();
    const movingPosition = point(movingPositionRaw.x, movingPositionRaw.y);
    const targetPosition = point(targetPositionRaw.x, targetPositionRaw.y);
    const movingAngle = movingBody.rotation(); const targetAngle = targetBody.rotation();
    const loop = localLoop.map((local) => rotate(
      subtract(add(movingPosition, rotate(local, movingAngle)), targetPosition),
      -targetAngle,
    ));
    const halfX = targetLength() / 2; const halfY = targetThickness() / 2;
    let deepest = 0;
    for (let index = 0; index < loop.length; index += 1) {
      const a = loop[index]; const b = loop[(index + 1) % loop.length];
      const dx = b.x - a.x; const dy = b.y - a.y;
      let enter = 0; let leave = 1;
      const clipAxis = (origin, delta, lower, upper) => {
        if (Math.abs(delta) <= 1e-16) return origin >= lower && origin <= upper;
        let first = (lower - origin) / delta; let last = (upper - origin) / delta;
        if (first > last) [first, last] = [last, first];
        enter = Math.max(enter, first); leave = Math.min(leave, last);
        return enter <= leave;
      };
      if (!clipAxis(a.x, dx, -halfX, halfX) || !clipAxis(a.y, dy, -halfY, halfY)) continue;
      enter = clamp(enter, 0, 1); leave = clamp(leave, 0, 1);
      if (leave < enter) continue;
      // On a clipped straight segment, distance to each rectangle face is
      // affine. The maximum of their minimum occurs at an endpoint or where
      // two of those affine functions cross, so this is exact for the sampled
      // CAD polyline rather than a visual/raster estimate.
      const distances = [
        [halfX - a.x, -dx], [halfX + a.x, dx],
        [halfY - a.y, -dy], [halfY + a.y, dy],
      ];
      const candidates = [enter, leave];
      for (let left = 0; left < distances.length; left += 1) {
        for (let right = left + 1; right < distances.length; right += 1) {
          const denominator = distances[left][1] - distances[right][1];
          if (Math.abs(denominator) <= 1e-16) continue;
          const fraction = (distances[right][0] - distances[left][0]) / denominator;
          if (fraction >= enter && fraction <= leave) candidates.push(fraction);
        }
      }
      candidates.forEach((fraction) => {
        deepest = Math.max(deepest, Math.min(...distances.map(([constant, slope]) => constant + slope * fraction)));
      });
    }
    // Also cover the inverse containment case where a target corner lies inside
    // the CAD solid without any CAD edge entering the target rectangle.
    [point(-halfX, -halfY), point(halfX, -halfY), point(halfX, halfY), point(-halfX, halfY)]
      .filter((corner) => pointInsideSimpleLoop(corner, loop))
      .forEach((corner) => {
        const distance = loop.reduce((minimum, start, index) => Math.min(
          minimum,
          pointSegmentDistance(corner, start, loop[(index + 1) % loop.length]),
        ), Infinity);
        if (Number.isFinite(distance)) deepest = Math.max(deepest, distance);
      });
    return Math.max(0, deepest);
  }
  function exactWeaponLoopsTargetPenetration(physics) {
    const loops = physics?.weaponLoops?.length ? physics.weaponLoops : [physics?.weaponLoop];
    return loops.reduce((deepest, loop) => Math.max(
      deepest,
      exactCadLoopTargetPenetration(loop, physics.weaponBody, physics.targetBody),
    ), 0);
  }
  function exactLoopFloorPenetration(localLoop, movingBody, floor = groundY()) {
    if (!localLoop?.length || !movingBody) return 0;
    const rawPosition = movingBody.translation(); const position = point(rawPosition.x, rawPosition.y); const angle = movingBody.rotation();
    const lowest = Math.min(...localLoop.map((local) => add(position, rotate(local, angle)).y));
    return Math.max(0, floor - lowest);
  }
  function materialNewBoundaryLength(previousGeometry, nextGeometry) {
    // Gc is energy per newly-created crack area, not per ballistic tool path.
    // Polygon clipping splits inherited edges at intersections, so classifying
    // several interior samples of every next-boundary segment cleanly separates
    // old free surfaces from newly-created material interfaces.  The tolerance
    // is numerical (0.1 um), well below the accepted CAD contact tolerance.
    const previous = materialBoundarySegments(previousGeometry);
    const tolerance = 1e-7;
    return materialBoundarySegments(nextGeometry).reduce((sum, segment) => {
      const inherited = [.2, .5, .8].every((fraction) => {
        const sample = lerp(segment.start, segment.end, fraction);
        return previous.some((old) => pointSegmentDistance(sample, old.start, old.end) <= tolerance);
      });
      return sum + (inherited ? 0 : length(subtract(segment.end, segment.start)));
    }, 0);
  }
 function damageRowSegments(damage, row) {
   if (damage?.segments?.[row]) return damage.segments[row];
   const halfLength = targetLength() / 2;
   const left = -halfLength + clamp(damage?.depths?.[row] || 0, 0, targetLength());
   const right = halfLength - clamp(damage?.rightDepths?.[row] || 0, 0, targetLength());
   return right - left > 1e-10 ? [[left, right]] : [];
 }

 function targetRemainingLocalRects(damage = state.sim?.materialDamage) {
   const halfThickness = targetThickness() / 2;
   if (!damage?.rows) return [{ x0: -targetLength() / 2, x1: targetLength() / 2, y0: -halfThickness, y1: halfThickness }];
   const rectangles = [];
   for (let row = 0; row < damage.rows; row += 1) {
     const y0 = -halfThickness + row * damage.rowHeight; const y1 = y0 + damage.rowHeight;
     damageRowSegments(damage, row).forEach(([x0, x1]) => {
       if (x1 - x0 > 1e-10) rectangles.push({ x0, x1, y0, y1 });
     });
   }
   return rectangles;
 }

  function targetDamageMassProperties(damage = state.sim?.materialDamage) {
   const lengthX = targetLength(); const thicknessY = targetThickness();
   const initialMass = effectiveTargetMass(); const initialInertia = targetInertia();
   if (!damage?.depths?.length) {
     return { initialMass, remainingMass: initialMass, removedMass: 0, center: point(0, 0), inertia: initialInertia };
   }
   // The side-view notch only removes the Z slice actually overlapped by the
   // weapon.  The former area-ratio rule silently removed the same notch across
   // the target's entire Z width (and even from fixture/ballast mass), which made
    // a 6 g chip erase roughly 173 g from the rigid body in the default case.
    const arealDensity = targetMaterialProperties().density * effectiveToolWidthZ('weapon');
    let sliceMass = 0; let sliceFirstX = 0; let sliceFirstY = 0; let sliceInertiaAtOrigin = 0;
    if (Array.isArray(damage.geometry)) {
      const moments = materialGeometryMoments(damage.geometry);
      sliceMass = arealDensity * moments.area;
      sliceFirstX = arealDensity * moments.firstX;
      sliceFirstY = arealDensity * moments.firstY;
      sliceInertiaAtOrigin = arealDensity * moments.polarAtOrigin;
    } else {
      targetRemainingLocalRects(damage).forEach(({ x0, x1, y0, y1 }) => {
        const width = x1 - x0; const height = y1 - y0;
        sliceMass += arealDensity * width * height;
        sliceFirstX += arealDensity * height * (x1 ** 2 - x0 ** 2) / 2;
        sliceFirstY += arealDensity * width * (y1 ** 2 - y0 ** 2) / 2;
        sliceInertiaAtOrigin += arealDensity * (
          height * (x1 ** 3 - x0 ** 3) / 3
          + width * (y1 ** 3 - y0 ** 3) / 3
        );
      });
    }
   const fullSliceMass = arealDensity * lengthX * thicknessY;
   const fullSliceInertia = fullSliceMass * (lengthX ** 2 + thicknessY ** 2) / 12;
   const removedMass = Math.max(0, fullSliceMass - sliceMass);
   const removedInertiaAtOrigin = Math.max(0, fullSliceInertia - sliceInertiaAtOrigin);
   const remainingMass = initialMass - removedMass;
   if (!(remainingMass > 1e-7)) return { initialMass, remainingMass, removedMass, invalid: true };
   // The original full slice has zero first moment, therefore the remaining
   // target's first moment equals that of the still-present material slice.
   const center = point(sliceFirstX / remainingMass, sliceFirstY / remainingMass);
   const inertiaAtOrigin = initialInertia - removedInertiaAtOrigin;
    const inertia = inertiaAtOrigin - remainingMass * (center.x ** 2 + center.y ** 2);
    if (!Number.isFinite(inertia) || !(inertia > 0)) {
      return { initialMass, remainingMass, removedMass, center, inertia, invalid: true };
    }
    return {
      initialMass, remainingMass, removedMass, center,
      inertia,
    };
 }
 function createMaterialDamageState() {
   const rows = clamp(Math.ceil(targetThickness() / .00025), 32, 256);
   return {
     rows,
     rowHeight: targetThickness() / rows,
      depths: Array(rows).fill(0),
      rightDepths: Array(rows).fill(0),
      segments: Array.from({ length: rows }, () => [[-targetLength() / 2, targetLength() / 2]]),
      geometry: initialTargetMaterialGeometry(),
     pending: Array(rows).fill(0),
     rightPending: Array(rows).fill(0),
     removedArea: 0,
     removedMass: 0,
      dirty: false,
      version: 0,
      cuts: [],
      activeTooth: null,
      activeStartTime: null,
      lastCutTime: null,
      activeAngularTravel: 0,
   };
 }
 function cloneMaterialDamage(damage) {
   if (!damage) return null;
   return {
     ...damage,
     depths: [...damage.depths],
     rightDepths: [...(damage.rightDepths || Array(damage.rows).fill(0))],
      segments: (damage.segments || Array.from({ length: damage.rows }, (_, row) => damageRowSegments(damage, row)))
        .map((segments) => segments.map((segment) => [...segment])),
      geometry: cloneMaterialGeometry(damage.geometry),
     pending: [...damage.pending],
     rightPending: [...(damage.rightPending || Array(damage.rows).fill(0))],
     cuts: damage.cuts.map((cut) => ({ ...cut })),
   };
 }
  function targetRemainingLocalPolygon(damage = state.sim?.materialDamage) {
    const halfLength = targetLength() / 2; const halfThickness = targetThickness() / 2;
    if (!damage?.depths?.length) return [point(-halfLength, -halfThickness), point(halfLength, -halfThickness), point(halfLength, halfThickness), point(-halfLength, halfThickness)];
    // An orthogonal staircase represents each damage row exactly.  The old
    // diagonal interpolation made bottom-row removal half its booked area and
    // top-row removal 1.5×, so collider mass and the drawn notch disagreed.
    const rightDepths = damage.rightDepths || Array(damage.rows).fill(0);
    const polygon = [];
    for (let row = 0; row < damage.rows; row += 1) {
      const bottom = -halfThickness + row * damage.rowHeight;
      const top = bottom + damage.rowHeight;
      const right = halfLength - clamp(rightDepths[row] || 0, 0, targetLength());
      polygon.push(point(right, bottom), point(right, top));
    }
    for (let row = damage.rows - 1; row >= 0; row -= 1) {
      const top = -halfThickness + (row + 1) * damage.rowHeight;
      const bottom = -halfThickness + row * damage.rowHeight;
      const depth = clamp(damage.depths[row] || 0, 0, targetLength());
      const x = -halfLength + depth;
      polygon.push(point(x, top), point(x, bottom));
    }
    return removeDuplicateLoopPoints(polygon).points;
 }
 function targetRemainingWorldPolygon() { return targetRemainingLocalPolygon().map(targetToWorld); }
  function targetRemainingLocalGeometry(damage = state.sim?.materialDamage) {
    // An empty MultiPolygon is a valid terminal state, not "missing data".
    // Falling back to the initial rectangle here resurrected a fully removed
    // target in the renderer and in the rebuilt Rapier collider.
    return Array.isArray(damage?.geometry) ? damage.geometry : initialTargetMaterialGeometry();
  }
 function targetRemainingWorldGeometry() {
   return targetRemainingLocalGeometry().map((polygon) => polygon.map((ring) => ring.map(([x, y]) => targetToWorld(point(x, y)))));
 }
 function targetRemainingWorldRects() {
   return targetRemainingLocalRects().map(({ x0, x1, y0, y1 }) => [
     point(x0, y0), point(x1, y0), point(x1, y1), point(x0, y1),
   ].map(targetToWorld));
 }
 // The arena floor is a fixed global datum so every test uses scene Y = 0.
  function groundY() { return 0; }
  function rigJointErrorTolerance() { return clamp(activeWeaponRadius() * .005, RIG_JOINT_ERROR_MIN, RIG_JOINT_ERROR_MAX); }
  function rigForkJointErrorTolerance() {
    const radius = Math.max(.001, ...activeForkBodyPoints().map(length));
    return clamp(radius * .005, RIG_JOINT_ERROR_MIN, RIG_JOINT_ERROR_MAX);
  }
 function targetSupportY() { return groundY() + Math.max(0, number(state.params.targetClearance) / 1000); }
 function targetRestY() { return targetSupportY() + targetThickness() / 2; }
  // The clearance represents a persistent equivalent support envelope from
  // floor asperities / imperfect seating, not an unsupported initial height.
  function targetInitialY() { return targetRestY(); }

  function allToothTips() {
    const omega = signedOmega();
    if (state.params.paramWeaponEnabled) {
      return parameterWeaponGeometry().segments.map((segment) => {
        const relative = rotate(segment.tip, state.sim.angle);
        return {
          index: `param-tooth:${segment.toothOrder}`,
          order: segment.toothOrder,
          relative,
          point: add(weaponSceneOrigin(), relative),
          tangent: point(-omega * relative.y, omega * relative.x),
          source: segment.tip,
          parametric: true,
        };
      });
    }
    const geometry = weaponCadCollisionGeometry();
    return geometry.teeth.map((feature, order) => {
      const local = weaponLocal(geometry.sourceOutline[feature.index]);
      const relative = rotate(local, state.sim.angle);
      return { index: `cad-tooth:${feature.index}`, order, relative, point: add(weaponSceneOrigin(), relative), tangent: point(-omega * relative.y, omega * relative.x), source: geometry.sourceOutline[feature.index] };
    });
  }
  function initialWeaponOmega() {
    if (state.params.weaponStartMode !== 'setSpeed') return 0;
    return (Math.sign(nominalSignedOmega()) || 1) * state.metrics.weaponCommandAngularVelocity;
  }

  function configuredInitialDriveSpeed() {
    return state.params.driveStartMode === 'setSpeed' ? state.metrics.commandVehicleSpeed : 0;
  }

 function createSimulation() {
    const initialGap = Math.max(0, number(state.params.targetClearance) / 1000);
    const target = { pos: point(number(state.params.targetSceneX) / 1000, targetInitialY()), vel: point(0, 0), angle: 0, omega: 0, grounded: true };
    const initialAngle = ((number(state.params.weaponInitialAngle) % 360) + 360) % 360 * Math.PI / 180;
    const initialOmega = initialWeaponOmega();
    const rigPhysics = createRapierRig(target, initialAngle, initialOmega);
    const initialForkAngle = rigPhysics?.initialForkAngle ?? 0;
    // When Rapier is present, a failed unified CAD rig is an invalid scenario,
    // not permission to silently switch to a different collision model.
    const rigCreationFailed = Boolean(USE_RAPIER_RIG && !rigPhysics);
    const physics = rigPhysics;
   return {
      running: false,
      time: 0,
      angle: initialAngle,
      forkOrigin: initialForkPivotAt(weaponBaseSceneOrigin()),
      forkVelocity: point(configuredInitialDriveSpeed(), 0),
      forkAngle: initialForkAngle,
      forkOmega: 0,
      forkGrounded: Boolean(rigPhysics?.initialForkTip),
      forkContact: false,
      forkEngaged: false,
      forkSupportLift: 0,
      targetPushedByFork: false,
      // A user-specified gap is an equivalent rough-floor support envelope.
      // The target starts supported and only becomes airborne after separation.
      targetLaunched: false,
      robotTravel: 0,
      // The chassis starts from rest. Rapier integrates traction-limited motor
      // force, so every bite uses actual instantaneous speed instead of a
      // preloaded no-load velocity.
      driveSpeed: configuredInitialDriveSpeed(),
      driveSlip: 0,
      weaponOmega: initialOmega,
      lastWeaponAngleStep: 0,
      previousWeaponOrigin: weaponBaseSceneOrigin(),
      target,
      physics,
      // CCD needs the target pose at the *same* previous simulation instant as
      // previousTips.  Re-expressing a prior tooth point in the current target
      // frame creates false crossings whenever a struck target translates or spins.
      previousTargetPose: snapshotTargetPose(target),
      previousTips: new Map(),
      activeContactIds: new Set(),
      toothCooldowns: new Map(),
      lastImpactTime: -Infinity,
      lastBlockingTime: -Infinity,
      // A non-tooth CAD body contact is a unilateral rigid constraint. Stop
      // the prescribed drive/rotor rather than inventing a small impulse.
      weaponBlocked: false,
      modelDomainStopped: false,
      // `failureDomain` is the authoritative terminal classification.  The two
      // legacy booleans remain for compatibility, but an internal exception is
      // deliberately neither a rigid-solver nor a material-model conclusion.
      failureDomain: rigCreationFailed ? 'internal' : null,
      currentContact: null,
      lastImpact: null,
      hitCount: 0,
      bodyImpactCount: 0,
      lastEventHit: 0,
      lastEventBodyImpact: 0,
      lastBodyImpactTime: -Infinity,
      blockingAnnounced: false,
      rigActiveContacts: new Set(),
      rigWeaponEpisode: null,
      rigMinFloorClearance: initialGap,
      rigForkMinFloorClearance: groundClearanceStatus().shovelClearance,
      rigMaxPenetration: { floor: 0, forkFloor: 0, fork: 0, weapon: 0 },
      rigRejectedPenetration: { floor: 0, forkFloor: 0, fork: 0, weapon: 0 },
      rigJointError: 0,
      rigForkJointError: 0,
      rigMaxJointError: 0,
      rigMaxForkJointError: 0,
      rigSubstepsLastTick: 0,
      trail: [],
      materialDamage: createMaterialDamageState(),
      materialStats: { removedArea: 0, removedVolume: 0, removedMass: 0, work: 0, deformationWork: 0 },
      materialMaxIntrusion: 0,
      lastRigFailure: null,
      creationError: rigCreationFailed ? (state.rapierError || 'Rapier CAD 刚体创建失败') : null,
      solverDomainStopped: rigCreationFailed,
      completed: rigCreationFailed,
    };
  }

  function snapshotTargetPose(target = state.sim.target) {
    return { pos: point(target.pos.x, target.pos.y), angle: number(target.angle), vel: point(target.vel.x, target.vel.y), omega: number(target.omega) };
  }
  function interpolateTargetPose(previous, current, fraction) {
    return {
      pos: lerp(previous.pos, current.pos, fraction),
      angle: previous.angle + (current.angle - previous.angle) * fraction,
      vel: lerp(previous.vel, current.vel, fraction),
      omega: previous.omega + (current.omega - previous.omega) * fraction,
    };
  }
  function targetToWorldAt(local, pose) { return add(pose.pos, rotate(local, pose.angle)); }
  function worldToTargetAt(world, pose) { return rotate(subtract(world, pose.pos), -pose.angle); }
  function targetPointVelocityAt(world, pose) { return add(pose.vel, scalePoint(perpendicular(subtract(world, pose.pos)), pose.omega)); }
  function targetToWorld(local) { const target = state.sim.target; return add(target.pos, rotate(local, target.angle)); }
  function worldToTarget(world) { const target = state.sim.target; return rotate(subtract(world, target.pos), -target.angle); }
  function targetCorners() { const l = targetLength() / 2; const t = targetThickness() / 2; return [point(-l, -t), point(l, -t), point(l, t), point(-l, t)].map(targetToWorld); }
  function targetPointVelocity(world) { const target = state.sim.target; return add(target.vel, scalePoint(perpendicular(subtract(world, target.pos)), target.omega)); }
  function targetHasUncutZBacking() {
    return state.params.contactModel === 'material'
      && effectiveToolWidthZ('weapon') < targetWidthZ() - GEOMETRY_CLEARANCE_EPS;
  }
  function targetLowestPoint() {
    const remaining = state.params.contactModel === 'material'
      && !targetHasUncutZBacking()
      && state.sim?.materialDamage?.geometry?.length
      ? targetRemainingWorldGeometry().flat(2)
      : targetCorners();
    return Math.min(...remaining.map((p) => p.y));
  }
  function targetLowestContact() {
    return targetCorners().reduce((lowest, candidate) => candidate.y < lowest.y ? candidate : lowest);
  }

  function segmentAabbIntersection(start, end, minX, maxX, minY, maxY) {
    const d = subtract(end, start);
    let enter = 0; let exit = 1; let enterNormal = null;
    const axes = [
      { p: start.x, d: d.x, min: minX, max: maxX, minNormal: point(-1, 0), maxNormal: point(1, 0) },
      { p: start.y, d: d.y, min: minY, max: maxY, minNormal: point(0, -1), maxNormal: point(0, 1) },
    ];
    for (const axis of axes) {
      if (Math.abs(axis.d) < 1e-10) { if (axis.p < axis.min || axis.p > axis.max) return null; continue; }
      let t1 = (axis.min - axis.p) / axis.d; let t2 = (axis.max - axis.p) / axis.d;
      let n1 = axis.minNormal; let n2 = axis.maxNormal;
      if (t1 > t2) { [t1, t2] = [t2, t1]; [n1, n2] = [n2, n1]; }
      if (t1 > enter) { enter = t1; enterNormal = n1; }
      exit = Math.min(exit, t2);
      if (enter > exit) return null;
    }
    const inside = start.x >= minX && start.x <= maxX && start.y >= minY && start.y <= maxY;
    if (inside) {
      const distances = [start.x - minX, maxX - start.x, start.y - minY, maxY - start.y];
      const min = Math.min(...distances); const index = distances.indexOf(min);
      enterNormal = [point(-1, 0), point(1, 0), point(0, -1), point(0, 1)][index];
      enter = 0;
    }
    return enterNormal ? { t: clamp(enter, 0, 1), normal: enterNormal, inside } : null;
  }

  function currentPenetration(local, normal) {
    const l = targetLength() / 2; const t = targetThickness() / 2;
    if (Math.abs(normal.x) > .5) return clamp(normal.x < 0 ? local.x + l : l - local.x, 0, targetLength());
    return clamp(normal.y < 0 ? local.y + t : t - local.y, 0, targetThickness());
  }

  function detectWeaponContacts() {
    const sim = state.sim; const l = targetLength() / 2; const t = targetThickness() / 2;
    const samples = cadBlockingSamples(); const contacts = [];
    if (!samples.length) return contacts;
    const currentOrigin = weaponSceneOrigin(); const previousOrigin = sim.previousWeaponOrigin || currentOrigin;
    const currentTargetPose = snapshotTargetPose(sim.target);
    const previousTargetPose = sim.previousTargetPose || currentTargetPose;
    const angleStep = Number.isFinite(sim.lastWeaponAngleStep) ? sim.lastWeaponAngleStep : signedOmega() * FIXED_DT;
    const distanceToOriginSweep = (position) => {
      const delta = subtract(currentOrigin, previousOrigin); const denominator = dot(delta, delta);
      const fraction = denominator > 1e-12 ? clamp(dot(subtract(position, previousOrigin), delta) / denominator, 0, 1) : 0;
      return length(subtract(position, lerp(previousOrigin, currentOrigin, fraction)));
    };
    const broadReach = activeWeaponRadius() + Math.hypot(l, t) + .003;
    if (Math.min(distanceToOriginSweep(previousTargetPose.pos), distanceToOriginSweep(currentTargetPose.pos)) > broadReach) {
      sim.previousTips = new Map(samples.map((sample) => [sample.index, add(currentOrigin, rotate(sample.local, sim.angle))]));
      sim.previousWeaponOrigin = currentOrigin;
      return contacts;
    }
    // A 0.5 ms physics step can rotate a fast weapon dozens of degrees.  Split
    // its circular sweep into <= 5° chords so the contact test does not wait
    // for a lucky coarse-step crossing or cut straight through a target.
    const arcSteps = Math.max(1, Math.ceil(Math.abs(angleStep) / (Math.PI / 72)));
    // Sweep the target at matching physical times. Re-expressing a prior tooth
    // point in the final target frame is what creates ghost impacts after flight.
    const targetAngleSteps = Math.max(1, Math.ceil(Math.abs(currentTargetPose.angle - previousTargetPose.angle) / (Math.PI / 72)));
    const targetTranslationSteps = Math.max(1, Math.ceil(length(subtract(currentTargetPose.pos, previousTargetPose.pos)) / CAD_COLLISION_CHORD));
    const sweepDistance = activeWeaponRadius() * Math.abs(angleStep) + length(subtract(currentOrigin, previousOrigin));
    const maxChord = clamp(Math.min(targetThickness() * .5, .0015), .00035, .0015);
    // Never cap a required CCD subdivision and silently advance through it.
    // Extremely dense sweeps are handled as an explicit solver-domain stop
    // by the TOI loop rather than becoming untested motion.
    const spatialSteps = Math.max(1, Math.ceil(sweepDistance / maxChord));
    const sweepSteps = Math.max(arcSteps, targetAngleSteps, targetTranslationSteps, spatialSteps);
    const omega = signedOmega();
    const startAngle = sim.angle - angleStep;
    samples.forEach((sample) => {
      let previous = sim.previousTips.get(sample.index) || add(previousOrigin, rotate(sample.local, startAngle));
      let previousFraction = 0;
      for (let step = 1; step <= sweepSteps; step += 1) {
        const fraction = step / sweepSteps;
        const theta = startAngle + angleStep * fraction;
        const relative = rotate(sample.local, theta);
        const next = add(lerp(previousOrigin, currentOrigin, fraction), relative);
        const targetPoseStart = interpolateTargetPose(previousTargetPose, currentTargetPose, previousFraction);
        const targetPoseEnd = interpolateTargetPose(previousTargetPose, currentTargetPose, fraction);
        // Transform each endpoint using the target pose from that endpoint's
        // instant. The resulting segment is the real relative tooth sweep.
        const startLocal = worldToTargetAt(previous, targetPoseStart); const endLocal = worldToTargetAt(next, targetPoseEnd);
        const hit = segmentAabbIntersection(startLocal, endLocal, -l, l, -t, t);
        if (hit) {
          const contactLocal = lerp(startLocal, endLocal, hit.t);
          const hitFraction = (step - 1 + hit.t) / sweepSteps;
          const targetPoseAtHit = interpolateTargetPose(previousTargetPose, currentTargetPose, hitFraction);
          const contact = targetToWorldAt(contactLocal, targetPoseAtHit);
          const normal = rotate(hit.normal, targetPoseAtHit.angle);
          const hitTheta = startAngle + angleStep * hitFraction;
          const hitRelative = rotate(sample.local, hitTheta);
          const tangent = point(-omega * hitRelative.y, omega * hitRelative.x);
          const weaponVelocity = add(tangent, point(number(sim.driveSpeed), 0));
          const targetVelocity = targetPointVelocityAt(contact, targetPoseAtHit);
          const relativeVelocity = subtract(weaponVelocity, targetVelocity);
          const normalVelocity = dot(relativeVelocity, normal);
          // Require an inward crossing of the visible CAD boundary. A point
          // already inside is a t=0 constraint recovery, not an ignored pass.
          if (normalVelocity < -0.002) {
            // The impact point is exactly on the target boundary, so its
            // mathematical penetration is zero. For a useful bite readout we
            // report the inward distance covered by this real CAD edge over the
            // remainder of this CCD chord. It is geometric sweep only, not a
            // prediction of material fracture depth.
            const sweepBite = clamp(-dot(subtract(endLocal, contactLocal), hit.normal), 0, Math.max(targetLength(), targetThickness()));
            contacts.push({ index: sample.index, source: sample.source, sampleLocal: sample.local, isImpactSample: Boolean(sample.isImpactSample), previous, relative: hitRelative, tangent, point: contact, local: contactLocal, normal, normalLocal: hit.normal, endLocal, relativeVelocity, normalVelocity, penetration: sweepBite, inside: false, sweepFraction: hitFraction });
            break;
          }
        }
        previous = next; previousFraction = fraction;
      }
    });
    sim.previousTips = new Map(samples.map((sample) => [sample.index, add(currentOrigin, rotate(sample.local, sim.angle))]));
    sim.previousWeaponOrigin = currentOrigin;
    // Adjacent samples around one physical tooth may cross the target during
    // the same fixed step.  Only the earliest CAD-boundary crossing is one hit.
    return contacts.sort((a, b) => a.sweepFraction - b.sweepFraction).slice(0, 1);
  }

  function targetKineticEnergy(target = state.sim.target) {
    return .5 * effectiveTargetMass() * dot(target.vel, target.vel) + .5 * targetInertia() * target.omega ** 2;
  }
  function targetEnergyAfterImpulse(contact, impulse) {
    const target = state.sim.target; const mass = effectiveTargetMass(); const inertia = Math.max(targetInertia(), 1e-9); const r = subtract(contact, target.pos);
    const velocity = add(target.vel, scalePoint(impulse, 1 / mass)); const omega = target.omega + cross(r, impulse) / inertia;
    return .5 * mass * dot(velocity, velocity) + .5 * inertia * omega ** 2;
  }
  function capImpulseByTargetEnergy(contact, rawImpulse, energyBudget) {
    const before = targetKineticEnergy(); const rawEnergy = Math.max(0, targetEnergyAfterImpulse(contact, rawImpulse) - before);
    if (rawEnergy <= energyBudget || rawEnergy <= 1e-9) return { impulse: rawImpulse, transferredEnergy: rawEnergy, scale: 1 };
    let low = 0; let high = 1;
    for (let i = 0; i < 18; i += 1) {
      const middle = (low + high) / 2; const energy = Math.max(0, targetEnergyAfterImpulse(contact, scalePoint(rawImpulse, middle)) - before);
      if (energy > energyBudget) high = middle; else low = middle;
    }
    const impulse = scalePoint(rawImpulse, low);
    return { impulse, transferredEnergy: Math.max(0, targetEnergyAfterImpulse(contact, impulse) - before), scale: low };
  }
  function rotorEnergyAfterImpulse(contact, impulse) {
    const inertia = Math.max(state.metrics.inertia, 1e-9); const beforeOmega = signedOmega();
    const afterOmega = beforeOmega - cross(subtract(contact, weaponSceneOrigin()), impulse) / inertia;
    const direction = Math.sign(nominalSignedOmega()) || 1;
    const nonReversingOmega = Math.max(0, direction * afterOmega) * direction;
    return .5 * inertia * nonReversingOmega ** 2;
  }
  function capImpulseByEnergyLedger(contact, rawImpulse, targetBudget, rotorBudget = targetBudget * 1.5) {
    const targetBefore = targetKineticEnergy(); const rotorBefore = .5 * state.metrics.inertia * signedOmega() ** 2;
    const candidate = (scale) => {
      const impulse = scalePoint(rawImpulse, scale);
      return {
        impulse,
        targetGain: Math.max(0, targetEnergyAfterImpulse(contact, impulse) - targetBefore),
        rotorLoss: Math.max(0, rotorBefore - rotorEnergyAfterImpulse(contact, impulse)),
      };
    };
    const full = candidate(1);
    if (full.targetGain <= targetBudget && full.rotorLoss <= rotorBudget) return { ...full, scale: 1 };
    let low = 0; let high = 1;
    for (let index = 0; index < 20; index += 1) {
      const middle = (low + high) / 2; const value = candidate(middle);
      if (value.targetGain <= targetBudget && value.rotorLoss <= rotorBudget) low = middle; else high = middle;
    }
    return { ...candidate(low), scale: low };
  }
  function usableCadToothDepth() {
    const geometry = weaponCadCollisionGeometry();
    const radii = geometry.sourceOutline.map((source) => length(weaponLocal(source))).filter(Number.isFinite);
    if (!radii.length || !geometry.teeth.length) return Math.max(.001, state.metrics.radius * .08);
    const sorted = [...radii].sort((a, b) => a - b);
    const baseRadius = sorted[Math.floor(sorted.length * .55)] || sorted[0];
    const toothRadius = Math.max(...geometry.teeth.map((tooth) => tooth.radius));
    return clamp(toothRadius - baseRadius, .001, Math.max(.001, state.metrics.radius * .45));
  }
  function contactEngagement(contact) {
    const toothFrequency = Math.max(0, state.metrics.liveToothFrequency);
    const targetVelocity = targetPointVelocity(contact.point);
    const forwardRelative = subtract(point(state.sim.driveSpeed, 0), targetVelocity);
    // -normal points from the target face into its material. Only robot feed
    // in that direction can create a bite; tangential blade speed alone is a
    // scrape and must not unlock the full stored rotor energy.
    const feedSpeed = Math.max(0, dot(forwardRelative, scalePoint(contact.normal, -1)));
    const bite = toothFrequency > 1e-9 ? feedSpeed / toothFrequency : 0;
    const toothDepth = usableCadToothDepth();
    const ccdEntry = Math.max(0, contact.penetration || 0);
    const engagedDepth = Math.min(ccdEntry, bite + CCD_POSITION_SLOP);
    const ratio = clamp(engagedDepth / toothDepth, 0, 1);
    // Squared engagement is intentionally conservative at a shallow scrape.
    // It is a contact-compliance proxy, not a material cutting model.
    return { toothFrequency, feedSpeed, bite, toothDepth, ccdEntry, engagedDepth, factor: ratio * ratio };
  }
  function estimateImpact(contact) {
    const rWeapon = subtract(contact.point, weaponSceneOrigin());
    const rTarget = subtract(contact.point, state.sim.target.pos);
    // The side-view chassis has one explicit translational DOF: horizontal
    // travel. Vertical reaction is supplied by the unseen wheels/floor, so
    // only the X component contributes to an impact effective mass.
    const invRobotX = 1 / positive(state.params.robotMass, .01);
    const invForDirection = (direction) => invRobotX * direction.x ** 2
      + (cross(rWeapon, direction) ** 2) / Math.max(state.metrics.inertia, 1e-9)
      + 1 / effectiveTargetMass()
      + (cross(rTarget, direction) ** 2) / Math.max(targetInertia(), 1e-9);
    const restitution = clamp(number(state.params.restitution), 0, .25);
    const vn = contact.normalVelocity;
    // `normal` points out of the target face toward the incoming tooth.  The
    // target must receive the opposite impulse; using the outward normal here
    // used to push it back into the fork and inject implausible motion.
    const invMassNormal = invForDirection(contact.normal);
    const normalMagnitude = vn < 0 ? -(1 + restitution) * vn / invMassNormal : 0;
    const tangent = perpendicular(contact.normal);
    const vt = dot(contact.relativeVelocity, tangent);
    const invMassTangent = invForDirection(tangent);
    const friction = weaponTargetFriction();
    const rawTangentialImpulse = clamp(vt / invMassTangent, -friction * normalMagnitude, friction * normalMagnitude);
    const rawImpulse = add(scalePoint(contact.normal, -normalMagnitude), scalePoint(tangent, rawTangentialImpulse));
    const effectiveMass = 1 / Math.max(invMassNormal, 1e-9);
    const collisionEnergy = .5 * effectiveMass * vn ** 2;
    const rotorEnergy = .5 * state.metrics.inertia * signedOmega() ** 2;
    const coupling = clamp(number(state.params.impactEfficiency) / 100, 0, .30);
    const engagement = contactEngagement(contact);
    const energyBudget = Math.min(rotorEnergy * coupling, collisionEnergy) * engagement.factor;
    const capped = capImpulseByTargetEnergy(contact.point, rawImpulse, energyBudget);
    const normalImpulse = normalMagnitude * capped.scale; const tangentialImpulse = rawTangentialImpulse * capped.scale;
    return { contact: contact.point, normal: contact.normal, penetration: contact.penetration, relativeSpeed: length(contact.relativeVelocity), normalVelocity: vn, normalImpulse, tangentialImpulse, friction, impulse: length(capped.impulse), impulseVector: capped.impulse, effectiveMass, invMassNormal, invMassTangent, collisionEnergy, energyBudget, coupling, engagement, transferredEnergy: capped.transferredEnergy, time: state.sim.time };
  }

  function reduceWeaponSpeedFromImpact(impact) {
    const sim = state.sim; const inertia = Math.max(state.metrics.inertia, 1e-9); const beforeEnergy = .5 * inertia * sim.weaponOmega ** 2;
    // The target-energy cap has already scaled J.  Apply the rotor's angular
    // reaction once; subtracting target energy a second time incorrectly
    // slowed the weapon even for an impulse through its axis.
    const impulseOmega = sim.weaponOmega - cross(subtract(impact.contact, weaponSceneOrigin()), impact.impulseVector) / inertia;
    const direction = Math.sign(nominalSignedOmega()) || 1;
    sim.weaponOmega = Math.max(0, direction * impulseOmega) * direction;
    impact.rotorEnergyBefore = beforeEnergy; impact.rotorEnergyAfter = .5 * inertia * sim.weaponOmega ** 2; impact.rotorEnergyLoss = Math.max(0, beforeEnergy - impact.rotorEnergyAfter);
  }

  function projectCadOutOfTarget() {
    const target = state.sim.target; const halfLength = targetLength() / 2; const halfThickness = targetThickness() / 2;
    let separated = false;
    // Several passes catch a broad solid blade whose neighbours were already
    // inside the target when the first point reached its time of impact.
    for (let pass = 0; pass < 4; pass += 1) {
      let deepest = null;
      cadBlockingSamples().forEach((sample) => {
        const bladePoint = add(weaponSceneOrigin(), rotate(sample.local, state.sim.angle));
        const local = worldToTarget(bladePoint);
        if (local.x < -halfLength || local.x > halfLength || local.y < -halfThickness || local.y > halfThickness) return;
        const faces = [
          { depth: local.x + halfLength, normal: point(-1, 0) },
          { depth: halfLength - local.x, normal: point(1, 0) },
          { depth: local.y + halfThickness, normal: point(0, -1) },
          { depth: halfThickness - local.y, normal: point(0, 1) },
        ];
        const face = faces.reduce((best, candidate) => candidate.depth < best.depth ? candidate : best);
        if (!deepest || face.depth > deepest.depth) deepest = { face, bladePoint };
      });
      if (!deepest) break;
      const worldNormal = rotate(deepest.face.normal, target.angle);
      target.pos = add(target.pos, scalePoint(worldNormal, -(deepest.face.depth + CCD_POSITION_SLOP)));
      // If the target is travelling back into the blocking CAD edge, remove
      // only that inward component. No artificial launch impulse is created.
      const intoBlade = dot(target.vel, worldNormal);
      if (intoBlade > 0) target.vel = subtract(target.vel, scalePoint(worldNormal, intoBlade));
      syncTargetToPhysics(target);
      separated = true;
    }
    if (separated) enforceTargetFloorGuard();
    return separated;
  }

  function applyBlockingResponse(contact) {
    const sim = state.sim;
    if (contact.normalVelocity >= -0.002) return false;
    // We know the outer DXF body touched, but not its out-of-plane thickness,
    // local compliance, bearing flex or material failure law. Applying an
   // arbitrary "small" energy here would be tuning, not mechanics. Treat it
    // as an unresolved unilateral constraint. Freeze this test at the exact
    // DXF boundary; do not delete rotor energy or invent a contact impulse.
   const rotorEnergyBefore = .5 * state.metrics.inertia * sim.weaponOmega ** 2;
    const isFirstBodyImpact = !sim.blockingAnnounced;
    if (isFirstBodyImpact) {
      sim.bodyImpactCount += 1;
      sim.lastBodyImpactTime = sim.time;
    }
    const bodyImpactNumber = Math.max(1, sim.bodyImpactCount);
   sim.weaponBlocked = true;
    sim.running = false;
    sim.completed = true;
   sim.lastBlockingTime = sim.time;
    const impact = {
      contact: contact.point,
      normal: contact.normal,
      penetration: 0,
      relativeSpeed: length(contact.relativeVelocity),
      normalVelocity: contact.normalVelocity,
      normalImpulse: 0,
      tangentialImpulse: 0,
      friction: 0,
      impulse: 0,
      impulseVector: point(0, 0),
      collisionEnergy: 0,
      energyBudget: 0,
      coupling: 0,
      impactKind: 'body',
      bodyContact: true,
      toothOrder: null,
      sourceIndex: contact.index,
      bodyImpactNumber,
      blocking: true,
      unresolvedContact: true,
      targetEnergyBefore: targetKineticEnergy(),
      targetEnergyAfter: targetKineticEnergy(),
      targetEnergyGain: 0,
     transferredEnergy: 0,
     rotorEnergyBefore,
      rotorEnergyAfter: rotorEnergyBefore,
      rotorEnergyLoss: 0,
     time: sim.time,
    };
    sim.lastImpact = impact;
    if (isFirstBodyImpact) {
      addEvent(`刀体 / 背板碰撞 #${bodyImpactNumber}：source ${contact.index ?? '未知'}，法向冲量 0 N·s；非刀齿 DXF 外轮廓先接触，缺少该处厚度、柔度和材料失效模型，仿真已在边界处暂停；未计入牙齿命中或 bite。`, 'warning');
      updateStatus('非刀齿接触待建模', 'warning');
      sim.lastEventBodyImpact = bodyImpactNumber;
      sim.blockingAnnounced = true;
    }
    return impact;
  }

  function triggerImpact(contact) {
    if (!contact.isImpactSample) return false;
    const impact = estimateImpact(contact);
    if (impact.normalVelocity >= -0.002 || impact.normalImpulse <= 0) return false;
    const sim = state.sim;
    impact.targetEnergyBefore = targetKineticEnergy();
    applyImpulseToTarget(impact.contact, impact.impulseVector);
    projectCadOutOfTarget();
    impact.targetEnergyAfter = targetKineticEnergy();
    impact.targetEnergyGain = Math.max(0, impact.targetEnergyAfter - impact.targetEnergyBefore);
    // Store the measured state transition, rather than only the pre-impact
    // prediction used by the limiter. This leaves an auditable energy ledger
    // for every simplified collision in the UI/event log.
    impact.transferredEnergy = impact.targetEnergyGain;
    reduceWeaponSpeedFromImpact(impact);
    const reactionX = impact.impulseVector.x;
    // Equal/opposite horizontal impulse is represented by the chassis travel
    // DOF. A strong strike may briefly reverse the chassis; motor traction
    // then recovers it through updateDrive instead of deleting momentum.
    sim.driveSpeed -= reactionX / positive(state.params.robotMass, .01);
    const tractionImpulse = availableDriveTraction(sim.driveSpeed) * .008;
    sim.driveSlip = Math.max(sim.driveSlip, clamp((Math.abs(reactionX) - tractionImpulse) / Math.max(Math.abs(reactionX), 1e-6), 0, 1));
    sim.targetLaunched = true;
    sim.hitCount += 1;
    impact.impactKind = 'tooth';
    impact.bodyContact = false;
    impact.toothOrder = contact.toothOrder;
    impact.sourceIndex = contact.index;
    impact.toothHitNumber = sim.hitCount;
    sim.toothCooldowns.set(contact.index, sim.time);
    sim.lastImpactTime = sim.time;
    sim.lastImpact = impact;
    sim.currentContact = { ...contact, impact };
    if (sim.hitCount <= 3 || sim.hitCount - sim.lastEventHit >= 20) {
      addEvent(`牙齿命中 #${sim.hitCount}：source ${contact.index ?? '未知'}，法向冲量 ${format(impact.normalImpulse, 6)} N·s；靶动能 +${format(impact.targetEnergyGain)} J，转子损失 ${format(impact.rotorEnergyLoss)} J。`, 'impact');
      sim.lastEventHit = sim.hitCount;
    }
    return true;
  }

  function applyImpulseToTarget(contact, impulse) {
    const target = state.sim.target; const mass = effectiveTargetMass(); const r = subtract(contact, target.pos);
    const body = state.sim.physics?.targetBody;
    if (body) {
      syncTargetToPhysics(target);
      body.applyImpulseAtPoint({ x: impulse.x, y: impulse.y }, { x: contact.x, y: contact.y }, true);
      syncTargetFromPhysics();
      return;
    }
    target.vel = add(target.vel, scalePoint(impulse, 1 / mass));
    target.omega += cross(r, impulse) / Math.max(targetInertia(), 1e-9);
  }

  function resolveFloorContact(dt) {
    const target = state.sim.target; const floor = targetSupportY(); const normal = point(0, 1); const tangent = point(1, 0); let touching = false;
    for (let iteration = 0; iteration < 3; iteration += 1) {
      const initialContact = targetLowestContact(); const penetration = floor - initialContact.y;
      if (penetration < -FLOOR_SKIN) break;
      touching = true;
      if (penetration > 0) target.pos.y += penetration + FLOOR_SKIN;
      const correctedContact = targetLowestContact(); const r = subtract(correctedContact, target.pos); const contactVelocity = targetPointVelocity(correctedContact);
      const invMassNormal = 1 / effectiveTargetMass() + (cross(r, normal) ** 2) / Math.max(targetInertia(), 1e-9);
      const floorRestitution = getFloorMaterial().restitution;
      const bounceThreshold = .35;
      let normalImpulse = 0;
      if (contactVelocity.y < -bounceThreshold) normalImpulse = -(1 + floorRestitution) * contactVelocity.y / invMassNormal;
      else if (contactVelocity.y < 0) normalImpulse = -contactVelocity.y / invMassNormal;
      if (normalImpulse > 0) applyImpulseToTarget(correctedContact, scalePoint(normal, normalImpulse));
      const floorFriction = targetFloorFriction(); const afterNormalVelocity = targetPointVelocity(correctedContact); const tangentSpeed = dot(afterNormalVelocity, tangent);
      const invMassTangent = 1 / effectiveTargetMass() + (cross(r, tangent) ** 2) / Math.max(targetInertia(), 1e-9);
      const requiredStaticImpulse = -tangentSpeed / invMassTangent;
      // Three positional iterations make tunnelling less likely; share the
      // support friction between them so the floor does not become 3× stickier.
      const supportImpulse = Math.max(normalImpulse, effectiveTargetMass() * GRAVITY * dt) / 3;
      const staticLimit = floorFriction.static * supportImpulse;
      const tangentialImpulse = Math.abs(requiredStaticImpulse) <= staticLimit ? requiredStaticImpulse : -Math.sign(tangentSpeed || 1) * floorFriction.kinetic * supportImpulse;
      if (Math.abs(tangentialImpulse) > 1e-10) applyImpulseToTarget(correctedContact, scalePoint(tangent, tangentialImpulse));
    }
    if (touching) {
      // Do not erase a small *upward* fork-lift velocity merely because the
      // target's lowest corner is still inside the floor skin.  That old dead
      // zone made a fork hold the target for hundreds of milliseconds before
      // a tooth could reach it.
      if (target.vel.y < 0 && Math.abs(target.vel.y) < .025) target.vel.y = 0;
      if (Math.abs(target.vel.x) < .004) target.vel.x = 0;
      if (Math.abs(target.omega) < .04) target.omega = 0;
    }
    target.grounded = touching && target.vel.y <= .025;
  }

 function updateFreeTarget(dt) {
   const target = state.sim.target;
   if (stepRapierTarget(dt)) {
      // Do not silently clamp angular velocity: that would delete angular
      // momentum/energy. Rapier owns free-flight dynamics and CCD here.
     if (state.sim.trail.length === 0 || state.sim.time - state.sim.trail[state.sim.trail.length - 1].time > .016) state.sim.trail.push({ ...target.pos, time: state.sim.time });
     if (state.sim.trail.length > 100) state.sim.trail.shift();
     return;
   }
   target.vel.y -= GRAVITY * dt;
   target.pos = add(target.pos, scalePoint(target.vel, dt));
   target.angle += target.omega * dt;
    resolveFloorContact(dt);
    if (state.sim.trail.length === 0 || state.sim.time - state.sim.trail[state.sim.trail.length - 1].time > .016) state.sim.trail.push({ ...target.pos, time: state.sim.time });
    if (state.sim.trail.length > 100) state.sim.trail.shift();
  }

  function recoverWeaponSpeed(dt) {
    const sim = state.sim;
    if (sim.weaponBlocked) return;
    const inertia = Math.max(state.metrics.inertia, 1e-9);
    const direction = Math.sign(nominalSignedOmega()) || 1;
    const commanded = direction * state.metrics.weaponCommandAngularVelocity;
    const torque = weaponMotorTorqueAt(sim.weaponOmega);
    if (!torque) return;
    const next = sim.weaponOmega + torque / inertia * dt;
    sim.weaponOmega = direction > 0 ? Math.min(next, commanded) : Math.max(next, commanded);
  }
  function updateDrive(dt) {
    const sim = state.sim;
    if (sim.weaponBlocked) { sim.driveSpeed = 0; return; }
    const desired = state.metrics.commandVehicleSpeed; const acceleration = availableDriveTraction(sim.driveSpeed) / positive(state.params.robotMass, .01);
    sim.driveSpeed = Math.min(desired, sim.driveSpeed + acceleration * dt * Math.max(.15, 1 - sim.driveSlip));
    sim.driveSlip = Math.max(0, sim.driveSlip - dt * 4.5);
  }
  // Contact candidates come only from the forward CAD-edge strip.  The old
  // code accidentally used `bounds.width` before defining it, which caused the
  // fallback to use every fork sample (including high/rear geometry) as a fake
  // collision point.  Work in target-local space so a rotated target cannot be
  // hit merely because its world AABB overlaps the fork.
  function getGroundedForkContactCandidate() {
    const leading = getShovelLeadingEdgeAt(weaponSceneOrigin());
    const halfLength = targetLength() / 2; const halfThickness = targetThickness() / 2;
    const candidates = leading.points.map((forkPoint) => {
      const local = worldToTarget(forkPoint);
      return { forkPoint, local, depth: local.x + halfLength, verticalOutside: Math.abs(local.y) - halfThickness };
    }).filter((candidate) => candidate.depth >= -FORK_CONTACT_SKIN
      && candidate.depth <= targetLength() + FORK_CONTACT_SKIN
      && candidate.verticalOutside <= FORK_VERTICAL_CONTACT_SKIN);
    if (!candidates.length) return null;
    candidates.sort((a, b) => b.depth - a.depth || a.forkPoint.y - b.forkPoint.y);
    const candidate = candidates[0];
    const contactLocal = point(-halfLength, clamp(candidate.local.y, -halfThickness, halfThickness));
    return {
      ...candidate,
      contact: targetToWorld(contactLocal),
      normal: normalise(rotate(point(-1, 0), state.sim.target.angle)),
    };
  }

  // While it is merely being driven by the fork, the target is a grounded
  // slider.  This deliberately removes the ballistic degree of freedom: the
  // floor carries the vertical load and pitch moment, and only a real weapon
  // strike may switch the target into the free-flight solver.
  function updateGroundedTarget(dt) {
    const sim = state.sim; const target = sim.target; const floorFriction = targetFloorFriction();
    const supportY = targetRestY() + clamp(sim.forkSupportLift || 0, 0, MAX_FORK_STATIC_LIFT);
    const speed = target.vel.x; const drag = floorFriction.kinetic * GRAVITY * dt;
    target.vel.x = Math.abs(speed) <= drag ? 0 : speed - Math.sign(speed) * drag;
    target.vel.y = 0; target.omega = 0; target.angle = 0;
    target.pos.x += target.vel.x * dt; target.pos.y = supportY; target.grounded = true;
    if (sim.trail.length === 0 || sim.time - sim.trail[sim.trail.length - 1].time > .016) sim.trail.push({ ...target.pos, time: sim.time });
    if (sim.trail.length > 100) sim.trail.shift();
  }

  function applyForkContact(dt) {
    const sim = state.sim; const target = sim.target; let candidate = getGroundedForkContactCandidate();
    if (!candidate) return false;

    // Correct the kinematic robot position before applying force.  This keeps
    // the actual fork nose on the target's left face instead of allowing a
    // sampled point to tunnel through the full rectangle.
    const correction = Math.max(0, candidate.depth - CCD_POSITION_SLOP);
    if (correction > 0) {
      sim.robotTravel = Math.max(0, sim.robotTravel - correction);
      candidate = getGroundedForkContactCandidate();
      if (!candidate) return false;
    }

    sim.forkEngaged = true; sim.targetPushedByFork = true;
    if (!sim.forkContact) {
      sim.forkContact = true;
      addEvent('叉子真实前缘接触靶子：地面承担竖向载荷，仅模拟受限水平推送/轻微几何支撑。', 'fork');
    }

    // A fork may statically support the target by its real nose height, but
    // only if that nose is immediately below the target underside.  This is a
    // geometric lift of at most 2.5 mm, never an injected upward velocity.
    const halfThickness = targetThickness() / 2;
    const nearUnderside = candidate.local.y <= -halfThickness + .002;
    if (nearUnderside) {
      // `targetRestY` already includes the user-selected floor clearance, so
      // support must replace (not add to) that floor height.  Otherwise a
      // 0.1 mm clearance would turn a 0.2 mm fork nose into a 0.3 mm gap and
      // make contact flicker every other simulation step.
      const supportedCenterY = candidate.forkPoint.y + halfThickness;
      sim.forkSupportLift = clamp(supportedCenterY - targetRestY(), 0, MAX_FORK_STATIC_LIFT);
    }

    // The force path is horizontal.  Applying it at the target centre models
    // the ground's opposing pitch reaction, preventing a low fork force from
    // spuriously spinning or catapulting the target.
    const mass = effectiveTargetMass(); const robotMass = positive(state.params.robotMass, .01);
    const relativeSpeed = sim.driveSpeed - target.vel.x;
    if (relativeSpeed <= 1e-5) return true;
    const floorFriction = targetFloorFriction(); const normalLoad = mass * GRAVITY;
    const staticLimit = floorFriction.static * normalLoad;
    const interfaceFactor = clamp(.45 + shovelTargetFriction() * .45, .45, .95);
    const driveForce = availableDriveTraction(sim.driveSpeed) * interfaceFactor;

    // Static floor friction can hold a stationary target.  The robot remains
    // geometrically blocked and no target impulse is manufactured.
    if (Math.abs(target.vel.x) < .002 && driveForce <= staticLimit) {
      sim.driveSpeed = 0;
      sim.driveSlip = Math.max(sim.driveSlip, .7);
      return true;
    }

    const requestedImpulse = driveForce * dt;
    const closingImpulse = relativeSpeed / (1 / robotMass + 1 / mass);
    const impulse = Math.max(0, Math.min(requestedImpulse, closingImpulse));
    if (impulse <= 1e-10) return true;
    target.vel.x += impulse / mass;
    sim.driveSpeed = Math.max(0, sim.driveSpeed - impulse / robotMass);
    sim.driveSlip = Math.max(sim.driveSlip, clamp(impulse / Math.max(requestedImpulse, 1e-9), 0, 1) * .25);
    return true;
  }
  function resolveFreeForkOverlap() {
    const candidate = getGroundedForkContactCandidate();
    if (!candidate || candidate.depth <= 0) return false;
    const target = state.sim.target;
    // The fork may guide/support a free target, but it must never be allowed
    // to pass through it. This is a non-energetic positional constraint: the
    // fork does not manufacture an upward catapult impulse.
    target.pos = add(target.pos, scalePoint(candidate.normal, -(candidate.depth + CCD_POSITION_SLOP)));
    const intoFork = dot(target.vel, candidate.normal);
    if (intoFork > 0) target.vel = subtract(target.vel, scalePoint(candidate.normal, intoFork));
    syncTargetToPhysics(target); enforceTargetFloorGuard();
    state.sim.forkEngaged = true;
    return true;
  }

 function snapshotMotionState() {
   const sim = state.sim;
   return {
      // Grounded target prediction does not step Rapier. Airborne prediction
      // does, so retain an exact engine snapshot for a possible TOI rollback.
      physicsSnapshot: sim.targetLaunched ? snapshotTargetPhysics(sim.physics) : null,
     angle: sim.angle, robotTravel: sim.robotTravel, driveSpeed: sim.driveSpeed, driveSlip: sim.driveSlip, weaponOmega: sim.weaponOmega,
      lastWeaponAngleStep: sim.lastWeaponAngleStep, forkContact: sim.forkContact, forkEngaged: sim.forkEngaged,
      forkSupportLift: sim.forkSupportLift, targetPushedByFork: sim.targetPushedByFork, targetLaunched: sim.targetLaunched,
      target: { ...snapshotTargetPose(sim.target), grounded: sim.target.grounded },
      trail: sim.trail.map((entry) => ({ ...entry })),
      previousTips: new Map(sim.previousTips), previousWeaponOrigin: point(sim.previousWeaponOrigin.x, sim.previousWeaponOrigin.y),
      previousTargetPose: snapshotTargetPose(sim.previousTargetPose), activeContactIds: new Set(sim.activeContactIds),
      hitCount: sim.hitCount,
      bodyImpactCount: sim.bodyImpactCount,
      lastEventHit: sim.lastEventHit,
      lastEventBodyImpact: sim.lastEventBodyImpact,
      lastImpactTime: sim.lastImpactTime,
      lastBodyImpactTime: sim.lastBodyImpactTime,
      currentContact: clonePlaybackPlain(sim.currentContact),
      lastImpact: clonePlaybackPlain(sim.lastImpact),
      rigWeaponEpisode: cloneRigEpisode(sim.rigWeaponEpisode),
      blockingAnnounced: sim.blockingAnnounced,
      eventCount: activeEventEntries().length,
    };
  }
 function restoreMotionState(snapshot) {
   const sim = state.sim;
    const restoredPhysics = snapshot.physicsSnapshot ? restoreTargetPhysics(snapshot.physicsSnapshot) : false;
   sim.angle = snapshot.angle; sim.robotTravel = snapshot.robotTravel; sim.driveSpeed = snapshot.driveSpeed; sim.driveSlip = snapshot.driveSlip; sim.weaponOmega = snapshot.weaponOmega;
    sim.lastWeaponAngleStep = snapshot.lastWeaponAngleStep; sim.forkContact = snapshot.forkContact; sim.forkEngaged = snapshot.forkEngaged;
    sim.forkSupportLift = snapshot.forkSupportLift; sim.targetPushedByFork = snapshot.targetPushedByFork; sim.targetLaunched = snapshot.targetLaunched;
    sim.target.pos = point(snapshot.target.pos.x, snapshot.target.pos.y); sim.target.vel = point(snapshot.target.vel.x, snapshot.target.vel.y);
    sim.target.angle = snapshot.target.angle; sim.target.omega = snapshot.target.omega; sim.target.grounded = snapshot.target.grounded;
    sim.trail = snapshot.trail.map((entry) => ({ ...entry })); sim.previousTips = new Map(snapshot.previousTips);
    sim.previousWeaponOrigin = point(snapshot.previousWeaponOrigin.x, snapshot.previousWeaponOrigin.y); sim.previousTargetPose = snapshotTargetPose(snapshot.previousTargetPose); sim.activeContactIds = new Set(snapshot.activeContactIds);
    sim.hitCount = snapshot.hitCount;
    sim.bodyImpactCount = snapshot.bodyImpactCount;
    sim.lastEventHit = snapshot.lastEventHit;
    sim.lastEventBodyImpact = snapshot.lastEventBodyImpact;
    sim.lastImpactTime = snapshot.lastImpactTime;
    sim.lastBodyImpactTime = snapshot.lastBodyImpactTime;
    sim.currentContact = clonePlaybackPlain(snapshot.currentContact);
    sim.lastImpact = clonePlaybackPlain(snapshot.lastImpact);
    sim.rigWeaponEpisode = cloneRigEpisode(snapshot.rigWeaponEpisode);
    sim.blockingAnnounced = snapshot.blockingAnnounced;
    const events = activeEventEntries();
    events.length = snapshot.eventCount;
    // When no complete engine snapshot was required, align the target body
    // manually. When it was restored, this is idempotent and keeps the JS
    // mirror exactly in sync with the restored Rapier state.
    if (!restoredPhysics && sim.targetLaunched && sim.physics?.world) {
      state.rapierError = state.rapierError || 'Rapier CCD 快照恢复失败';
    }
    syncTargetToPhysics(sim.target);
 }
  function advancePhysicalSegment(dt) {
    const sim = state.sim;
    updateDrive(dt);
    sim.robotTravel += sim.driveSpeed * dt;
    sim.forkEngaged = false; sim.forkSupportLift = 0;
    if (sim.targetLaunched) { updateFreeTarget(dt); resolveFreeForkOverlap(); }
    else {
      const forkPushing = applyForkContact(dt);
      if (forkPushing || sim.targetPushedByFork) updateGroundedTarget(dt);
      else {
        sim.target.pos.x = number(state.params.targetSceneX) / 1000; sim.target.pos.y = targetRestY();
        sim.target.vel = point(0, 0); sim.target.angle = 0; sim.target.omega = 0; sim.target.grounded = true;
      }
    }
    recoverWeaponSpeed(dt);
    sim.lastWeaponAngleStep = signedOmega() * dt;
    sim.angle += sim.lastWeaponAngleStep;
    if (Math.abs(sim.angle) > Math.PI * 8) sim.angle %= Math.PI * 2;
  }
  function materialiseWeaponContact(contact) {
    const sim = state.sim; const sampleLocal = contact.sampleLocal || weaponLocal(contact.source);
    const origin = weaponSceneOrigin(); const relative = rotate(sampleLocal, sim.angle); const bladePoint = add(origin, relative);
    const halfLength = targetLength() / 2; const halfThickness = targetThickness() / 2;
    const local = worldToTarget(bladePoint); const normalLocal = contact.normalLocal || rotate(contact.normal, -sim.target.angle);
    if (Math.abs(normalLocal.x) > .5) local.x = normalLocal.x < 0 ? -halfLength : halfLength;
    else local.y = normalLocal.y < 0 ? -halfThickness : halfThickness;
    const contactPoint = targetToWorld(local); const normal = normalise(rotate(normalLocal, sim.target.angle));
    const omega = signedOmega(); const tangent = point(-omega * relative.y, omega * relative.x);
    const weaponVelocity = add(tangent, point(number(sim.driveSpeed), 0)); const targetVelocity = targetPointVelocity(contactPoint);
    const relativeVelocity = subtract(weaponVelocity, targetVelocity);
    return { ...contact, sampleLocal, relative, point: contactPoint, local, normal, normalLocal, tangent, relativeVelocity, normalVelocity: dot(relativeVelocity, normal) };
  }
  function recordCcdHistory() {
    const sim = state.sim; const origin = weaponSceneOrigin();
    sim.previousTips = new Map(cadBlockingSamples().map((sample) => [sample.index, add(origin, rotate(sample.local, sim.angle))]));
    sim.previousWeaponOrigin = origin; sim.previousTargetPose = snapshotTargetPose(sim.target);
  }
  function finishPhysicsTick(contact, active) {
    const sim = state.sim;
    recordCcdHistory(); sim.activeContactIds = active; sim.currentContact = contact || null;
    if (sim.currentContact && !sim.currentContact.impact && sim.currentContact.isImpactSample) sim.currentContact.impact = estimateImpact(sim.currentContact);
    sim.time += FIXED_DT;
    const endTime = simulationEndTime();
    if (sim.time >= endTime - FIXED_DT * 1e-6) {
      sim.running = false; sim.completed = true;
      addEvent(`到达设定计算时长 ${endTime.toFixed(4)} s，自动暂停。`, 'done'); updateStatus('计算区间完成', 'paused');
    }
  }

  function rigEnergySnapshot() {
    const sim = state.sim; const physics = sim.physics;
    const robotVelocity = physics.robotBody.linvel(); const weaponVelocity = physics.weaponBody.linvel();
    const forkVelocity = physics.forkBody.linvel(); const forkPosition = physics.forkBody.translation();
    const robotMass = number(physics.robotBody.mass?.()) || positive(state.params.robotMass, .01);
    const weaponMass = number(physics.weaponBody.mass?.());
    const targetVelocity = physics.targetBody.linvel(); const targetPosition = physics.targetBody.translation();
    const targetComRaw = physics.targetBody.worldCom?.() || targetPosition;
    const weaponComRaw = physics.weaponBody.worldCom?.() || physics.weaponBody.translation();
    const forkComRaw = physics.forkBody.worldCom?.() || forkPosition;
    const targetMass = Math.max(number(physics.targetBody.mass?.()), 1e-9);
    const targetRotationalInertia = Math.max(number(physics.targetBody.principalInertia?.()), 1e-12);
    const targetKinetic = .5 * targetMass * (targetVelocity.x ** 2 + targetVelocity.y ** 2)
      + .5 * targetRotationalInertia * physics.targetBody.angvel() ** 2;
    const targetPotential = targetMass * GRAVITY * targetComRaw.y;
    const forkMass = Math.max(number(physics.forkBody.mass?.()), 1e-9);
    const forkRotationalInertia = Math.max(number(physics.forkBody.principalInertia?.()), 1e-12);
    const forkKinetic = .5 * forkMass * (forkVelocity.x ** 2 + forkVelocity.y ** 2)
      + .5 * forkRotationalInertia * physics.forkBody.angvel() ** 2;
    const forkPotential = forkMass * GRAVITY * forkComRaw.y;
    return {
      target: targetKinetic + targetPotential,
      targetKinetic,
      targetPotential,
      targetVx: physics.targetBody.linvel().x,
      targetVelocity: point(targetVelocity.x, targetVelocity.y),
      targetPosition: point(targetPosition.x, targetPosition.y),
      targetCom: point(targetComRaw.x, targetComRaw.y),
      targetAngle: physics.targetBody.rotation(),
      targetMass,
      targetRotationalInertia,
      targetOmega: physics.targetBody.angvel(),
      robotSpeed: robotVelocity.x,
      robotVelocity: point(robotVelocity.x, robotVelocity.y),
      weaponPosition: point(physics.weaponBody.translation().x, physics.weaponBody.translation().y),
      weaponCom: point(weaponComRaw.x, weaponComRaw.y),
      weaponAngle: physics.weaponBody.rotation(),
      weaponOmega: physics.weaponBody.angvel(),
      forkPosition: point(forkPosition.x, forkPosition.y),
      forkCom: point(forkComRaw.x, forkComRaw.y),
      forkVelocity: point(forkVelocity.x, forkVelocity.y),
      forkAngle: physics.forkBody.rotation(),
      forkOmega: physics.forkBody.angvel(),
      forkMass,
      forkRotationalInertia,
      fork: forkKinetic + forkPotential,
      forkKinetic,
      forkPotential,
      rotor: .5 * state.metrics.inertia * physics.weaponBody.angvel() ** 2,
      chassis: .5 * robotMass * (robotVelocity.x ** 2 + robotVelocity.y ** 2)
        + .5 * weaponMass * (weaponVelocity.x ** 2 + weaponVelocity.y ** 2),
    };
  }

  function rigMechanicalEnergy(snapshot) {
    return number(snapshot?.target) + number(snapshot?.rotor) + number(snapshot?.chassis) + number(snapshot?.fork);
  }

  function rigContactKinematics(contact, physics) {
    const contactPoint = contact.point;
    const movingBody = contact.role === 'fork' ? physics.forkBody : physics.weaponBody;
    const movingVelocity = movingBody.velocityAtPoint({ x: contactPoint.x, y: contactPoint.y });
    const targetVelocity = physics.targetBody.velocityAtPoint({ x: contactPoint.x, y: contactPoint.y });
    const relativeVelocity = point(movingVelocity.x - targetVelocity.x, movingVelocity.y - targetVelocity.y);
    return { relativeVelocity, relativeSpeed: length(relativeVelocity), normalVelocity: dot(relativeVelocity, contact.normal) };
  }

  function preContactKinematics(contact, before, physics) {
    const p = contact.point;
    const targetArm = subtract(p, before.targetCom || before.targetPosition);
    const targetVelocity = add(before.targetVelocity, point(-before.targetOmega * targetArm.y, before.targetOmega * targetArm.x));
    if (contact.role === 'fork') {
      const forkArm = subtract(p, before.forkCom || before.forkPosition);
      const toolVelocity = add(before.forkVelocity, point(-before.forkOmega * forkArm.y, before.forkOmega * forkArm.x));
      return { targetVelocity, toolVelocity, relativeVelocity: subtract(toolVelocity, targetVelocity) };
    }
    const weaponArm = subtract(p, before.weaponCom || before.weaponPosition);
    const toolVelocity = add(before.robotVelocity, point(-before.weaponOmega * weaponArm.y, before.weaponOmega * weaponArm.x));
    return { targetVelocity, toolVelocity, relativeVelocity: subtract(toolVelocity, targetVelocity), weaponArm };
  }

  function polygonHorizontalIntervals(polygon, y) {
    const xs = [];
    for (let index = 0; index < polygon.length; index += 1) {
      const a = polygon[index]; const b = polygon[(index + 1) % polygon.length];
      // Half-open edge rule counts a shared vertex exactly once.
      if (!((a.y <= y && b.y > y) || (b.y <= y && a.y > y))) continue;
      const fraction = (y - a.y) / (b.y - a.y);
      xs.push(a.x + (b.x - a.x) * fraction);
    }
    xs.sort((a, b) => a - b);
    const intervals = [];
    for (let index = 0; index + 1 < xs.length; index += 2) intervals.push([xs[index], xs[index + 1]]);
    return intervals;
  }

  function materialGeometryHorizontalIntervals(geometry, y) {
    const intervals = [];
    (geometry || []).forEach((polygon) => {
      if (!polygon.length) return;
      let solid = polygonHorizontalIntervals(polygon[0].map(([x, py]) => point(x, py)), y);
      for (let ring = 1; ring < polygon.length; ring += 1) {
        const holes = polygonHorizontalIntervals(polygon[ring].map(([x, py]) => point(x, py)), y);
        solid = subtractIntervals(solid, holes);
      }
      intervals.push(...solid);
    });
    return mergeIntervals(intervals);
  }

  function pointTouchesMaterialGeometry(localPoint, geometry, tolerance = GEOMETRY_CLEARANCE_EPS) {
    // Horizontal parity tests are intentionally half-open and therefore return
    // no interval exactly on a horizontal top/bottom edge.  Check every real
    // ring boundary first so a still-present edge/corner can never be mistaken
    // for freshly removed void and granted a penetration allowance.
    if (materialBoundarySegments(geometry)
      .some(({ start, end }) => pointSegmentDistance(localPoint, start, end) <= tolerance)) return true;
    return materialGeometryHorizontalIntervals(geometry, localPoint.y)
      .some(([start, end]) => localPoint.x >= start - tolerance && localPoint.x <= end + tolerance);
  }

  function physicsTargetLocalPoint(worldPoint, physics) {
    const position = physics.targetBody.translation();
    return rotate(subtract(worldPoint, point(position.x, position.y)), -physics.targetBody.rotation());
  }

  function clippingPolygonFromLoop(loop) {
    if (!loop?.length) return [];
    const ring = loop.map((p) => [p.x, p.y]);
    const first = ring[0]; const last = ring[ring.length - 1];
    if (!last || Math.hypot(first[0] - last[0], first[1] - last[1]) > 1e-12) ring.push([...first]);
    return [[ring]];
  }

  function predictedWeaponLoopInTarget(before, physics, intervalDt, fraction, localLoop = physics.weaponLoop) {
    const weaponPosition = add(before.weaponPosition, scalePoint(before.robotVelocity, intervalDt * fraction));
    const weaponAngle = before.weaponAngle + before.weaponOmega * intervalDt * fraction;
    const targetPosition = add(before.targetPosition, scalePoint(before.targetVelocity, intervalDt * fraction));
    const targetAngle = before.targetAngle + before.targetOmega * intervalDt * fraction;
    return localLoop.map((local) => {
      const world = add(weaponPosition, rotate(local, weaponAngle));
      return rotate(subtract(world, targetPosition), -targetAngle);
    });
  }

  function actualWeaponLoopInTarget(physics, localLoop = physics.weaponLoop) {
    const weaponPositionRaw = physics.weaponBody.translation(); const targetPositionRaw = physics.targetBody.translation();
    const weaponPosition = point(weaponPositionRaw.x, weaponPositionRaw.y); const targetPosition = point(targetPositionRaw.x, targetPositionRaw.y);
    const weaponAngle = physics.weaponBody.rotation(); const targetAngle = physics.targetBody.rotation();
    return localLoop.map((local) => rotate(
      subtract(add(weaponPosition, rotate(local, weaponAngle)), targetPosition),
      -targetAngle,
    ));
  }

  function shortestAngleDelta(start, end) {
    let delta = end - start;
    while (delta > Math.PI) delta -= Math.PI * 2;
    while (delta < -Math.PI) delta += Math.PI * 2;
    return delta;
  }

  function interpolatedWeaponLoopInTarget(before, physics, fraction, localLoop = physics.weaponLoop) {
    const t = clamp(fraction, 0, 1);
    const weaponEndRaw = physics.weaponBody.translation();
    const targetEndRaw = physics.targetBody.translation();
    const weaponPosition = add(before.weaponPosition, scalePoint(
      subtract(point(weaponEndRaw.x, weaponEndRaw.y), before.weaponPosition), t,
    ));
    const targetPosition = add(before.targetPosition, scalePoint(
      subtract(point(targetEndRaw.x, targetEndRaw.y), before.targetPosition), t,
    ));
    const weaponAngle = before.weaponAngle
      + shortestAngleDelta(before.weaponAngle, physics.weaponBody.rotation()) * t;
    const targetAngle = before.targetAngle
      + shortestAngleDelta(before.targetAngle, physics.targetBody.rotation()) * t;
    return localLoop.map((local) => rotate(
      subtract(add(weaponPosition, rotate(local, weaponAngle)), targetPosition),
      -targetAngle,
    ));
  }

  function adaptiveMaterialPoseLoops(before, physics, startFraction, localLoop) {
    const first = interpolatedWeaponLoopInTarget(before, physics, startFraction, localLoop);
    const last = actualWeaponLoopInTarget(physics, localLoop);
    const poses = [{ fraction: startFraction, loop: first }];
    let maxDeviation = 0; let failed = false;
    const subdivide = (f0, loop0, f1, loop1, depth) => {
      const fm = (f0 + f1) / 2;
      const middle = interpolatedWeaponLoopInTarget(before, physics, fm, localLoop);
      const deviation = middle.reduce((largest, value, index) => {
        const chordMid = scalePoint(add(loop0[index], loop1[index]), .5);
        return Math.max(largest, length(subtract(value, chordMid)));
      }, 0);
      maxDeviation = Math.max(maxDeviation, deviation);
      if (deviation > MATERIAL_SWEEP_GEOMETRY_TOLERANCE) {
        if (depth >= MATERIAL_SWEEP_MAX_RECURSION) { failed = true; return; }
        subdivide(f0, loop0, fm, middle, depth + 1);
        if (failed) return;
        subdivide(fm, middle, f1, loop1, depth + 1);
        return;
      }
      poses.push({ fraction: f1, loop: loop1 });
    };
    subdivide(startFraction, first, 1, last, 0);
    return { poses, maxDeviation, failed };
  }

  function sweptBoundaryComponents(poses) {
    const components = [];
    poses.forEach(({ loop }) => components.push(clippingPolygonFromLoop(loop)));
    for (let poseIndex = 0; poseIndex + 1 < poses.length; poseIndex += 1) {
      const start = poses[poseIndex].loop; const end = poses[poseIndex + 1].loop;
      for (let edge = 0; edge < start.length; edge += 1) {
        const next = (edge + 1) % start.length;
        const bridge = [start[edge], start[next], end[next], end[edge]];
        if (Math.abs(polygonArea(bridge)) > 1e-16) components.push(clippingPolygonFromLoop(bridge));
      }
    }
    return components.filter((component) => component?.length);
  }

  function mergeIntervals(intervals) {
    const sorted = intervals.filter(([start, end]) => end - start > 1e-12).sort((a, b) => a[0] - b[0]);
    const merged = [];
    sorted.forEach(([start, end]) => {
      const last = merged[merged.length - 1];
      if (last && start <= last[1] + 1e-10) last[1] = Math.max(last[1], end);
      else merged.push([start, end]);
    });
    return merged;
  }

  function subtractIntervals(segments, cuts) {
    let remaining = segments.map((segment) => [...segment]);
    mergeIntervals(cuts).forEach(([cutStart, cutEnd]) => {
      const next = [];
      remaining.forEach(([start, end]) => {
        if (cutEnd <= start || cutStart >= end) { next.push([start, end]); return; }
        if (cutStart > start + 1e-10) next.push([start, Math.min(cutStart, end)]);
        if (cutEnd < end - 1e-10) next.push([Math.max(cutEnd, start), end]);
      });
      remaining = next;
    });
    return remaining;
  }

  function segmentsLength(segments) { return segments.reduce((sum, [start, end]) => sum + Math.max(0, end - start), 0); }

  function materialSweepEnvelope(contact, before, intervalDt, physics) {
    const damage = state.sim.materialDamage;
    if (!damage || !Array.isArray(damage.geometry) || !damage.geometry.length || contact.role !== 'weapon' || !contact.isTooth) return null;
    const clipping = globalThis.polygonClipping;
    if (!clipping?.union || !clipping?.difference) return { invalid: true, reason: '精确材料多边形求交模块未加载' };
    const cuttingLoop = physics.weaponToothLoops?.[contact.toothOrder];
    if (!cuttingLoop?.length) return { invalid: true, reason: '当前识别牙没有可闭合的 CAD 牙瓣；为避免把刀体误当切削刃，本次不删除材料' };
    const kinematics = preContactKinematics(contact, before, physics);
    const relativeSpeed = length(kinematics.relativeVelocity);
    const closingSpeed = Math.max(0, -dot(kinematics.relativeVelocity, contact.normal));
    const delay = contact.geometricGap > 0 && closingSpeed > 1e-9 ? contact.geometricGap / closingSpeed : 0;
    const activeDt = clamp(intervalDt - delay, 0, intervalDt);
    if (activeDt <= 1e-12 && contact.penetration <= 0) return null;

    const halfLength = targetLength() / 2; const halfThickness = targetThickness() / 2;
    // Contact penetration reported by a convex sub-shape is not distance along
    // the cutting path (for a rotating concave CAD body it may be millimetres
    // while the edge travelled micrometres).  Only integrated relative motion
    // is a valid path length for chip volume/work.
    const pathLength = Math.max(relativeSpeed * activeDt, 1e-9);
    const startFraction = clamp(delay / Math.max(intervalDt, 1e-12), 0, 1);
    try {
      const adaptive = adaptiveMaterialPoseLoops(before, physics, startFraction, cuttingLoop);
      if (adaptive.failed) return {
        invalid: true,
        reason: `材料扫掠边界在 ${MATERIAL_SWEEP_MAX_RECURSION} 级细分后仍不能达到 ${format(MATERIAL_SWEEP_GEOMETRY_TOLERANCE * 1000, 6)} mm 几何误差；本步不输出切削量`,
      };
      // The union contains every sampled solid and the swept quadrilateral of
      // every real CAD boundary segment between adjacent poses. Unlike a union
      // of start/mid/end snapshots, pure translation cannot leave an unmodelled
      // gap; rotational arc error is bounded by the adaptive midpoint test.
      const components = sweptBoundaryComponents(adaptive.poses);
      const sweptGeometry = clipping.union(...components);
      const desiredGeometry = clipping.difference(damage.geometry, sweptGeometry);
      const oldArea = materialGeometryArea(damage.geometry);
      const remainingArea = materialGeometryArea(desiredGeometry);
      const freshArea = Math.max(0, oldArea - remainingArea);
      const newCrackLength = freshArea > 1e-15
        ? materialNewBoundaryLength(damage.geometry, desiredGeometry)
        : 0;
      const initialSegments = Array.from({ length: damage.rows }, (_, row) => damageRowSegments(damage, row).map((segment) => [...segment]));
      const desiredSegments = Array.from({ length: damage.rows }, (_, row) => {
        const y = -halfThickness + (row + .5) * damage.rowHeight;
        return materialGeometryHorizontalIntervals(desiredGeometry, y)
          .map(([start, end]) => [Math.max(start, -halfLength), Math.min(end, halfLength)])
          .filter(([start, end]) => end - start > 1e-12);
      });
      const rowAdvances = initialSegments.map((segments, row) => Math.max(0, segmentsLength(segments) - segmentsLength(desiredSegments[row])));
      const maxAdvance = Math.max(0, ...rowAdvances);
      const desiredDepths = desiredSegments.map((segments) => segments.length ? Math.max(0, segments[0][0] + halfLength) : targetLength());
      const desiredRightDepths = desiredSegments.map((segments) => segments.length ? Math.max(0, halfLength - segments[segments.length - 1][1]) : targetLength());
      return {
        desiredGeometry, desiredSegments, desiredDepths, desiredRightDepths, rowAdvances,
        freshArea, maxAdvance, pathLength, newCrackLength,
        effectiveThickness: pathLength > 1e-12 ? freshArea / pathLength : 0,
        activeDt, relativeSpeed, closingSpeed,
        sweepPoseCount: adaptive.poses.length,
        sweepGeometryError: adaptive.maxDeviation,
      };
    } catch (error) {
      return { invalid: true, reason: `精确材料多边形差集失败：${error?.message || error}` };
    }
  }

  function bodyInverseMassAtPoint(body, worldPoint, direction) {
    const mass = Math.max(number(body.mass?.()), 1e-12);
    const inertia = Math.max(number(body.effectiveAngularInertia?.()) || number(body.principalInertia?.()), 1e-12);
    const comRaw = body.worldCom?.() || body.translation(); const com = point(comRaw.x, comRaw.y);
    return 1 / mass + cross(subtract(worldPoint, com), direction) ** 2 / inertia;
  }

  function pairStoppingImpulse(toolBody, targetBody, worldPoint, direction, relativeSpeed) {
    const inverse = bodyInverseMassAtPoint(toolBody, worldPoint, direction)
      + bodyInverseMassAtPoint(targetBody, worldPoint, direction);
    return inverse > 1e-12 ? Math.max(0, relativeSpeed) / inverse : 0;
  }

  function rigidBodyKineticEnergy(body) {
    const velocity = body.linvel(); const mass = Math.max(number(body.mass?.()), 0);
    const inertia = Math.max(number(body.principalInertia?.()), 0); const omega = body.angvel();
    return .5 * mass * (velocity.x ** 2 + velocity.y ** 2) + .5 * inertia * omega ** 2;
  }

  function contactPairKineticEnergy(toolBody, targetBody) {
    return rigidBodyKineticEnergy(toolBody) + rigidBodyKineticEnergy(targetBody);
  }

  function applyPairImpulse(toolBody, targetBody, worldPoint, impulseOnTarget) {
    targetBody.applyImpulseAtPoint({ x: impulseOnTarget.x, y: impulseOnTarget.y }, worldPoint, true);
    toolBody.applyImpulseAtPoint({ x: -impulseOnTarget.x, y: -impulseOnTarget.y }, worldPoint, true);
  }

  function weaponRigInverseMassAtPoint(physics, worldPoint, direction) {
    const axisRaw = physics.weaponBody.translation(); const axis = point(axisRaw.x, axisRaw.y);
    const totalMass = positive(state.params.robotMass, .01);
    // The chassis has one scene DOF (X); vertical reaction is supplied by the
    // arena through the wheels.  The weapon contributes rotation about the
    // revolute axis, not free translation of its small I/R^2 mass proxy.
    return direction.x ** 2 / totalMass
      + cross(subtract(worldPoint, axis), direction) ** 2 / Math.max(state.metrics.inertia, 1e-12);
  }

  function weaponTargetStoppingImpulse(physics, worldPoint, direction, relativeSpeed) {
    const inverse = weaponRigInverseMassAtPoint(physics, worldPoint, direction)
      + bodyInverseMassAtPoint(physics.targetBody, worldPoint, direction);
    return inverse > 1e-12 ? Math.max(0, relativeSpeed) / inverse : 0;
  }

  function weaponTargetInverseMassCoupling(physics, worldPoint, leftDirection, rightDirection) {
    const axisRaw = physics.weaponBody.translation(); const axis = point(axisRaw.x, axisRaw.y);
    const targetComRaw = physics.targetBody.worldCom?.() || physics.targetBody.translation();
    const targetCom = point(targetComRaw.x, targetComRaw.y);
    const rotorArm = subtract(worldPoint, axis); const targetArm = subtract(worldPoint, targetCom);
    const totalMass = positive(state.params.robotMass, .01);
    const targetMass = Math.max(number(physics.targetBody.mass?.()), 1e-12);
    const targetInertia = Math.max(number(physics.targetBody.principalInertia?.()), 1e-12);
    return leftDirection.x * rightDirection.x / totalMass
      + cross(rotorArm, leftDirection) * cross(rotorArm, rightDirection) / Math.max(state.metrics.inertia, 1e-12)
      + dot(leftDirection, rightDirection) / targetMass
      + cross(targetArm, leftDirection) * cross(targetArm, rightDirection) / targetInertia;
  }

  function solveCoupledCoulombImpulse(physics, worldPoint, normalDirection, relativeVelocity, friction, restitution = 0) {
    const tangent = perpendicular(normalDirection);
    const vn = Math.max(0, dot(relativeVelocity, normalDirection));
    const vt = dot(relativeVelocity, tangent);
    const kNN = weaponTargetInverseMassCoupling(physics, worldPoint, normalDirection, normalDirection);
    const kNT = weaponTargetInverseMassCoupling(physics, worldPoint, normalDirection, tangent);
    const kTT = weaponTargetInverseMassCoupling(physics, worldPoint, tangent, tangent);
    const rhsN = vn * (1 + clamp(restitution, 0, 1)); const rhsT = vt;
    const determinant = kNN * kTT - kNT * kNT;
    let normalImpulse = 0; let tangentImpulse = 0; let mode = 'separating';
    if (rhsN > 0 && determinant > 1e-18) {
      const stickN = (rhsN * kTT - rhsT * kNT) / determinant;
      const stickT = (rhsT * kNN - rhsN * kNT) / determinant;
      if (stickN >= 0 && Math.abs(stickT) <= Math.max(0, friction) * stickN + 1e-12) {
        normalImpulse = stickN; tangentImpulse = stickT; mode = 'stick';
      } else {
        const slideSign = Math.sign(vt);
        const denominator = kNN + slideSign * Math.max(0, friction) * kNT;
        normalImpulse = denominator > 1e-18 ? Math.max(0, rhsN / denominator) : 0;
        tangentImpulse = slideSign * Math.max(0, friction) * normalImpulse;
        mode = 'slide';
      }
    }
    return {
      normalImpulse,
      tangentImpulse,
      impulse: add(scalePoint(normalDirection, normalImpulse), scalePoint(tangent, tangentImpulse)),
      mode,
      matrix: { kNN, kNT, kTT },
    };
  }

  function weaponTargetGeneralisedKineticEnergy(physics) {
    const chassisVx = physics.robotBody.linvel().x;
    return rigidBodyKineticEnergy(physics.targetBody)
      + .5 * positive(state.params.robotMass, .01) * chassisVx ** 2
      + .5 * Math.max(state.metrics.inertia, 1e-12) * physics.weaponBody.angvel() ** 2;
  }

  function applyWeaponTargetImpulse(physics, worldPoint, impulseOnTarget) {
    physics.targetBody.applyImpulseAtPoint(
      { x: impulseOnTarget.x, y: impulseOnTarget.y },
      worldPoint,
      true,
    );
    const axisRaw = physics.weaponBody.translation(); const axis = point(axisRaw.x, axisRaw.y);
    const totalMass = positive(state.params.robotMass, .01);
    const nextVx = physics.robotBody.linvel().x - impulseOnTarget.x / totalMass;
    const nextOmega = physics.weaponBody.angvel()
      - cross(subtract(worldPoint, axis), impulseOnTarget) / Math.max(state.metrics.inertia, 1e-12);
    // Material contact is solved directly in the rig's two generalised DOFs.
    // Keeping both linked bodies at the same axis velocity avoids asking the
    // revolute joint to erase an incompatible free-body impulse one substep
    // later (the former source of large, unclassified energy loss).
    physics.robotBody.setLinvel({ x: nextVx, y: 0 }, true);
    physics.weaponBody.setLinvel({ x: nextVx, y: 0 }, true);
    physics.weaponBody.setAngvel(nextOmega, true);
  }

  function materialContactEstimate(contact, before, intervalDt, physics) {
    const material = targetMaterialProperties();
    const toolMaterial = contact.role === 'fork' ? getShovelMaterial() : getWeaponMaterial();
    const width = effectiveToolWidthZ(contact.role);
    const edgeRadius = positive(number(state.params.edgeRadius) / 1000, .00001);
    const kinematics = preContactKinematics(contact, before, physics);
    const relativeSpeed = length(kinematics.relativeVelocity);
    const targetYieldStress = positive(material.yieldStrength, 1) * 1e6;
    const targetShearStress = positive(material.shearStrength, material.yieldStrength / Math.sqrt(3)) * 1e6;
    const minChipThickness = Math.max(0, number(material.minChipThickness));
    const hasTraceableMinChipThickness = minChipThickness > 0 && Boolean(material.minChipSource);
    const hasTraceableFractureEnergy = Math.max(0, number(material.fractureEnergy)) > 0
      && Boolean(material.fractureSource);
    const toolYieldStrength = positive(toolMaterial.yieldStrength, material.yieldStrength);
    const estimate = {
      model: 'elastic-plastic', width, edgeRadius, uncutThickness: 0, feedPerTooth: 0,
      rakeAngle: null, shearAngle: null, cuttingForce: 0, thrustForce: 0,
      plasticCuttingForce: 0, fractureForce: 0, fractureWork: 0,
      resultantForce: 0, impulseLimit: 0, requiredMaterialWork: 0, materialWork: 0, relativeSpeed,
      relativeVelocity: { ...kinematics.relativeVelocity },
      validity: material.validity, source: material.source, ploughing: false,
      minChipThickness, minChipSource: material.minChipSource || null,
    };
    if (width <= 1e-9 || relativeSpeed <= 1e-9) {
      estimate.validity = width <= 1e-9 ? 'Z 向没有实体重叠，因此不应产生接触冲量' : '接触点相对速度为零';
      return estimate;
    }

    if (contact.role === 'weapon' && contact.parametric) {
      // The parameter tooth has a literal working face and finite triangular
      // back support, but no independently specified microscopic tip radius,
      // relief face, fracture path or validated continuous-removal TOI.
      estimate.model = 'plastic-edge';
      estimate.ploughing = true;
      estimate.edgeRadius = 0;
      estimate.uncutThickness = Math.min(targetThickness(), Math.max(contact.penetration, GEOMETRY_CLEARANCE_EPS));
      estimate.resultantForce = targetYieldStress * width * estimate.uncutThickness;
      estimate.thrustForce = estimate.resultantForce;
      estimate.cuttingForce = 0;
      estimate.sweep = null;
      estimate.noMaterialRemoval = true;
      estimate.validity = `${material.validity}；参数测试齿定义真实迎击面与有限背撑，但未定义可追溯微观刃圆/后角/断裂路径；本次仅按实际几何侵入计算有限塑性接触，不删除材料或声明切屑体积`;
    } else if (contact.role === 'weapon' && contact.isTooth) {
      const toothFrequency = Math.abs(before.weaponOmega) / (Math.PI * 2) * state.metrics.cadToothCount;
      const feedSpeed = Math.max(0, before.robotSpeed - kinematics.targetVelocity.x);
      const feedPerTooth = toothFrequency > 1e-9 ? feedSpeed / toothFrequency : 0;
      const sweep = materialSweepEnvelope(contact, before, intervalDt, physics);
      if (sweep?.invalid) {
        estimate.model = 'geometry-domain'; estimate.invalidReason = sweep.reason;
        estimate.validity = sweep.reason; estimate.sweep = null;
        return estimate;
      }
      // The force-bearing chip thickness comes from fresh CAD swept area /
      // cutting-path length.  This is the volume identity V=b*h*s, so very high
      // RPM with little new feed naturally tends toward a small h without an
      // arbitrary engagement coefficient.  The conventional feed/tooth value
      // is retained separately as a kinematic readout.
      // Keep V = b*h*s exact.  Clamping h while still committing the complete
      // polygon difference made the simulation delete more volume than the
      // material-work calculation paid for.  A large swept section must demand
      // proportionally large work (and may make the substep invalid); it must
      // never be silently truncated only in the force equation.
      const uncutThickness = Math.max(0, sweep?.effectiveThickness || 0);
      estimate.feedPerTooth = feedPerTooth; estimate.uncutThickness = uncutThickness; estimate.sweep = sweep;
      // h_min/r_e depends on the tool, work material, speed and loading
      // history; edge radius by itself is not a material law. Without a
      // traceable h_min measurement the response remains finite plastic
      // ploughing but may not delete a detached chip.
      estimate.ploughing = uncutThickness > 0
        && (!hasTraceableMinChipThickness || !hasTraceableFractureEnergy || uncutThickness <= minChipThickness);

      const loop = physics.weaponLoop; const index = clamp(number(contact.colliderIndex), 0, loop.length - 1);
      const segment = normalise(subtract(loop[(index + 1) % loop.length], loop[index]));
      let faceTangent = rotate(segment, before.weaponAngle);
      const cuttingDirection = normalise(kinematics.relativeVelocity);
      const cuttingNormal = perpendicular(cuttingDirection);
      if (dot(faceTangent, cuttingNormal) < 0) faceTangent = scalePoint(faceTangent, -1);
      const rakeAngle = Math.asin(clamp(dot(faceTangent, cuttingDirection), -1, 1));
      const beta = Math.atan(weaponTargetFriction());
      const shearAngle = Math.PI / 4 + rakeAngle / 2 - beta / 2;
      const denominator = Math.cos(shearAngle + beta - rakeAngle);
      estimate.rakeAngle = rakeAngle; estimate.shearAngle = shearAngle;

      if (estimate.ploughing) {
        // Merchant's sharp-edge chip model is outside its domain when the
        // undeformed chip is no thicker than the entered edge radius.  Treat
        // that interval as finite plastic ploughing: it may dissipate energy
        // and deflect the target, but it does not claim detached chip volume.
        estimate.model = 'plastic-ploughing';
        estimate.cuttingForce = targetYieldStress * width * uncutThickness;
        estimate.resultantForce = estimate.cuttingForce;
        estimate.validity = !hasTraceableMinChipThickness
          ? `${material.validity}；未提供同刀具、靶材与速度条件的实测最小切屑厚度 h_min，本步保守地只计算有限塑性犁削，不删除材料或声明切屑体积`
          : (!hasTraceableFractureEnergy
            ? `${material.validity}；未提供有来源的断裂能 Gc，本步只计算有限塑性犁削，不删除材料或声明真实断裂体积`
            : `${material.validity}；未切削厚度 ${format(uncutThickness * 1000, 5)} mm 不大于实测 h_min=${format(minChipThickness * 1000, 5)} mm（${material.minChipSource}），本步只计算有限塑性犁削而不删除材料`);
      } else if (uncutThickness > 0 && material.model === 'merchant' && shearAngle > radians(5) && shearAngle < radians(85) && Math.abs(denominator) > .05) {
        const shearForce = targetShearStress * width * uncutThickness / Math.sin(shearAngle);
        const cuttingForce = shearForce * Math.cos(beta - rakeAngle) / denominator;
        const thrustForce = cuttingForce * Math.tan(beta - rakeAngle);
        if (Number.isFinite(cuttingForce) && Number.isFinite(thrustForce) && cuttingForce > 0) {
          // Merchant's Ft/Fc relation is signed.  Taking abs(Ft) and then
          // forcing it toward separation turned a negative thrust solution into
          // an artificial upward launch force at high rake angles.
          estimate.model = 'merchant'; estimate.cuttingForce = cuttingForce; estimate.thrustForce = thrustForce;
          estimate.resultantForce = Math.hypot(cuttingForce, thrustForce);
        } else {
          estimate.model = 'direct-shear';
          estimate.cuttingForce = targetShearStress * width * uncutThickness;
          estimate.thrustForce = 0; estimate.resultantForce = estimate.cuttingForce;
          estimate.validity = `${material.validity}；Merchant 力圆给出非压缩/非正切削力，已退回直接剪切下界`;
        }
      } else if (uncutThickness > 0) {
        // For anisotropic plywood or a Merchant geometry outside its stated
        // domain, charge only a direct shear-plane resistance. It is an
        // explicit lower-bound strength estimate, not a hidden fit factor.
        estimate.model = 'direct-shear';
        estimate.cuttingForce = targetShearStress * width * uncutThickness;
        estimate.resultantForce = estimate.cuttingForce;
        if (material.model !== 'shear') estimate.validity = `${material.validity}；当前齿面超出 Merchant 适用角，已退回直接剪切下界`;
      } else if (contact.penetration > 0) {
        // Incipient point contact can be smaller than the damage raster and
        // therefore have zero resolved swept area.  It is not an infinitely
        // rigid wall: use the measured geometric indentation as the projected
        // plastic contact height until a resolvable chip section develops.
        const indentation = Math.min(edgeRadius, Math.max(contact.penetration, GEOMETRY_CLEARANCE_EPS));
        estimate.model = 'plastic-ploughing'; estimate.ploughing = true;
        estimate.uncutThickness = indentation;
        estimate.cuttingForce = targetYieldStress * width * indentation;
        estimate.resultantForce = estimate.cuttingForce;
        estimate.validity = `${material.validity}；接触压入小于切口离散分辨率，按有限塑性犁削求解，不作刚性冲量或虚假删料`;
      }
    } else {
      // A non-tooth body or fork edge is not a cutter. Its first plastic
      // footprint is the Z overlap times the measured/entered edge diameter.
      // Below this force Rapier's non-penetration solution is elastic; above
      // it the transmitted impulse is limited by the weaker material flow.
      const sweep = null;
      estimate.sweep = null;
      estimate.model = 'plastic-edge'; estimate.ploughing = true;
      estimate.uncutThickness = Math.max(edgeRadius * 2, Math.min(sweep?.effectiveThickness || 0, targetThickness()));
      estimate.resultantForce = targetYieldStress * width * estimate.uncutThickness;
      estimate.cuttingForce = 0;
      estimate.thrustForce = estimate.resultantForce;
      estimate.validity = `${material.validity}；非刀齿接触按刃口直径投影的首次塑性屈服估算`;
    }
    if (contact.isTooth && estimate.sweep?.freshArea > 1e-15 && !estimate.ploughing && estimate.cuttingForce > 0) {
      if (toolYieldStrength + 1e-9 < material.yieldStrength) {
        estimate.model = 'tool-failure-domain';
        estimate.invalidReason = `刀具屈服强度 ${format(toolYieldStrength, 3)} MPa 低于靶材 ${format(material.yieldStrength, 3)} MPa；当前模型没有刀具塑性/崩刃自由度，不能把较软刀具误当成降低靶材切削强度`;
        estimate.validity = estimate.invalidReason;
        return estimate;
      }
      estimate.plasticCuttingForce = estimate.cuttingForce;
      const fractureEnergy = Math.max(0, material.fractureEnergy || 0);
      estimate.fractureWork = fractureEnergy * width * Math.max(0, estimate.sweep.newCrackLength || 0);
      estimate.fractureForce = estimate.sweep.pathLength > 1e-12
        ? estimate.fractureWork / estimate.sweep.pathLength
        : 0;
      estimate.cuttingForce += estimate.fractureForce;
      estimate.resultantForce = Math.hypot(estimate.cuttingForce, estimate.thrustForce);
      if (fractureEnergy > 0) {
        estimate.validity = `${estimate.validity}；已按材料多边形前后实际新增裂纹边界 ${format((estimate.sweep.newCrackLength || 0) * 1000, 4)} mm 计入 Gc=${format(fractureEnergy, 0)} J/m² 的断裂功（${material.fractureSource || '用户输入'}）`;
      } else {
        estimate.validity = `${estimate.validity}；未提供同材料/状态/速率的 Gc，当前删料功仅为塑性与摩擦下界，不能作为真实破坏量定论`;
      }
    }
    const effectiveDt = estimate.sweep?.activeDt ?? intervalDt;
    estimate.impulseLimit = Math.max(0, estimate.resultantForce * effectiveDt);
    estimate.requiredMaterialWork = estimate.sweep
      ? Math.max(0, estimate.cuttingForce * estimate.sweep.pathLength)
      : 0;
    // Before the impulse is applied this is only the constitutive work demand.
    // recordMaterialCut replaces it with the measured two-body kinetic-energy
    // loss, which is the authoritative committed energy ledger.
    estimate.materialWork = estimate.requiredMaterialWork;
    return estimate;
  }

  function recordMaterialCut(contact, estimate, motionFraction = 1, impulseWork = 0) {
    const sim = state.sim; const damage = sim.materialDamage; const sweep = estimate.sweep;
    if (!damage || !sweep || contact.role !== 'weapon' || !contact.isTooth || estimate.resultantForce <= 0) return 0;
    const fraction = clamp(motionFraction, 0, 1);
    if (fraction < 1 - 1e-9) return -1;
    const fullArea = targetLength() * targetThickness();
    const oldRemainingArea = Array.isArray(damage.geometry)
      ? materialGeometryArea(damage.geometry)
      : Array.from({ length: damage.rows }, (_, row) => segmentsLength(damageRowSegments(damage, row)))
        .reduce((sum, lengthX) => sum + lengthX * damage.rowHeight, 0);
    const oldArea = Math.max(0, fullArea - oldRemainingArea);
    const desiredGeometry = cloneMaterialGeometry(sweep.desiredGeometry);
    const desiredSegments = sweep.desiredSegments.map((segments) => segments.map((segment) => [...segment]));
    const desiredDepths = [...sweep.desiredDepths];
    const desiredRightDepths = [...sweep.desiredRightDepths];
    const desiredPending = damage.pending.map((value, row) => value + (sweep.rowAdvances[row] || 0));
    const remainingArea = materialGeometryArea(desiredGeometry);
    const removedArea = Math.max(0, fullArea - remainingArea);
    const freshArea = Math.max(0, removedArea - oldArea);
    if (freshArea <= 1e-15) { delete estimate.sweep; return 0; }
    const removedVolume = freshArea * estimate.width; const maxAdvance = sweep.maxAdvance;
    const candidateDamage = {
      ...damage,
      geometry: desiredGeometry,
      segments: desiredSegments,
      depths: desiredDepths,
      rightDepths: desiredRightDepths,
      pending: desiredPending,
      removedArea,
    };
    const massProperties = targetDamageMassProperties(candidateDamage);
    if (massProperties.invalid) { delete estimate.sweep; return -1; }
    // Commit all geometry fields atomically only after the area delta and mass
    // properties are valid.  A sub-threshold polygon normalisation must not
    // mutate geometry behind an unchanged playback version.
    damage.geometry = desiredGeometry;
    damage.segments = desiredSegments;
    damage.depths = desiredDepths;
    damage.rightDepths = desiredRightDepths;
    damage.pending = desiredPending;
    damage.removedArea = removedArea; damage.dirty = true; damage.version += 1;
    if (contact.isTooth && (damage.activeTooth === null || damage.activeTooth === undefined)) {
      damage.activeTooth = contact.toothOrder;
      damage.activeStartTime = sim.time;
      damage.activeAngularTravel = 0;
    }
    damage.lastCutTime = sim.time;
    const localPoint = worldToTarget(contact.point);
    damage.cuts.push({ x: localPoint.x, y: localPoint.y, depth: maxAdvance, time: sim.time });
    if (damage.cuts.length > 256) damage.cuts.shift();
    damage.removedMass = massProperties.removedMass;
    sim.materialStats.removedArea = removedArea;
    sim.materialStats.removedVolume = removedArea * estimate.width;
    sim.materialStats.removedMass = massProperties.removedMass;
    // This is the actual mechanical energy dissipated by the equal-and-opposite
    // material impulse.  F*s based on the unconstrained ballistic path double
    // counted work near a stopping contact, while the event card independently
    // discarded tangential work.  One measured impulse-energy ledger is now the
    // sole material-work source.
    const work = Math.max(0, impulseWork);
    sim.materialStats.work += work;
    estimate.materialWork = work; estimate.freshArea = freshArea; estimate.removedVolume = removedVolume;
    // The old collider may report this already-removed overlap during the
    // prediction substep.  It is acceptable only because the replacement
    // collider is built immediately before the next integration substep.
    estimate.allowedPenetration = Math.max(contact.penetration, maxAdvance);
    delete estimate.sweep;
    return estimate.allowedPenetration;
  }

  function triangulateMaterialGeometry(geometry) {
    const triangulator = globalThis.earcut?.default || globalThis.earcut;
    if (typeof triangulator !== 'function') return null;
    const triangles = [];
    geometry.forEach((polygon) => {
      const vertices = []; const holes = []; let vertexCount = 0;
      polygon.forEach((ring, ringIndex) => {
        const open = ring.length > 1 && Math.hypot(ring[0][0] - ring[ring.length - 1][0], ring[0][1] - ring[ring.length - 1][1]) <= 1e-12
          ? ring.slice(0, -1) : ring;
        if (ringIndex > 0) holes.push(vertexCount);
        open.forEach(([x, y]) => { vertices.push(x, y); vertexCount += 1; });
      });
      const indices = triangulator(vertices, holes, 2);
      for (let index = 0; index + 2 < indices.length; index += 3) {
        const triangle = [indices[index], indices[index + 1], indices[index + 2]].map((vertexIndex) => point(vertices[vertexIndex * 2], vertices[vertexIndex * 2 + 1]));
        if (Math.abs(polygonArea(triangle)) > 1e-14) triangles.push(triangle);
      }
    });
    return triangles;
  }

  function rebuildMaterialTargetCollider(physics) {
    const damage = state.sim.materialDamage;
    if (!physics?.materialCuttingEnabled || !damage?.dirty) return true;
    const geometry = targetRemainingLocalGeometry(damage);
    if (!state.rapier || !geometry.length || typeof physics.targetBody.setAdditionalMassProperties !== 'function') return false;
    const massProperties = targetDamageMassProperties(damage);
    if (massProperties.invalid || !(massProperties.inertia > 0)) return false;
    const replacements = [];
    const previousComRaw = physics.targetBody.worldCom?.() || physics.targetBody.translation();
    const previousCom = point(previousComRaw.x, previousComRaw.y);
    const previousVelocityRaw = physics.targetBody.linvel();
    const previousVelocity = point(previousVelocityRaw.x, previousVelocityRaw.y);
    const previousOmega = physics.targetBody.angvel();
    try {
      replacements.push(...createMaterialBoundaryColliders(
        physics.world,
        physics.targetBody,
        geometry,
        {
          friction: 0,
          restitution: 0,
          groups: physics.targetMaterialGroups,
          solverGroups: physics.targetMaterialSolverGroups,
          density: 0,
        },
      ));
      if (!replacements.length) throw new Error('靶子已无可求解的剩余材料边界');
      const previous = physics.targetMaterialColliders || [];
      previous.forEach((collider) => physics.world.removeCollider(collider, true));
      physics.targetBody.setAdditionalMassProperties(
        massProperties.remainingMass,
        massProperties.center,
        massProperties.inertia,
        true,
      );
      const nextComRaw = physics.targetBody.worldCom?.() || physics.targetBody.translation();
      const comShift = point(nextComRaw.x - previousCom.x, nextComRaw.y - previousCom.y);
      // Removing a chip does not teleport the velocity field of the material
      // that remains.  Rapier stores COM velocity, so shift it by ω×ΔCOM when
      // the local mass centre changes.
      physics.targetBody.setLinvel({
        x: previousVelocity.x - previousOmega * comShift.y,
        y: previousVelocity.y + previousOmega * comShift.x,
      }, true);
      physics.targetMaterialColliders = replacements;
      physics.targetColliders = [physics.targetCollider, ...replacements];
    } catch (_) {
      replacements.forEach((collider) => { try { physics.world.removeCollider(collider, true); } catch (__){ /* tick snapshot rolls back */ } });
      return false;
    }
    damage.pending.fill(0); damage.rightPending?.fill(0); damage.dirty = false;
    return true;
  }

  function applyMaterialContactLimits(contacts, before, intervalDt, physics) {
    if (state.params.contactModel === 'rigid') return { ok: true };
    if (!physics?.materialResponseEnabled) {
      // A cut depth cannot be inferred from strength/hardness alone.  Until the
      // user supplies a traceable minimum-chip thickness and fracture energy for
      // this tool/material/speed range, keep Rapier's ordinary non-penetration
      // response and remove no material.  This is a physical lower-bound model,
      // not an empirically tuned cutting-efficiency fallback.
      const cuttingSafetyGated = Boolean(physics?.materialCuttingRequested)
        && !physics?.materialCuttingEnabled;
      const validity = cuttingSafetyGated
        ? 'h_min 与 Gc 已提供，但完整剩余材料边界的连续最早压缩 TOI 尚未通过安全验证；本次退回 Rapier 非穿透刚体边界，不删除材料、不报告切屑体积。'
        : '未提供同刀具、靶材和速度条件下可追溯的最小切屑厚度 h_min 与断裂能 Gc；本次仅求解非穿透刚体边界，不删除材料、不报告切屑体积。';
      contacts.filter((contact) => contact.role === 'weapon').forEach((contact) => {
        contact.material = {
          model: cuttingSafetyGated ? 'cutting-safety-gated' : 'no-removal-boundary',
          materialWork: 0,
          allowedPenetration: 0,
          validity,
        };
      });
      return { ok: true, massRemovalEnergy: 0, noRemovalBoundary: true };
    }
    if (state.sim.materialDamage?.activeTooth != null) {
      state.sim.materialDamage.activeAngularTravel += Math.abs(before.weaponOmega) * intervalDt;
    }
    // Rapier reports predictive manifolds before the exact CAD surfaces touch.
    // In material mode that pair has no built-in solver, so a far predictive
    // manifold is telemetry only; applying our boundary impulse there was a
    // literal force at a distance and tripped the out-of-band domain gate.
    const weaponContacts = contacts.filter((contact) => contact.role === 'weapon'
      && (contact.penetration > 0 || contact.geometricGap <= RIG_CONTACT_EVENT_MAX_GAP));
    const grouped = new Map();
    weaponContacts.forEach((contact) => {
      const key = contact.isTooth ? `tooth:${contact.toothOrder}` : 'body';
      const kinematics = preContactKinematics(contact, before, physics);
      const closing = Math.max(0, -dot(kinematics.relativeVelocity, contact.normal));
      const score = contact.penetration + closing * intervalDt - Math.max(0, contact.geometricGap);
      const previous = grouped.get(key);
      if (!previous || score > previous.score) grouped.set(key, { contact, kinematics, score, members: [] });
    });
    weaponContacts.forEach((contact) => {
      const key = contact.isTooth ? `tooth:${contact.toothOrder}` : 'body';
      grouped.get(key)?.members.push(contact);
    });

    let materialStepValid = true; let materialFailureReason = null;
    [...grouped.entries()]
      .sort(([left], [right]) => Number(!left.startsWith('tooth:')) - Number(!right.startsWith('tooth:')))
      .forEach(([, group]) => {
      if (!materialStepValid) return;
      const { contact, kinematics, members } = group;
      const followsActiveTooth = !contact.isTooth && state.sim.materialDamage?.activeTooth != null;
      if (followsActiveTooth) contact.cutFollower = true;
      const predictiveOnly = contact.penetration <= 0 && contact.geometricGap > GEOMETRY_CLEARANCE_EPS;
      if (predictiveOnly) {
        // Do this before any polygon boolean work: the prediction shell is a
        // broad-phase query only and must neither consume CPU nor apply force.
        const proximity = {
          model: 'predictive-proximity', materialWork: 0, allowedPenetration: 0,
          validity: `CAD 表面仍有 ${format(contact.geometricGap * 1000, 5)} mm 正间隙；仅作为接近查询，不施力、不计命中或 bite`,
        };
        contact.impulse = 0; contact.tangentImpulse = 0; contact.tangentImpulseSigned = 0;
        contact.suppressContactEvent = true; contact.material = proximity;
        members.forEach((member) => {
          member.impulse = 0; member.tangentImpulse = 0; member.tangentImpulseSigned = 0;
          member.suppressContactEvent = true; member.material = { ...proximity };
        });
        return;
      }
      const estimate = materialContactEstimate(contact, before, intervalDt, physics);
      if (estimate.invalidReason) {
        materialStepValid = false; materialFailureReason = estimate.invalidReason;
        return;
      }
      const relativeSpeed = length(kinematics.relativeVelocity);
      const hasFreshCut = contact.isTooth && estimate.sweep?.freshArea > 1e-15
        && estimate.resultantForce > 0 && !estimate.ploughing;
      const hasFiniteMaterialResponse = estimate.resultantForce > 0 && relativeSpeed > 1e-9
        && (hasFreshCut || estimate.ploughing);
      const postCutGeometry = hasFreshCut ? estimate.sweep.desiredGeometry : null;
      if (hasFreshCut && estimate.sweep.desiredGeometry?.length > 1
        && estimate.width >= targetWidthZ() - GEOMETRY_CLEARANCE_EPS) {
        materialStepValid = false;
        materialFailureReason = '切口已在完整 Z 宽上把靶子分成多个独立刚体；当前单靶刚体模型不能继续，需启用碎片/断裂多刚体模型';
        return;
      }
      let impulseOnTarget = point(0, 0); let normalImpulse = 0; let tangentSigned = 0;

      if (hasFiniteMaterialResponse) {
        // The impulse is applied after Rapier has advanced this substep, so its
        // direction and stopping bound must use the current contact velocity.
        // Using the interval-start velocity could push an already-rotated point
        // in the wrong direction and create a tiny amount of kinetic energy.
        const responseKinematics = rigContactKinematics(contact, physics);
        const responseSpeed = length(responseKinematics.relativeVelocity);
        if (responseSpeed <= 1e-9) { delete estimate.sweep; return; }
        const cuttingDirection = normalise(responseKinematics.relativeVelocity);
        const separationDirection = scalePoint(contact.normal, -1);
        let thrustDirection = perpendicular(cuttingDirection);
        if (dot(thrustDirection, separationDirection) < 0) thrustDirection = scalePoint(thrustDirection, -1);
        const effectiveDt = estimate.sweep?.activeDt ?? intervalDt;
        // Merchant cutting force acts opposite the tool's cutting velocity;
        // thrust is orthogonal to it and points toward separation.  The former
        // implementation applied hypot(Fc,Ft) entirely along the cutting path,
        // incorrectly turning the no-work thrust component into extra cutting
        // work and excessive material removal.
        const candidateImpulse = estimate.model === 'plastic-edge'
          ? scalePoint(separationDirection, Math.max(0, estimate.thrustForce) * effectiveDt)
          : add(
            scalePoint(cuttingDirection, Math.max(0, estimate.cuttingForce) * effectiveDt),
            scalePoint(thrustDirection, number(estimate.thrustForce) * effectiveDt),
          );
        if (dot(candidateImpulse, separationDirection) < -1e-12) {
          materialStepValid = false;
          materialFailureReason = '材料本构给出拉开靶子的法向力；该齿面/摩擦角超出压缩接触域，本子步不删除材料';
          delete estimate.sweep;
          return;
        }
        const required = length(candidateImpulse);
        const direction = required > 1e-12 ? scalePoint(candidateImpulse, 1 / required) : cuttingDirection;
        const drivingSpeed = Math.max(0, dot(responseKinematics.relativeVelocity, direction));
        const rigidUpper = weaponTargetStoppingImpulse(physics, contact.point, direction, drivingSpeed);
        const applied = Math.min(required, rigidUpper);
        // The prediction step has already traversed the full CAD sweep. If the
        // material impulse would stop it before the interval ends, that interval
        // is physically invalid and the whole fixed tick must be rolled back and
        // re-integrated at a finer dt. Partially carving the path while keeping
        // the full ballistic pose was the mechanism that accumulated a buried
        // blade. This is a convergence test, not a fitted response coefficient.
        if (required > rigidUpper + Math.max(1e-9, rigidUpper * 1e-5)) {
          materialStepValid = false;
          materialFailureReason = `材料阻力需 ${format(required, 5)} N·s，但本子步相对动量仅允许 ${format(rigidUpper, 5)} N·s`;
          delete estimate.sweep;
          return;
        }
        const stoppingFraction = rigidUpper > 1e-12 ? applied / rigidUpper : 0;
        const resistancePath = estimate.sweep?.pathLength ?? (responseSpeed * effectiveDt);
        const pathIntegrationError = .5 * stoppingFraction * resistancePath;
        estimate.pathIntegrationError = pathIntegrationError;
        if (pathIntegrationError > MATERIAL_PATH_INTEGRATION_TOLERANCE) {
          materialStepValid = false;
          materialFailureReason = `材料阻力使本子步的未减速 CAD 扫掠多算 ${format(pathIntegrationError * 1000, 5)} mm；要求加密到不超过 ${format(MATERIAL_PATH_INTEGRATION_TOLERANCE * 1000, 5)} mm`;
          delete estimate.sweep;
          return;
        }
        impulseOnTarget = scalePoint(direction, applied);
        const kineticBeforeImpulse = weaponTargetGeneralisedKineticEnergy(physics);
        applyWeaponTargetImpulse(physics, contact.point, impulseOnTarget);
        const kineticAfterImpulse = weaponTargetGeneralisedKineticEnergy(physics);
        const impulseWork = kineticBeforeImpulse - kineticAfterImpulse;
        // Rapier's WASM body state is single precision.  This tolerance only
        // distinguishes round-off from a real energy-producing impulse; it does
        // not alter the impulse or accepted penetration.
        const energyTolerance = Math.max(1e-7, kineticBeforeImpulse * 2e-6);
        if (impulseWork < -energyTolerance) {
          materialStepValid = false;
          materialFailureReason = `材料冲量使两刚体动能增加 ${format(-impulseWork, 6)} J；本子步已拒绝并请求加密`;
          delete estimate.sweep;
          return;
        }
        const requiredWork = Math.max(0, estimate.requiredMaterialWork || 0);
        const workTolerance = Math.max(
          energyTolerance,
          Math.max(0, estimate.cuttingForce) * MATERIAL_PATH_INTEGRATION_TOLERANCE,
          requiredWork * 2e-6,
        );
        if (impulseWork + workTolerance < requiredWork) {
          materialStepValid = false;
          materialFailureReason = `当前机械耗散 ${format(Math.max(0, impulseWork), 5)} J 不足以形成该 CAD 扫掠所需的 ${format(requiredWork, 5)} J 切削功（新扫掠面积 ${format((estimate.sweep?.freshArea || 0) * 1e6, 4)} mm²，路径 ${format((estimate.sweep?.pathLength || 0) * 1000, 5)} mm，未切削厚度 ${format((estimate.uncutThickness || 0) * 1000, 5)} mm）；本子步已拒绝并请求加密`;
          delete estimate.sweep;
          return;
        }
        normalImpulse = Math.max(0, dot(impulseOnTarget, scalePoint(contact.normal, -1)));
        tangentSigned = dot(impulseOnTarget, perpendicular(scalePoint(contact.normal, -1)));
        contact.rigidImpulse = rigidUpper;
        contact.rejectedRigidImpulse = Math.max(0, rigidUpper - applied);
        if (hasFreshCut && recordMaterialCut(contact, estimate, 1, Math.max(0, impulseWork)) < 0) {
          materialStepValid = false;
          materialFailureReason = '材料去除质量不小于靶子初始运动质量';
          return;
        }
        if (postCutGeometry) {
          const removedAtPrimary = !pointTouchesMaterialGeometry(
            physicsTargetLocalPoint(contact.point, physics),
            postCutGeometry,
          );
          if (!removedAtPrimary) estimate.allowedPenetration = 0;
        }
        if (!hasFreshCut) {
          // Plastic ploughing dissipates energy but does not assert detached
          // chip volume. Permit only the entered edge-radius indentation; a
          // deeper unresolved overlap remains a hard model-domain failure.
          const work = Math.max(0, impulseWork);
          state.sim.materialStats.work += work;
          state.sim.materialStats.deformationWork = (state.sim.materialStats.deformationWork || 0) + work;
          estimate.materialWork = work;
          const enteredLimit = contact.parametric
            ? Math.min(RAPIER_ALLOWED_LINEAR_ERROR, targetThickness() * .02)
            : positive(number(state.params.edgeRadius) / 1000, .00001);
          estimate.allowedPenetration = Math.min(enteredLimit, Math.max(0, contact.penetration));
          delete estimate.sweep;
        }
      } else {
        const predictiveOnly = contact.penetration <= 0 && contact.geometricGap > GEOMETRY_CLEARANCE_EPS;
        if (predictiveOnly) {
          // Rapier's prediction distance is only a query horizon.  With the
          // material pair deliberately removed from the built-in rigid solver,
          // a still-positive CAD gap must not create either an impulse or a hit.
          // The next refined substep will resolve the actual surface crossing.
          contact.impulse = 0; contact.tangentImpulse = 0; contact.tangentImpulseSigned = 0;
          contact.suppressContactEvent = true;
          estimate.model = 'predictive-proximity'; estimate.materialWork = 0;
          estimate.validity = `CAD 表面仍有 ${format(contact.geometricGap * 1000, 5)} mm 正间隙；仅作为接近查询，不施力、不计命中或 bite`;
          delete estimate.sweep;
          contact.material = estimate;
          members.forEach((member) => {
            if (member === contact) return;
            member.impulse = 0; member.tangentImpulse = 0; member.tangentImpulseSigned = 0;
            member.suppressContactEvent = true;
            member.material = { ...estimate };
          });
          return;
        }
        // The backing body and any tooth face outside the declared X-face
        // cutting domain remain non-penetrating boundaries.  Rapier detects the
        // manifold but does not solve this pair in material mode, so apply one
        // ordinary inelastic sequential impulse instead of an infinite-stiffness
        // solve followed by a non-invertible "undo" impulse.
        const responseKinematics = rigContactKinematics(contact, physics);
        const normalDirection = scalePoint(contact.normal, -1);
        const restitution = clamp(number(state.params.restitution), 0, .25);
        // Solve normal and tangential contact together with the complete 2×2
        // Delassus matrix. Independent scalar solves ignore normal–tangent
        // coupling for an off-centre target and can add kinetic energy.
        const coupled = solveCoupledCoulombImpulse(
          physics,
          contact.point,
          normalDirection,
          responseKinematics.relativeVelocity,
          weaponTargetFriction(),
          restitution,
        );
        normalImpulse = coupled.normalImpulse;
        tangentSigned = coupled.tangentImpulse;
        impulseOnTarget = coupled.impulse;
        const kineticBeforeBoundary = weaponTargetGeneralisedKineticEnergy(physics);
        if (normalImpulse > 0 || Math.abs(tangentSigned) > 0) applyWeaponTargetImpulse(physics, contact.point, impulseOnTarget);
        const kineticAfterBoundary = weaponTargetGeneralisedKineticEnergy(physics);
        const signedBoundaryWork = kineticBeforeBoundary - kineticAfterBoundary;
        const boundaryTolerance = Math.max(
          ENERGY_LEDGER_ABSOLUTE_TOLERANCE,
          Math.max(1, kineticBeforeBoundary, kineticAfterBoundary) * ENERGY_LEDGER_RELATIVE_TOLERANCE,
        );
        if (signedBoundaryWork < -boundaryTolerance) {
          materialStepValid = false;
          materialFailureReason = `非切削边界联立冲量使机械能增加 ${format(-signedBoundaryWork, 6)} J；本子步拒绝并请求加密`;
          return;
        }
        contact.rigidImpulse = normalImpulse;
        contact.rejectedRigidImpulse = 0;
        contact.boundaryWork = Math.max(0, signedBoundaryWork);
        contact.boundarySolveMode = coupled.mode;
        estimate.model = contact.isTooth ? 'tooth-boundary' : 'rigid-backing';
        // No material volume was removed on this branch.  Keep the constitutive
        // force estimate for diagnostics, but never book F*s as plastic work.
        estimate.materialWork = 0;
        estimate.validity = contact.isTooth
          ? `${estimate.validity}；本接触未形成新的迎击面 CAD 扫掠体积，按非穿透边界求解`
          : `${estimate.validity}；非牙形刀体不解释为切削`;
        delete estimate.sweep;
      }

      contact.impulse = normalImpulse;
      contact.tangentImpulse = Math.abs(tangentSigned);
      contact.tangentImpulseSigned = tangentSigned;
      contact.material = estimate;
      members.forEach((member) => {
        if (member === contact) return;
        member.impulse = 0; member.tangentImpulse = 0; member.tangentImpulseSigned = 0;
        member.suppressContactEvent = true;
        // A grouped manifold may be a distinct patch on the same tooth/body.
        // Suppressing its duplicate event must never suppress its geometry
        // constraint.  Only a point proven to lie in the freshly removed exact
        // polygon may inherit its stale pre-rebuild penetration; otherwise the
        // ordinary domain gate forces refinement instead of hiding overlap.
        const removedByThisCut = postCutGeometry
          ? !pointTouchesMaterialGeometry(physicsTargetLocalPoint(member.point, physics), postCutGeometry)
          : false;
        member.material = {
          ...estimate,
          allowedPenetration: removedByThisCut ? Math.max(0, member.penetration) : 0,
        };
      });
    });

    if (!materialStepValid) return { ok: false, failureDomain: 'material-model', reason: materialFailureReason };
    const energyBeforeRebuild = rigEnergySnapshot();
    const rebuilt = rebuildMaterialTargetCollider(physics);
    syncRigStateFromPhysics();
    if (!rebuilt) return { ok: false, failureDomain: 'material-model', reason: '材料缺口碰撞轮廓无法重建' };
    const energyAfterRebuild = rigEnergySnapshot();
    const massRemovalEnergy = rigMechanicalEnergy(energyBeforeRebuild)
      - rigMechanicalEnergy(energyAfterRebuild);
    const energyScale = Math.max(
      1,
      Math.abs(rigMechanicalEnergy(energyBeforeRebuild)),
      Math.abs(rigMechanicalEnergy(energyAfterRebuild)),
    );
    const energyTolerance = Math.max(ENERGY_LEDGER_ABSOLUTE_TOLERANCE, energyScale * ENERGY_LEDGER_RELATIVE_TOLERANCE);
    if (massRemovalEnergy < -energyTolerance) {
      return {
        ok: false,
        failureDomain: 'material-model',
        reason: `材料质量/质心重建使剩余系统机械能增加 ${format(-massRemovalEnergy, 6)} J，超过数值账本容差 ${format(energyTolerance, 6)} J`,
      };
    }
    return { ok: true, massRemovalEnergy };
  }

  function runtimeContactMode() {
    if (state.params.contactModel === 'rigid') return 'rigid-upper-bound';
    if (state.sim?.physics?.materialCuttingEnabled) return 'traceable-cutting';
    if (state.sim?.physics?.materialResponseEnabled) return 'finite-plastic-edge';
    // Neither imported CAD nor parameter teeth may silently borrow the rigid
    // boundary when the operator selected a finite-material result.  The rig
    // will stop and roll back at the first load-bearing weapon contact.
    return 'material-model-domain-gated';
  }

  function impactFromRigEpisode(episode) {
    const before = episode.startEnergy; const after = episode.endEnergy;
    const targetKineticGain = after.targetKinetic - before.targetKinetic;
    const targetMechanicalGain = after.target - before.target;
    const rotorLoss = before.rotor - after.rotor;
    const chassisLoss = before.chassis - after.chassis;
    // The fork is an independent hinged rigid body.  It is included in every
    // per-substep mechanical-energy snapshot and constraint-exchange term, so
    // its episode energy change must also appear in the source ledger.  Leaving
    // it out made the reported "unclassified" residual algebraically equal to
    // the fork's kinetic/potential-energy gain even when Rapier conserved the
    // complete system energy.
    const forkLoss = before.fork - after.fork;
    const sourceEnergy = rotorLoss + chassisLoss + forkLoss + episode.externalWork;
    const balanceResidual = sourceEnergy - targetMechanicalGain;
    const constraintEnergyExchange = number(episode.constraintEnergyExchange);
    const massRemovalEnergy = number(episode.massRemovalEnergy);
    const namedDissipation = Math.max(0, episode.materialWork || 0)
      + Math.max(0, episode.boundaryWork || 0)
      + constraintEnergyExchange
      + massRemovalEnergy;
    const unclassifiedEnergy = balanceResidual - namedDissipation;
    const energyScale = Math.max(
      1,
      Math.abs(sourceEnergy), Math.abs(targetMechanicalGain), Math.abs(namedDissipation),
      Math.abs(before.target + before.rotor + before.chassis + before.fork),
      Math.abs(after.target + after.rotor + after.chassis + after.fork),
    );
    const energyTolerance = Math.max(
      ENERGY_LEDGER_ABSOLUTE_TOLERANCE,
      energyScale * ENERGY_LEDGER_RELATIVE_TOLERANCE,
    );
    const numericalEnergyGain = Math.max(
      0,
      -constraintEnergyExchange,
      -massRemovalEnergy,
      -unclassifiedEnergy,
    );
    const energyConverged = Boolean(episode.complete)
      && Math.abs(unclassifiedEnergy) <= energyTolerance
      && numericalEnergyGain <= energyTolerance;
    const primary = episode.primary;
    const impactKind = episode.impactKind || (episode.toothOrder === null ? 'body' : 'tooth');
    return {
      contact: primary.point,
      normal: primary.normal,
      sourceIndex: primary.index,
      colliderIndex: primary.colliderIndex,
      classificationSourceIndex: episode.classificationSourceIndex ?? primary.index,
      classificationColliderIndex: episode.classificationColliderIndex ?? primary.colliderIndex,
      penetration: episode.maxPenetration,
      contactGap: primary.geometricGap || 0,
      relativeSpeed: episode.maxRelativeSpeed,
      normalVelocity: episode.peakNormalVelocity,
      normalImpulse: episode.normalImpulse,
      tangentialImpulse: episode.tangentialImpulse,
      friction: weaponTargetFriction(),
      impulse: length(episode.impulseVector || point(0, 0)),
      impulseVector: episode.impulseVector || point(0, 0),
      collisionEnergy: sourceEnergy,
      energyBudget: null,
      coupling: null,
      rigidBody: ['rigid-upper-bound', 'no-removal-boundary', 'cutting-safety-gated'].includes(runtimeContactMode()),
      contactModel: state.params.contactModel,
      runtimeContactModel: runtimeContactMode(),
      impactKind,
      toothOrder: episode.toothOrder,
      bodyContact: impactKind === 'body',
      toothHitNumber: episode.hitNumber ?? null,
      bodyImpactNumber: episode.bodyImpactNumber ?? null,
      classificationImpulse: episode.classificationImpulse || 0,
      feedBite: episode.feedBite || 0,
      targetEnergyBefore: before.targetKinetic,
      targetEnergyAfter: after.targetKinetic,
      targetEnergyGain: targetKineticGain,
      targetMechanicalGain,
      transferredEnergy: Math.max(0, targetKineticGain),
      rotorEnergyBefore: before.rotor,
      rotorEnergyAfter: after.rotor,
      rotorEnergyLoss: rotorLoss,
      chassisEnergyLoss: chassisLoss,
      forkEnergyLoss: forkLoss,
      externalWork: episode.externalWork,
      dissipatedEnergy: Math.max(0, balanceResidual),
      namedDissipation,
      unclassifiedEnergy,
      numericalEnergyGain,
      constraintEnergyExchange,
      massRemovalEnergy,
      energyTolerance,
      energyConverged,
      materialTickConverged: episode.materialTickConverged !== false,
      materialConvergenceChecks: number(episode.materialConvergenceChecks),
      effectiveWidthZ: episode.effectiveWidthZ || 0,
      uncutThickness: episode.uncutThickness || 0,
      cuttingForce: episode.cuttingForce || 0,
      materialWork: episode.materialWork || 0,
      boundaryWork: episode.boundaryWork || 0,
      rejectedRigidImpulse: episode.rejectedRigidImpulse || 0,
      materialModel: episode.materialModels?.length
        ? (episode.materialModels.length === 1 ? episode.materialModels[0] : `mixed: ${episode.materialModels.join(' + ')}`)
        : runtimeContactMode(),
      materialValidity: episode.materialValidities?.length
        ? episode.materialValidities.join('；')
        : (runtimeContactMode() === 'rigid-upper-bound'
          ? '理想刚体上限；不代表材料损伤'
          : (runtimeContactMode() === 'no-removal-boundary'
            ? '材料切削输入不完整；采用非穿透零删料边界，不声明材料损伤量'
            : (runtimeContactMode() === 'cutting-safety-gated'
              ? '材料切削输入已给，但连续最早压缩 TOI 安全门尚未通过；采用非穿透零删料边界，不声明材料损伤量'
              : (runtimeContactMode() === 'finite-plastic-edge'
                ? '参数齿仅定义线段、齿尖半径与有限塑性接触；不删除材料、不声明切屑体积'
                : targetMaterialProperties().validity)))),
      time: episode.hitTime || episode.startTime,
      endTime: episode.endTime,
      episodeComplete: Boolean(episode.complete),
    };
  }

  function registerRigContacts(contacts, before, eventTime, externalWork = 0, intervalDt = 0, energyPhases = {}) {
    const sim = state.sim; const physics = sim.physics; const after = rigEnergySnapshot();
    const active = new Set();
    const toolContacts = contacts.filter((contact) => !contact.suppressContactEvent && (contact.role === 'fork' || contact.role === 'weapon')
      && contact.geometricGap <= RIG_CONTACT_EVENT_MAX_GAP
      && (contact.impulse > 1e-10 || contact.penetration > 0));
    const weaponContacts = toolContacts.filter((contact) => contact.role === 'weapon');
    const forkContacts = toolContacts.filter((contact) => contact.role === 'fork');
    toolContacts.forEach((contact) => active.add(contact.role === 'weapon'
      ? `weapon:${contact.isTooth ? `tooth:${contact.toothOrder}` : 'body'}`
      : 'fork'));

    if (forkContacts.length) {
      sim.forkEngaged = true; sim.targetPushedByFork = true;
      if (!sim.forkContact) {
        sim.forkContact = true;
        addEvent('叉子真实 DXF 刚体首次接触：接触冲量、地面反力和底盘减速由 Rapier 联立求解。', 'fork');
      }
    }

    let episode = sim.rigWeaponEpisode;
    if (!weaponContacts.length && episode) {
      const damage = sim.materialDamage;
      episode.separationAngularTravel = number(episode.separationAngularTravel)
        + Math.abs(before.weaponOmega) * intervalDt;
      // A physical weapon contact cannot become a new event merely because
      // Rapier's manifold disappears for one numerical substep. Teeth retain
      // through one physical tooth pitch; a body/backing scrape retains through
      // at most one revolution. Both also require the true CAD envelope to stay
      // nearby, so a launched target closes the episode immediately.
      const toothPitch = Math.PI * 2 / Math.max(1, state.metrics.cadToothCount);
      const episodePitch = episode.impactKind === 'tooth' ? toothPitch : Math.PI * 2;
      const sameWeaponEpisodeCandidate = episode.counted
        && episode.separationAngularTravel < episodePitch;
      const cutCandidate = physics.materialCuttingEnabled
        && damage?.activeTooth != null
        && damage.activeAngularTravel < Math.PI * 2;
      // Both episode-retention tests inspect the same immutable post-step pose.
      // Run the conservative broad phase at most once when either needs it.
      const weaponStillNear = (sameWeaponEpisodeCandidate || cutCandidate)
        && rigNearCadContact().nearWeapon;
      const retainSameWeaponPass = sameWeaponEpisodeCandidate && weaponStillNear;
      const retainCutPass = cutCandidate && weaponStillNear;
      if (retainCutPass || retainSameWeaponPass) {
        // The episode energy interval includes these short manifold gaps, so its
        // motor work and constraint exchange must include them as well.  Leaving
        // the gap out of the ledger produced an apparent numerical energy gain
        // after correctly merging same-tooth chatter into one physical strike.
        episode.endTime = eventTime;
        episode.endEnergy = after;
        episode.externalWork += externalWork;
        episode.constraintEnergyExchange += number(energyPhases.constraintEnergyExchange);
        episode.massRemovalEnergy += number(energyPhases.massRemovalEnergy);
      }
      if (!retainCutPass && !retainSameWeaponPass) {
      episode.complete = true;
      episode.endTime = episode.lastSeenTime;
      sim.lastImpact = impactFromRigEpisode(episode);
      if (episode.counted) {
        const contactResultLabel = runtimeContactMode() === 'rigid-upper-bound'
          ? '理想刚体上限'
          : (runtimeContactMode() === 'no-removal-boundary'
            ? '非穿透零删料边界'
            : (runtimeContactMode() === 'cutting-safety-gated'
              ? '切削安全门：非穿透零删料'
              : `${runtimeContactMode() === 'finite-plastic-edge' ? '塑性变形功' : '材料功'} ${format(sim.lastImpact.materialWork)} J`));
        const isBodyImpact = episode.impactKind === 'body';
        const eventNumber = isBodyImpact ? episode.bodyImpactNumber : episode.hitNumber;
        const eventLabel = isBodyImpact ? '刀体 / 背板碰撞' : '牙齿命中';
        addEvent(`${eventLabel} #${eventNumber} 接触结束：主 source ${sim.lastImpact.sourceIndex ?? '未知'}，episode 总法向冲量 ${format(sim.lastImpact.normalImpulse, 6)} N·s；靶动能 ${sim.lastImpact.targetEnergyGain >= 0 ? '+' : ''}${format(sim.lastImpact.targetEnergyGain)} J，转子 ${sim.lastImpact.rotorEnergyLoss >= 0 ? '损失' : '增加'} ${format(Math.abs(sim.lastImpact.rotorEnergyLoss))} J；${contactResultLabel}${isBodyImpact ? '；未计入牙齿命中或 bite' : ''}。`, isBodyImpact ? 'warning' : 'impact');
        if (!sim.lastImpact.energyConverged) addEvent(
          `${eventLabel} #${eventNumber} 的完整能量账未收敛：主 source ${sim.lastImpact.sourceIndex ?? '未知'}，episode 总法向冲量 ${format(sim.lastImpact.normalImpulse, 6)} N·s；未归类残差 ${format(sim.lastImpact.unclassifiedEnergy, 6)} J，数值增能 ${format(sim.lastImpact.numericalEnergyGain, 6)} J，容差 ${format(sim.lastImpact.energyTolerance, 6)} J；本次切削量与击飞量标记为无效，不用于理论结论。`,
          'warning',
        );
      } else if (episode.normalImpulse > 1e-8) {
        addEvent(`未分类武器接触：source ${sim.lastImpact.sourceIndex ?? '未知'}，总法向冲量 ${format(sim.lastImpact.normalImpulse, 6)} N·s；靶动能 ${sim.lastImpact.targetEnergyGain >= 0 ? '+' : ''}${format(sim.lastImpact.targetEnergyGain)} J。`, 'warning');
      }
      sim.rigWeaponEpisode = null; episode = null;
      if (sim.materialDamage) {
        sim.materialDamage.activeTooth = null;
        sim.materialDamage.activeStartTime = null;
        sim.materialDamage.lastCutTime = null;
        sim.materialDamage.activeAngularTravel = 0;
      }
      }
    }

    let episodeImpact = null;
    if (weaponContacts.length) {
      if (!episode) {
        episode = {
          startTime: eventTime - intervalDt,
          lastSeenTime: eventTime,
          endTime: eventTime,
          startEnergy: before,
          endEnergy: after,
          externalWork: 0,
          normalImpulse: 0,
          tangentialImpulse: 0,
          maxPenetration: 0,
          maxRelativeSpeed: 0,
          peakNormalVelocity: 0,
          primary: weaponContacts[0],
          primaryImpulse: -Infinity,
          impactKind: null,
          classificationContact: null,
          classificationSourceIndex: null,
          classificationColliderIndex: null,
          classificationImpulse: 0,
          toothOrder: null,
          hitNumber: null,
          bodyImpactNumber: null,
          feedBite: 0,
          effectiveWidthZ: 0,
          uncutThickness: 0,
          cuttingForce: 0,
          materialWork: 0,
          boundaryWork: 0,
          constraintEnergyExchange: 0,
          massRemovalEnergy: 0,
          impulseVector: point(0, 0),
          rejectedRigidImpulse: 0,
          materialModels: [],
          materialValidities: [],
          separationAngularTravel: 0,
          counted: false,
          complete: false,
        };
        sim.rigWeaponEpisode = episode;
      }
      if (episode) episode.separationAngularTravel = 0;
      episode.lastSeenTime = eventTime; episode.endTime = eventTime; episode.endEnergy = after; episode.externalWork += externalWork;
      episode.constraintEnergyExchange += number(energyPhases.constraintEnergyExchange);
      episode.massRemovalEnergy += number(energyPhases.massRemovalEnergy);
      weaponContacts.forEach((contact) => {
        const kinematics = rigContactKinematics(contact, physics);
        episode.normalImpulse += contact.impulse;
        episode.tangentialImpulse += contact.tangentImpulse;
        const impulseNormal = scalePoint(contact.normal, -Math.max(0, contact.impulse || 0));
        const impulseTangent = scalePoint(perpendicular(scalePoint(contact.normal, -1)), contact.tangentImpulseSigned || 0);
        episode.impulseVector = add(episode.impulseVector, add(impulseNormal, impulseTangent));
        episode.boundaryWork += Math.max(0, contact.boundaryWork || 0);
        if (contact.material) {
          episode.effectiveWidthZ = Math.max(episode.effectiveWidthZ, contact.material.width || 0);
          episode.uncutThickness = Math.max(episode.uncutThickness, contact.material.uncutThickness || 0);
          episode.cuttingForce = Math.max(episode.cuttingForce, contact.material.resultantForce || 0);
          // contact.material.materialWork is already the exact kinetic-energy
          // loss measured immediately across the custom material impulse.
          // Re-projecting only its normal component here discarded legitimate
          // tangential cutting work and made the event card disagree with the
          // material ledger.
          episode.materialWork += Math.max(0, contact.material.materialWork || 0);
          episode.rejectedRigidImpulse = Math.max(episode.rejectedRigidImpulse, Math.max(0, contact.rejectedRigidImpulse || 0));
          if (contact.material.model && !episode.materialModels.includes(contact.material.model)) episode.materialModels.push(contact.material.model);
          if (contact.material.validity && !episode.materialValidities.includes(contact.material.validity)) episode.materialValidities.push(contact.material.validity);
        }
        episode.maxPenetration = Math.max(episode.maxPenetration, contact.penetration);
        episode.maxRelativeSpeed = Math.max(episode.maxRelativeSpeed, kinematics.relativeSpeed);
        if (Math.abs(kinematics.normalVelocity) > Math.abs(episode.peakNormalVelocity)) episode.peakNormalVelocity = kinematics.normalVelocity;
        if (contact.impulse > episode.primaryImpulse) {
          episode.primaryImpulse = contact.impulse;
          episode.primary = contact;
        }
      });
      if (!episode.counted) {
        const classified = weaponContacts.reduce((best, contact) => {
          const impulseMagnitude = Math.hypot(contact.impulse || 0, contact.tangentImpulse || 0);
          if (impulseMagnitude <= 1e-8 || (best && best.impulseMagnitude >= impulseMagnitude)) return best;
          return { contact, impulseMagnitude };
        }, null);
        if (classified) {
          const contact = classified.contact;
          episode.counted = true;
          episode.hitTime = eventTime;
          episode.impactKind = contact.isTooth ? 'tooth' : 'body';
          episode.classificationContact = contact;
          episode.classificationSourceIndex = contact.index;
          episode.classificationColliderIndex = contact.colliderIndex;
          episode.classificationImpulse = classified.impulseMagnitude;
          if (episode.impactKind === 'tooth') {
            episode.toothOrder = contact.toothOrder;
            // Bite is a pre-impact kinematic value. Sampling the velocities after
            // Rapier applies the contact impulse would mix collision-induced
            // rotor/chassis slowdown into the feed that existed at first touch.
            const toothFrequency = Math.abs(before.weaponOmega) / (Math.PI * 2) * state.metrics.cadToothCount;
            const targetArm = subtract(contact.point, before.targetCom || before.targetPosition);
            const targetContactVx = before.targetVelocity.x - before.targetOmega * targetArm.y;
            const feedSpeed = Math.max(0, before.robotSpeed - targetContactVx);
            episode.feedBite = toothFrequency > 1e-9 ? feedSpeed / toothFrequency : 0;
            episode.hitNumber = ++sim.hitCount;
            sim.lastImpactTime = eventTime;
            sim.lastEventHit = episode.hitNumber;
            const contactStartLabel = runtimeContactMode() === 'rigid-upper-bound'
              ? '理想刚体牙形'
              : (runtimeContactMode() === 'no-removal-boundary'
                ? '非穿透零删料牙形'
                : (runtimeContactMode() === 'cutting-safety-gated'
                  ? '切削安全门非穿透牙形'
                  : (runtimeContactMode() === 'finite-plastic-edge' ? '参数齿有限塑性边界' : '可追溯材料牙形')));
            addEvent(`${contactStartLabel}开始牙齿命中 #${episode.hitNumber}：首次受力 source ${contact.index ?? '未知'}，当前法向冲量 ${format(contact.impulse, 6)} N·s；正在累计真实接触与能量账。`, 'impact');
          } else {
            episode.toothOrder = null;
            episode.feedBite = 0;
            episode.bodyImpactNumber = ++sim.bodyImpactCount;
            sim.lastBodyImpactTime = eventTime;
            sim.lastEventBodyImpact = episode.bodyImpactNumber;
            addEvent(`刀体 / 背板碰撞 #${episode.bodyImpactNumber} 开始：首次受力 source ${contact.index ?? '未知'}，当前法向冲量 ${format(contact.impulse, 6)} N·s；不计入牙齿命中或 bite，正在累计真实接触与能量账。`, 'warning');
          }
        }
      }
      if (episode.counted) {
        episodeImpact = impactFromRigEpisode(episode);
        sim.lastImpact = episodeImpact;
      }
    }

    const strongestTool = toolContacts.sort((a, b) => b.impulse - a.impulse)[0];
    if (strongestTool) {
      const kinematics = rigContactKinematics(strongestTool, physics);
      sim.currentContact = { ...strongestTool, ...kinematics };
      if (strongestTool.role === 'weapon' && episodeImpact) sim.currentContact.impact = episodeImpact;
    } else {
      // Ground support is telemetry, not a weapon/fork contact phase.
      sim.currentContact = null;
    }
    sim.rigActiveContacts = active;
  }

  const FAILURE_DOMAINS = Object.freeze(new Set(['rigid-solver', 'material-model', 'internal']));

  function resolvedFailureDomain(sim = state.sim) {
    if (FAILURE_DOMAINS.has(sim?.failureDomain)) return sim.failureDomain;
    if (sim?.modelDomainStopped) return 'material-model';
    if (sim?.solverDomainStopped) return 'rigid-solver';
    return null;
  }

  function annotateRigFailure(sim, failureDomain, message) {
    const previous = sim.lastRigFailure && typeof sim.lastRigFailure === 'object'
      ? sim.lastRigFailure
      : null;
    sim.lastRigFailure = {
      ...(previous || {}),
      failureDomain,
      modelDomain: failureDomain === 'material-model',
      invalidRoles: previous?.invalidRoles?.length ? [...previous.invalidRoles] : [message],
    };
  }

  function stopRigSolver(message, status = '刚体求解超出有效域') {
    const sim = state.sim;
    sim.running = false; sim.completed = true;
    sim.failureDomain = 'rigid-solver';
    sim.solverDomainStopped = true; sim.modelDomainStopped = false;
    annotateRigFailure(sim, sim.failureDomain, message);
    addEvent(message, 'warning'); updateStatus(status, 'warning');
  }

  function stopRigModelDomain(message, status = '材料模型超出有效域') {
    const sim = state.sim;
    sim.running = false; sim.completed = true;
    sim.failureDomain = 'material-model';
    sim.modelDomainStopped = true;
    // This is an intentionally refused physical claim, not a Rapier numerical
    // failure.  Keeping the flags separate lets tests and exported results tell
    // a safe model-domain terminal from solver divergence.
    sim.solverDomainStopped = false;
    annotateRigFailure(sim, sim.failureDomain, message);
    addEvent(message, 'warning'); updateStatus(status, 'warning');
  }

  function stopRigInternal(message, status = '内部异常安全停止') {
    const sim = state.sim;
    sim.running = false; sim.completed = true;
    sim.failureDomain = 'internal';
    sim.solverDomainStopped = false; sim.modelDomainStopped = false;
    annotateRigFailure(sim, sim.failureDomain, message);
    addEvent(message, 'warning'); updateStatus(status, 'warning');
  }

  function cloneRigEpisode(episode) {
    if (!episode) return null;
    return {
      ...episode,
      startEnergy: episode.startEnergy ? { ...episode.startEnergy } : episode.startEnergy,
      endEnergy: episode.endEnergy ? { ...episode.endEnergy } : episode.endEnergy,
      impulseVector: episode.impulseVector ? { ...episode.impulseVector } : episode.impulseVector,
      materialModels: [...(episode.materialModels || [])],
      materialValidities: [...(episode.materialValidities || [])],
      classificationContact: episode.classificationContact ? {
        ...episode.classificationContact,
        point: episode.classificationContact.point ? { ...episode.classificationContact.point } : episode.classificationContact.point,
        normal: episode.classificationContact.normal ? { ...episode.classificationContact.normal } : episode.classificationContact.normal,
        material: episode.classificationContact.material ? { ...episode.classificationContact.material } : episode.classificationContact.material,
      } : episode.classificationContact,
      primary: episode.primary ? {
        ...episode.primary,
        point: episode.primary.point ? { ...episode.primary.point } : episode.primary.point,
        normal: episode.primary.normal ? { ...episode.primary.normal } : episode.primary.normal,
        material: episode.primary.material ? { ...episode.primary.material } : episode.primary.material,
      } : episode.primary,
    };
  }

  function snapshotRigBookkeeping() {
    const sim = state.sim;
    return {
      driveSlip: sim.driveSlip,
      forkContact: sim.forkContact,
      forkEngaged: sim.forkEngaged,
      targetPushedByFork: sim.targetPushedByFork,
      targetLaunched: sim.targetLaunched,
      target: { ...sim.target, pos: { ...sim.target.pos }, vel: { ...sim.target.vel } },
      currentContact: sim.currentContact,
      lastImpact: sim.lastImpact,
      hitCount: sim.hitCount,
      bodyImpactCount: sim.bodyImpactCount,
      lastEventHit: sim.lastEventHit,
      lastEventBodyImpact: sim.lastEventBodyImpact,
      lastImpactTime: sim.lastImpactTime,
      lastBodyImpactTime: sim.lastBodyImpactTime,
      blockingAnnounced: sim.blockingAnnounced,
      rigActiveContacts: new Set(sim.rigActiveContacts),
      rigWeaponEpisode: cloneRigEpisode(sim.rigWeaponEpisode),
      rigMinFloorClearance: sim.rigMinFloorClearance,
      rigForkMinFloorClearance: sim.rigForkMinFloorClearance,
      rigMaxPenetration: { ...sim.rigMaxPenetration },
      rigRejectedPenetration: { ...sim.rigRejectedPenetration },
      rigJointError: sim.rigJointError,
      rigForkJointError: sim.rigForkJointError,
      rigMaxJointError: sim.rigMaxJointError,
      rigMaxForkJointError: sim.rigMaxForkJointError,
      rigSubstepsLastTick: sim.rigSubstepsLastTick,
      trail: sim.trail.map((entry) => ({ ...entry })),
      materialDamage: cloneMaterialDamage(sim.materialDamage),
      materialStats: { ...sim.materialStats },
      materialMaxIntrusion: sim.materialMaxIntrusion,
      eventCount: activeEventEntries().length,
    };
  }

  function restoreRigBookkeeping(snapshot) {
    if (!snapshot) return;
    const sim = state.sim;
    sim.driveSlip = snapshot.driveSlip;
    sim.forkContact = snapshot.forkContact;
    sim.forkEngaged = snapshot.forkEngaged;
    sim.targetPushedByFork = snapshot.targetPushedByFork;
    sim.targetLaunched = snapshot.targetLaunched;
    sim.target = { ...snapshot.target, pos: { ...snapshot.target.pos }, vel: { ...snapshot.target.vel } };
    sim.currentContact = snapshot.currentContact;
    sim.lastImpact = snapshot.lastImpact;
    sim.hitCount = snapshot.hitCount;
    sim.bodyImpactCount = snapshot.bodyImpactCount;
    sim.lastEventHit = snapshot.lastEventHit;
    sim.lastEventBodyImpact = snapshot.lastEventBodyImpact;
    sim.lastImpactTime = snapshot.lastImpactTime;
    sim.lastBodyImpactTime = snapshot.lastBodyImpactTime;
    sim.blockingAnnounced = snapshot.blockingAnnounced;
    sim.rigActiveContacts = new Set(snapshot.rigActiveContacts);
    sim.rigWeaponEpisode = cloneRigEpisode(snapshot.rigWeaponEpisode);
    sim.rigMinFloorClearance = snapshot.rigMinFloorClearance;
    sim.rigForkMinFloorClearance = snapshot.rigForkMinFloorClearance;
    sim.rigMaxPenetration = { ...snapshot.rigMaxPenetration };
    sim.rigRejectedPenetration = { ...snapshot.rigRejectedPenetration };
    sim.rigJointError = snapshot.rigJointError;
    sim.rigForkJointError = snapshot.rigForkJointError;
    sim.rigMaxJointError = snapshot.rigMaxJointError;
    sim.rigMaxForkJointError = snapshot.rigMaxForkJointError;
    sim.rigSubstepsLastTick = snapshot.rigSubstepsLastTick;
    sim.trail = snapshot.trail.map((entry) => ({ ...entry }));
    sim.materialDamage = cloneMaterialDamage(snapshot.materialDamage);
    sim.materialStats = { ...snapshot.materialStats };
    sim.materialMaxIntrusion = snapshot.materialMaxIntrusion;
    const events = activeEventEntries();
    events.length = snapshot.eventCount;
    if (!state.trajectory?.building) renderEventEntries(events);
  }

  function materialTickSignature(baseline) {
    const sim = state.sim; const physics = sim.physics;
    const targetPosition = physics.targetBody.translation();
    const energy = rigEnergySnapshot();
    const startStats = baseline?.materialStats || {};
    const startDamageVersion = baseline?.materialDamage?.version || 0;
    const removedArea = number(sim.materialStats.removedArea) - number(startStats.removedArea);
    const materialWork = number(sim.materialStats.work) - number(startStats.work);
    const deformationWork = number(sim.materialStats.deformationWork) - number(startStats.deformationWork);
    const damageVersionDelta = number(sim.materialDamage?.version) - startDamageVersion;
    const episode = sim.rigWeaponEpisode;
    return {
      hasResponse: damageVersionDelta !== 0 || Math.abs(materialWork) > 1e-12 || Math.abs(deformationWork) > 1e-12,
      damageVersionDelta,
      removedArea,
      materialWork,
      deformationWork,
      mechanicalEnergy: rigMechanicalEnergy(energy),
      targetPosition: point(targetPosition.x, targetPosition.y),
      targetAngle: physics.targetBody.rotation(),
      weaponAngle: physics.weaponBody.rotation(),
      toothOrder: episode?.toothOrder ?? sim.currentContact?.toothOrder ?? null,
      sourceIndex: episode?.primary?.index ?? sim.currentContact?.index ?? null,
    };
  }

  function compareMaterialTickSignatures(coarse, fine) {
    if (!coarse?.hasResponse || !fine?.hasResponse) return {
      converged: Boolean(coarse?.hasResponse) === Boolean(fine?.hasResponse),
      reason: '材料响应在相邻分辨率间出现/消失',
    };
    const areaScale = Math.max(Math.abs(coarse.removedArea), Math.abs(fine.removedArea), 1e-10);
    const areaTolerance = Math.max(1e-10, areaScale * MATERIAL_TICK_RELATIVE_CONVERGENCE);
    const workScale = Math.max(Math.abs(coarse.materialWork), Math.abs(fine.materialWork), 1);
    const workTolerance = Math.max(
      ENERGY_LEDGER_ABSOLUTE_TOLERANCE,
      workScale * MATERIAL_TICK_RELATIVE_CONVERGENCE,
    );
    const energyScale = Math.max(Math.abs(coarse.mechanicalEnergy), Math.abs(fine.mechanicalEnergy), 1);
    const energyTolerance = Math.max(
      ENERGY_LEDGER_ABSOLUTE_TOLERANCE,
      energyScale * MATERIAL_TICK_RELATIVE_CONVERGENCE,
    );
    const targetPoseError = length(subtract(coarse.targetPosition, fine.targetPosition));
    const weaponArcError = Math.abs(shortestAngleDelta(coarse.weaponAngle, fine.weaponAngle)) * activeWeaponRadius();
    const targetHalfDiagonal = Math.hypot(targetLength(), targetThickness()) / 2;
    const targetArcError = Math.abs(shortestAngleDelta(coarse.targetAngle, fine.targetAngle)) * targetHalfDiagonal;
    const checks = {
      identity: coarse.toothOrder === fine.toothOrder && coarse.sourceIndex === fine.sourceIndex,
      damageVersion: coarse.damageVersionDelta === fine.damageVersionDelta,
      area: Math.abs(coarse.removedArea - fine.removedArea) <= areaTolerance,
      work: Math.abs(coarse.materialWork - fine.materialWork) <= workTolerance,
      energy: Math.abs(coarse.mechanicalEnergy - fine.mechanicalEnergy) <= energyTolerance,
      pose: Math.max(targetPoseError, weaponArcError, targetArcError) <= RAPIER_ALLOWED_LINEAR_ERROR,
    };
    return {
      converged: Object.values(checks).every(Boolean),
      checks,
      areaTolerance,
      workTolerance,
      energyTolerance,
      targetPoseError,
      weaponArcError,
      targetArcError,
      reason: `相邻分辨率差：删料 ${format(Math.abs(coarse.removedArea - fine.removedArea) * 1e6, 6)} mm²、材料功 ${format(Math.abs(coarse.materialWork - fine.materialWork), 6)} J、机械能 ${format(Math.abs(coarse.mechanicalEnergy - fine.mechanicalEnergy), 6)} J、几何 ${format(Math.max(targetPoseError, weaponArcError, targetArcError) * 1000, 6)} mm`,
    };
  }


  // Work-only MLCP A/B. Generalized coordinates are
  // [chassis x, weapon angle, fork angle, target x, target y, target angle].
  // Every impulse is solved at velocity level. No position projection, skin,
  // tolerance change, empirical response scale, or feedback controller exists.
  function solveRigContactMlcpV2(physics, intervalDt, phase = 'solve') {
    // A refused material-domain step must be observationally pure: no owner,
    // counters, impulses, or telemetry mutate before the production material
    // gate performs its transactional refusal.
    if (state.params.contactModel !== 'rigid') return null;
    // The audit object belongs to the Rapier transaction. snapshotRapierRig()
    // copies it below, so rejected dyadic attempts cannot inflate the accepted
    // trajectory's contact/impulse counters.
    const stats = physics.mlcpV2Audit ||= {
      calls: 0, candidateCalls: 0, solvedCalls: 0, totalNormalImpulse: 0,
      totalFrictionImpulse: 0, contactTypes: {}, maxNormalResidual: 0,
      maxComplementarityResidual: 0, maxFrictionConeResidual: 0,
      maxKineticEnergyGain: 0, candidateTruncation: {},
      candidateTruncationFailures: 0, lastCandidateTruncationFailure: null, last: null,
    };
    window.__MlcpV2Stats = stats;
    const ownershipProbe = phase === 'probe'; const postWorldSolve = phase === 'post';
    if (!ownershipProbe) stats.calls += 1;
    const target = physics.targetBody; const robot = physics.robotBody;
    const weapon = physics.weaponBody; const fork = physics.forkBody;
    if (!target || !robot || !weapon || !fork || !(intervalDt > 0)) return null;
    // Reuse Rapier's last actual manifold point when it still lies on the same
    // collider boundary. Shape-contact support witnesses are not a manifold:
    // on parallel faces their tangential coordinates may differ arbitrarily.
    const existingManifoldContacts = collectRigContacts(physics);

    const denseDot = (left, right) => left.reduce((sum, value, index) => sum + value * right[index], 0);
    const multiply = (matrix, vector) => matrix.map((row) => denseDot(row, vector));
    const addScaled = (targetVector, sourceVector, scale) => sourceVector.forEach((value, index) => { targetVector[index] += value * scale; });
    const cross2 = (left, right) => left.x * right.y - left.y * right.x;
    const tangentOf = (normal) => point(-normal.y, normal.x);
    const pointVelocity = (body, worldPoint) => {
      const linear = body.linvel(); const com = body.worldCom?.() || body.translation(); const omega = body.angvel();
      return point(linear.x - omega * (worldPoint.y - com.y), linear.y + omega * (worldPoint.x - com.x));
    };

    const targetCom = target.worldCom?.() || target.translation();
    const weaponPivot = weapon.translation(); const forkPivot = fork.translation();
    const forkLocalCom = fork.localCom?.() || point(0, 0);
    const forkComOffset = rotate(point(forkLocalCom.x, forkLocalCom.y), fork.rotation());
    const robotMass = positive(state.params.robotMass, .01);
    const weaponInertia = Math.max(weapon.principalInertia?.() || state.metrics.inertia, 1e-12);
    const forkMass = Math.max(fork.mass?.() || 0, 1e-12);
    const forkComInertia = Math.max(fork.principalInertia?.() || 0, 1e-12);
    const forkPivotInertia = forkComInertia + forkMass * dot(forkComOffset, forkComOffset);
    const coupling = -forkMass * forkComOffset.y;
    const determinant = robotMass * forkPivotInertia - coupling * coupling;
    if (!(determinant > 1e-16)) return null;
    const targetMass = Math.max(target.mass?.() || effectiveTargetMass(), 1e-12);
    const targetMoment = Math.max(target.principalInertia?.() || targetInertia(), 1e-12);

    const inverseMass = Array.from({ length: 6 }, () => Array(6).fill(0));
    inverseMass[0][0] = forkPivotInertia / determinant;
    inverseMass[0][2] = inverseMass[2][0] = -coupling / determinant;
    inverseMass[2][2] = robotMass / determinant;
    inverseMass[1][1] = 1 / weaponInertia;
    inverseMass[3][3] = inverseMass[4][4] = 1 / targetMass;
    inverseMass[5][5] = 1 / targetMoment;

    const qdot = [robot.linvel().x, weapon.angvel(), fork.angvel(), target.linvel().x, target.linvel().y, target.angvel()];
    // The MLCP must see the same free velocity that Rapier is about to see.
    // Symplectic force integration over this interval gives these generalized
    // increments; this is deterministic mechanics, not a response gain.
    const drive = physicalDriveForce(qdot[0], intervalDt);
    const motorTorque = weaponMotorTorqueAt(qdot[1]);
    const generalizedForce = [drive.applied, motorTorque, -forkMass * GRAVITY * forkComOffset.x, 0, -targetMass * GRAVITY, 0];
    const qFree = postWorldSolve
      ? qdot.slice()
      : qdot.map((value, index) => value + multiply(inverseMass, generalizedForce)[index] * intervalDt);
    // In owner mode contact pairs are removed from Rapier for the whole
    // substep. A zero-impulse feasible state must therefore still be returned
    // so the caller suppresses Rapier contact ownership consistently.
    const ownerWasActive = Boolean(physics.mlcpV2ClusterActive);

    const makeToolRow = (role, toolPoint, targetPoint, direction) => {
      const targetArm = subtract(targetPoint, point(targetCom.x, targetCom.y));
      if (role === 'weapon-target') {
        return [direction.x, cross2(subtract(toolPoint, point(weaponPivot.x, weaponPivot.y)), direction), 0,
          -direction.x, -direction.y, -cross2(targetArm, direction)];
      }
      return [direction.x, 0, cross2(subtract(toolPoint, point(forkPivot.x, forkPivot.y)), direction),
        -direction.x, -direction.y, -cross2(targetArm, direction)];
    };
    const makeTargetFloorRow = (worldPoint, direction) => {
      const targetArm = subtract(worldPoint, point(targetCom.x, targetCom.y));
      return [0, 0, 0, direction.x, direction.y, cross2(targetArm, direction)];
    };
    const makeForkFloorRow = (worldPoint, direction) => [
      direction.x, 0, cross2(subtract(worldPoint, point(forkPivot.x, forkPivot.y)), direction), 0, 0, 0,
    ];

    // Rapier's zero-thickness segment query is Float32. Near the unchanged
    // 80 um exact gate it can quantise an endpoint depth for several substeps.
    // Derive the parameter-fork witness from the same double-precision
    // segment-vs-OBB geometry used by exactCadLoopTargetPenetration instead.
    const exactParameterForkWitness = () => {
      const loop = physics.forkLoop || [];
      if (!state.params.paramForkEnabled || loop.length !== 2) return null;
      const forkOriginRaw = fork.translation(); const forkOrigin = point(forkOriginRaw.x, forkOriginRaw.y);
      const targetOriginRaw = target.translation(); const targetOrigin = point(targetOriginRaw.x, targetOriginRaw.y);
      const forkAngle = fork.rotation(); const targetAngle = target.rotation();
      const worldEnds = loop.map((local) => add(forkOrigin, rotate(local, forkAngle)));
      const localEnds = worldEnds.map((world) => rotate(subtract(world, targetOrigin), -targetAngle));
      const start = localEnds[0]; const delta = subtract(localEnds[1], start);
      const halfX = targetLength() / 2; const halfY = targetThickness() / 2;
      let enter = 0; let leave = 1;
      const clipAxis = (origin, velocity, lower, upper) => {
        if (Math.abs(velocity) <= 1e-16) return origin >= lower && origin <= upper;
        let first = (lower - origin) / velocity; let last = (upper - origin) / velocity;
        if (first > last) [first, last] = [last, first];
        enter = Math.max(enter, first); leave = Math.min(leave, last);
        return enter <= leave;
      };
      if (!clipAxis(start.x, delta.x, -halfX, halfX) || !clipAxis(start.y, delta.y, -halfY, halfY)) return null;
      enter = clamp(enter, 0, 1); leave = clamp(leave, 0, 1);
      if (leave < enter) return null;
      const faces = [
        { constant: halfX - start.x, slope: -delta.x, normal: point(1, 0) },
        { constant: halfX + start.x, slope: delta.x, normal: point(-1, 0) },
        { constant: halfY - start.y, slope: -delta.y, normal: point(0, 1) },
        { constant: halfY + start.y, slope: delta.y, normal: point(0, -1) },
      ];
      const fractions = [enter, leave];
      for (let left = 0; left < faces.length; left += 1) for (let right = left + 1; right < faces.length; right += 1) {
        const denominator = faces[left].slope - faces[right].slope;
        if (Math.abs(denominator) <= 1e-16) continue;
        const fraction = (faces[right].constant - faces[left].constant) / denominator;
        if (fraction >= enter && fraction <= leave) fractions.push(fraction);
      }
      let best = null;
      fractions.forEach((fraction) => {
        const depths = faces.map((face) => face.constant + face.slope * fraction);
        const depth = Math.min(...depths); const faceIndex = depths.indexOf(depth);
        if (!best || depth > best.depth) best = { depth, fraction, face: faces[faceIndex] };
      });
      if (!best || !(best.depth > 0)) return null;
      const toolPoint = lerp(worldEnds[0], worldEnds[1], best.fraction);
      const normal = rotate(best.face.normal, targetAngle);
      return {
        distance: -best.depth,
        point1: add(toolPoint, scalePoint(normal, best.depth)), point2: toolPoint,
        normal1: normal, normal2: scalePoint(normal, -1), exactParameterFork: true,
      };
    };

    const candidates = []; const toolObservations = []; let candidateTruncationFailure = null;
    const closestToolBoundaryPoint = (role, collider, sample) => {
      const body = role === 'weapon-target' ? weapon : fork;
      const metadata = role === 'weapon-target'
        ? physics.weaponColliderMeta?.get(collider.handle)
        : physics.forkColliderMeta?.get(collider.handle);
      const loop = metadata?.loop || (role === 'weapon-target' ? physics.weaponLoop : physics.forkLoop) || [];
      if (loop.length < 2) return { point: sample, distance: Infinity };
      const originRaw = body.translation(); const origin = point(originRaw.x, originRaw.y);
      const localSample = rotate(subtract(sample, origin), -body.rotation());
      let closest = null; let minimumSq = Infinity;
      const edgeCount = loop.length === 2 ? 1 : loop.length;
      for (let edgeIndex = 0; edgeIndex < edgeCount; edgeIndex += 1) {
        const start = loop[edgeIndex]; const end = loop[(edgeIndex + 1) % loop.length];
        const edge = subtract(end, start); const edgeSq = dot(edge, edge);
        const fraction = edgeSq > 1e-24 ? clamp(dot(subtract(localSample, start), edge) / edgeSq, 0, 1) : 0;
        const projected = add(start, scalePoint(edge, fraction)); const delta = subtract(projected, localSample);
        const distanceSq = dot(delta, delta);
        if (distanceSq < minimumSq) { minimumSq = distanceSq; closest = projected; }
      }
      return {
        point: add(origin, rotate(closest || localSample, body.rotation())),
        distance: Math.sqrt(minimumSq),
      };
    };
    const deduplicate = (entries, role, maximum = 2) => {
      const unique = []; let equivalentDropped = 0;
      entries.sort((left, right) => left.gap - right.gap);
      for (const entry of entries) {
        const duplicate = unique.some((other) => Math.hypot(entry.point.x - other.point.x, entry.point.y - other.point.y) <= GEOMETRY_CLEARANCE_EPS
          && dot(entry.normal, other.normal) >= 1 - 1e-10
          && Math.max(...entry.normalRow.map((value, axis) => Math.abs(value - other.normalRow[axis]))) <= GEOMETRY_CLEARANCE_EPS
          && Math.max(...entry.tangentRow.map((value, axis) => Math.abs(value - other.tangentRow[axis]))) <= GEOMETRY_CLEARANCE_EPS);
        if (duplicate) equivalentDropped += 1;
        else unique.push(entry);
      }
      const kept = unique.slice(0, maximum); const capDropped = unique.slice(maximum);
      const active = (entry) => entry.gap <= 0 || entry.crossing
        || entry.gap + entry.freeClosing * intervalDt <= 0;
      const activeCapDropped = capDropped.filter(active);
      const phaseKey = postWorldSolve ? 'post' : 'probe';
      const byRole = stats.candidateTruncation[role] ||= {};
      const bucket = byRole[phaseKey] ||= {
        calls: 0, rawCount: 0, uniqueCount: 0, keptCount: 0,
        equivalentDropped: 0, capDropped: 0, activeCapDropped: 0,
        maxRawCount: 0, maxUniqueCount: 0, maxKeptCount: 0,
        maxCapDropped: 0, maxActiveCapDropped: 0,
        maxDroppedPenetration: 0, maxDroppedPredictedClosing: 0,
        maxDroppedRequiredStoppingImpulse: 0,
      };
      bucket.calls += 1; bucket.rawCount += entries.length; bucket.uniqueCount += unique.length;
      bucket.keptCount += kept.length; bucket.equivalentDropped += equivalentDropped;
      bucket.capDropped += capDropped.length; bucket.activeCapDropped += activeCapDropped.length;
      bucket.maxRawCount = Math.max(bucket.maxRawCount, entries.length);
      bucket.maxUniqueCount = Math.max(bucket.maxUniqueCount, unique.length);
      bucket.maxKeptCount = Math.max(bucket.maxKeptCount, kept.length);
      bucket.maxCapDropped = Math.max(bucket.maxCapDropped, capDropped.length);
      bucket.maxActiveCapDropped = Math.max(bucket.maxActiveCapDropped, activeCapDropped.length);
      capDropped.forEach((entry) => {
        const endpointClosing = Math.max(0, -(entry.gap + entry.freeClosing * intervalDt));
        const effectiveInverseMass = denseDot(entry.normalRow, multiply(inverseMass, entry.normalRow));
        const complementarityClosing = Math.max(0, -(entry.freeClosing + Math.max(0, entry.gap) / intervalDt));
        bucket.maxDroppedPenetration = Math.max(bucket.maxDroppedPenetration, Math.max(0, -entry.gap));
        bucket.maxDroppedPredictedClosing = Math.max(bucket.maxDroppedPredictedClosing, endpointClosing);
        bucket.maxDroppedRequiredStoppingImpulse = Math.max(bucket.maxDroppedRequiredStoppingImpulse,
          effectiveInverseMass > 1e-16 ? complementarityClosing / effectiveInverseMass : Infinity);
      });
      if (activeCapDropped.length && !candidateTruncationFailure) {
        const first = activeCapDropped[0];
        candidateTruncationFailure = {
          role, rawCount: entries.length, uniqueCount: unique.length, keptCount: kept.length,
          droppedCount: capDropped.length, activeDroppedCount: activeCapDropped.length,
          gap: first.gap, freeClosing: first.freeClosing,
          point: first.point, normal: first.normal,
        };
        stats.candidateTruncationFailures += 1;
        stats.lastCandidateTruncationFailure = candidateTruncationFailure;
      }
      return kept.map((entry, index) => ({ ...entry, key: entry.key || (role + ':' + index) }));
    };
    const gatherTool = (role, colliders, friction) => {
      const entries = [];
      const toolBody = role === 'weapon-target' ? weapon : fork;
      for (const collider of colliders || []) {
        let contact = null;
        try { contact = physics.targetCollider.contactCollider?.(collider, RAPIER_PREDICTION_DISTANCE); } catch (_) { contact = null; }
        if (role === 'fork-target' && state.params.paramForkEnabled) contact = exactParameterForkWitness() || contact;
        if (!contact || !Number.isFinite(contact.distance) || !contact.point1 || !contact.point2 || !contact.normal1) continue;
        if (role === 'fork-target' && !state.params.paramForkEnabled) {
          const metadata = physics.forkColliderMeta?.get(collider.handle);
          const loop = metadata?.loop || physics.forkLoop || [];
          const toolWorld = point(contact.point2.x, contact.point2.y);
          const forkPosition = fork.translation();
          const toolLocal = rotate(subtract(toolWorld, point(forkPosition.x, forkPosition.y)), -fork.rotation());
          const triangleIndexes = metadata?.triangleVertexIndexes || [];
          const triangleLoop = metadata?.triangleLoop || [];
          const onOuterBoundary = triangleIndexes.some((startIndex, edgeIndex) => {
            const endIndex = triangleIndexes[(edgeIndex + 1) % triangleIndexes.length];
            const delta = Math.abs(startIndex - endIndex);
            if (!(delta === 1 || delta === loop.length - 1)) return false;
            return pointSegmentDistance(toolLocal, triangleLoop[edgeIndex], triangleLoop[(edgeIndex + 1) % triangleLoop.length]) <= GEOMETRY_CLEARANCE_EPS;
          });
          // Earcut diagonals are bookkeeping seams, not fork surfaces.
          if (!onOuterBoundary) continue;
        }
        const normal = normalise(point(contact.normal1.x, contact.normal1.y));
        const targetPoint = point(contact.point1.x, contact.point1.y); const toolPoint = point(contact.point2.x, contact.point2.y);
        // Prefer a current same-collider solver-manifold point, validated against
        // the actual outer boundary. Without one, project the target support
        // witness to the closest real tool edge. Both bodies receive the impulse
        // at this one common point, so internal normal/friction impulses create
        // neither a fictitious net force nor a depth-by-impulse couple.
        const expectedRole = role === 'weapon-target' ? 'weapon' : 'fork';
        const manifoldWitnesses = existingManifoldContacts
          .filter((entry) => entry.role === expectedRole && entry.other?.handle === collider.handle && entry.point && entry.hasSolverContact)
          .map((entry) => ({ entry, boundary: closestToolBoundaryPoint(role, collider, entry.point) }))
          .sort((left, right) => left.boundary.distance - right.boundary.distance);
        const validManifold = manifoldWitnesses.find((entry) => entry.boundary.distance <= RAPIER_ALLOWED_LINEAR_ERROR);
        const boundaryFallback = closestToolBoundaryPoint(role, collider, targetPoint);
        const worldPoint = contact.exactParameterFork
          ? toolPoint
          : (validManifold ? point(validManifold.entry.point.x, validManifold.entry.point.y) : boundaryFallback.point);
        const normalRow = makeToolRow(role, worldPoint, worldPoint, normal);
        const tangentRow = makeToolRow(role, worldPoint, worldPoint, tangentOf(normal));
        const freeClosing = denseDot(normalRow, qFree);
        entries.push({ role, gap: contact.distance, point: worldPoint, targetPoint, toolPoint,
          normal, friction, normalRow, tangentRow, freeClosing, collider,
          crossing: contact.distance > 0 && contact.distance + freeClosing * intervalDt <= 0 });
      }
      // At most the two independent endpoints of a 2-D contact patch are
      // required. Keeping two (rather than one deepest point) preserves
      // load-bearing torque for CAD faces and adjacent parameter teeth while
      // bounding exhaustive Coulomb mode enumeration.
      toolObservations.push(...deduplicate(entries, role, 2));
    };
    // CAD solids are triangulated only for Rapier. Their internal Earcut
    // diagonals are not physical faces, and the current metadata does not yet
    // expose a common exact outer-boundary witness for this MLCP. Restrict the
    // structural override to the explicit single parametric tooth; CAD weapon
    // collisions remain wholly owned by Rapier and the unchanged exact gate.
    if (state.params.paramWeaponEnabled) {
      gatherTool('weapon-target', physics.weaponColliders, weaponTargetFriction());
    }
    if (effectiveToolWidthZ('fork') > 1e-9) gatherTool('fork-target', physics.forkColliders, shovelTargetFriction());

    // Solver ownership is geometric, not an event debounce. A new positive-gap
    // onset is left to Rapier/CCD (and its restitution). Once a real parameter
    // tooth overlap starts the cluster, ownership persists while either tool is
    // actually load-bearing. If all tool witnesses are positive again, Rapier
    // owns the next onset; there is no masked, unconstrained blind substep.
    const weaponObservations = toolObservations.filter((entry) => entry.role === 'weapon-target');
    const weaponContactNow = weaponObservations.some((entry) => entry.gap <= 0);
    const weaponNear = weaponObservations.some((entry) => entry.gap <= RAPIER_PREDICTION_DISTANCE);
    const weaponCrossing = weaponObservations.some((entry) => entry.crossing);
    const forkObservations = toolObservations.filter((entry) => entry.role === 'fork-target');
    const forkContactNow = forkObservations.some((entry) => entry.gap <= 0);
    const forkNear = forkObservations.some((entry) => entry.gap <= RAPIER_PREDICTION_DISTANCE);
    const forkCrossing = forkObservations.some((entry) => entry.crossing);
    const toolSweptNow = toolObservations.some((entry) => entry.gap <= 0 || entry.crossing);
    if (!postWorldSolve) {
      const sameHandoffTick = Number.isFinite(physics.mlcpV2HandoffTick)
        && Math.abs(physics.mlcpV2HandoffTick - state.sim.time) <= FIXED_DT * 1e-6;
      if (Number.isFinite(physics.mlcpV2HandoffTick) && !sameHandoffTick) physics.mlcpV2HandoffTick = null;
      if (weaponContactNow || (state.params.paramWeaponEnabled && forkContactNow)) {
        physics.mlcpV2ClusterActive = true;
        physics.mlcpV2HandoffTick = null;
      } else if (sameHandoffTick) {
        return { ownerStep: true, probe: ownershipProbe, fullCluster: false,
          contactEntries: [], auditRows: [], handoff: true };
      }
      if (!physics.mlcpV2ClusterActive) return null;
      // Release decides ownership for the next physical substep. This handoff
      // barrier still masks the old tool manifold, but leaves floor support to
      // Rapier because there is no contact KKT solve in a handoff substep.
      if (!weaponNear && !weaponCrossing && !forkNear && !forkCrossing) {
        physics.mlcpV2ClusterActive = false;
        physics.mlcpV2HandoffTick = state.sim.time;
        return { ownerStep: true, probe: ownershipProbe, fullCluster: false,
          contactEntries: [], auditRows: [], handoff: true };
      }
      if (!ownerWasActive && toolObservations.some((entry) => entry.crossing)) return null;
      if (candidateTruncationFailure) return {
        ownerStep: true, fullCluster: true,
        failure: '持久多接触 MLCP 的 ' + candidateTruncationFailure.role + ' 同时存在 '
          + candidateTruncationFailure.activeDroppedCount + ' 个未纳入的独立有效约束',
        candidateTruncationFailure, contactEntries: [], auditRows: [],
        types: toolObservations.map((entry) => entry.role),
      };
      if (!toolSweptNow) return { ownerStep: true, probe: ownershipProbe, fullCluster: true,
        contactEntries: [], auditRows: [] };
    }
    if (postWorldSolve && candidateTruncationFailure) return {
      ownerStep: true, fullCluster: true,
      failure: '持久多接触 MLCP 的 ' + candidateTruncationFailure.role + ' 同时存在 '
        + candidateTruncationFailure.activeDroppedCount + ' 个未纳入的独立有效约束',
      candidateTruncationFailure, contactEntries: [], auditRows: [],
      types: toolObservations.map((entry) => entry.role),
    };
    toolObservations.filter((entry) => entry.gap <= 0 || entry.crossing).forEach((entry) => candidates.push(entry));

    // Floor rows share the same reduced solve while a tool cluster is owned.
    // Candidate activation uses endpoint non-penetration (Moreau g/dt) and the
    // exact geometric plane; it does not alter the 80 um acceptance gate.
    const targetHalfX = targetLength() / 2; const targetHalfY = targetThickness() / 2;
    const targetOrigin = target.translation(); const targetAngle = target.rotation();
    const targetFloorEntries = [point(-targetHalfX, -targetHalfY), point(targetHalfX, -targetHalfY),
      point(-targetHalfX, targetHalfY), point(targetHalfX, targetHalfY)].map((local, index) => {
      const worldPoint = add(point(targetOrigin.x, targetOrigin.y), rotate(local, targetAngle)); const normal = point(0, 1);
      const row = makeTargetFloorRow(worldPoint, normal); const gap = worldPoint.y - targetSupportY();
      return { role: 'target-floor', key: 'target-floor:' + index, gap, point: worldPoint, normal,
        friction: targetFloorFriction().kinetic, normalRow: row,
        tangentRow: makeTargetFloorRow(worldPoint, tangentOf(normal)), freeClosing: denseDot(row, qFree) };
    }).filter((entry) => entry.gap <= 0 || entry.gap + entry.freeClosing * intervalDt <= 0);
    deduplicate(targetFloorEntries, 'target-floor', 2).forEach((entry) => candidates.push(entry));

    const forkOrigin = fork.translation(); const forkAngle = fork.rotation();
    const forkFloorEntries = (physics.forkLoop || []).map((local, index) => {
      const worldPoint = add(point(forkOrigin.x, forkOrigin.y), rotate(local, forkAngle)); const normal = point(0, 1);
      const row = makeForkFloorRow(worldPoint, normal); const gap = worldPoint.y - groundY();
      return { role: 'fork-floor', key: 'fork-floor:' + index, gap, point: worldPoint, normal,
        friction: shovelTargetFriction(), normalRow: row,
        tangentRow: makeForkFloorRow(worldPoint, tangentOf(normal)), freeClosing: denseDot(row, qFree) };
    }).filter((entry) => entry.gap <= 0 || entry.gap + entry.freeClosing * intervalDt <= 0);
    deduplicate(forkFloorEntries, 'fork-floor', 2).forEach((entry) => candidates.push(entry));
    if (candidateTruncationFailure) return {
      ownerStep: true, fullCluster: true,
      failure: '持久多接触 MLCP 的 ' + candidateTruncationFailure.role + ' 同时存在 '
        + candidateTruncationFailure.activeDroppedCount + ' 个未纳入的独立有效约束',
      candidateTruncationFailure, contactEntries: [], auditRows: [],
      types: candidates.map((entry) => entry.role),
    };
    if (!candidates.length) return postWorldSolve
      ? { ownerStep: true, fullCluster: true, contactEntries: [], auditRows: [], postWorldSolve: true }
      : null;
    if (ownershipProbe) return {
      ownerStep: true, probe: true, fullCluster: true, contactEntries: [],
      auditRows: candidates.map((entry) => ({ role: entry.role })),
      types: candidates.map((entry) => entry.role),
    };
    stats.candidateCalls += 1;

    const solveDense = (matrix, rhs) => {
      const count = rhs.length; if (!count) return [];
      const augmented = matrix.map((row, index) => [...row, rhs[index]]);
      for (let column = 0; column < count; column += 1) {
        let pivot = column;
        for (let row = column + 1; row < count; row += 1) if (Math.abs(augmented[row][column]) > Math.abs(augmented[pivot][column])) pivot = row;
        if (Math.abs(augmented[pivot][column]) <= 1e-13) return null;
        [augmented[column], augmented[pivot]] = [augmented[pivot], augmented[column]];
        const scale = augmented[column][column];
        for (let index = column; index <= count; index += 1) augmented[column][index] /= scale;
        for (let row = 0; row < count; row += 1) if (row !== column) {
          const factor = augmented[row][column];
          for (let index = column; index <= count; index += 1) augmented[row][index] -= factor * augmented[column][index];
        }
      }
      return augmented.map((row) => row[count]);
    };

    const normalRows = candidates.map((entry) => entry.normalRow);
    const tangentRows = candidates.map((entry) => entry.tangentRow);
    const minvNormal = normalRows.map((row) => multiply(inverseMass, row));
    const minvTangent = tangentRows.map((row) => multiply(inverseMass, row));
    const freeNormal = normalRows.map((row) => denseDot(row, qFree));
    // Moreau time-stepping: a positive exact gap may close during the
    // interval, but the admissible endpoint speed is -g/dt, not zero. This
    // avoids a virtual expanded tool. Existing overlap gets no positive bias
    // (and therefore no energy-injecting position correction).
    const normalBias = candidates.map((entry) => Math.max(0, entry.gap) / intervalDt);
    const freeComplementarity = freeNormal.map((value, index) => value + normalBias[index]);
    const freeTangent = tangentRows.map((row) => denseDot(row, qFree));
    const count = candidates.length;
    const massMatrixColumns = Array.from({ length: 6 }, (_, column) => {
      const basis = Array(6).fill(0); basis[column] = 1; return solveDense(inverseMass, basis);
    });
    const massMatrix = Array.from({ length: 6 }, (_, row) => massMatrixColumns.map((column) => column?.[row] || 0));
    let best = null;

    // Enumerate normal inactive/active and friction stick/+slide/-slide modes.
    // At most two reduced witnesses are kept per physical pair, so the normal
    // active set remains small and deterministic.
    for (let normalMask = 0; normalMask < (1 << count); normalMask += 1) {
      const activeNormals = Array.from({ length: count }, (_, index) => index).filter((index) => normalMask & (1 << index));
      const frictionModes = Array(count).fill(0);
      const recurseFriction = (offset) => {
        if (offset < activeNormals.length) {
          const index = activeNormals[offset];
          for (const mode of [0, 1, -1]) { frictionModes[index] = mode; recurseFriction(offset + 1); }
          return;
        }
        const unknownRows = []; const rhs = []; const normalUnknown = new Map(); const stickUnknown = new Map();
        activeNormals.forEach((index) => {
          const mode = frictionModes[index]; const effective = normalRows[index].map((value, axis) => value + mode * candidates[index].friction * tangentRows[index][axis]);
          normalUnknown.set(index, unknownRows.length); unknownRows.push(effective); rhs.push(-freeComplementarity[index]);
        });
        activeNormals.filter((index) => frictionModes[index] === 0).forEach((index) => {
          stickUnknown.set(index, unknownRows.length); unknownRows.push(tangentRows[index]); rhs.push(-freeTangent[index]);
        });
        const equations = [];
        activeNormals.forEach((index) => equations.push(normalRows[index]));
        activeNormals.filter((index) => frictionModes[index] === 0).forEach((index) => equations.push(tangentRows[index]));
        const matrix = equations.map((equation) => unknownRows.map((unknown) => denseDot(equation, multiply(inverseMass, unknown))));
        const values = solveDense(matrix, rhs); if (!values) return;
        const lambdaN = Array(count).fill(0); const lambdaT = Array(count).fill(0);
        activeNormals.forEach((index) => {
          lambdaN[index] = values[normalUnknown.get(index)];
          lambdaT[index] = frictionModes[index] === 0 ? values[stickUnknown.get(index)] : frictionModes[index] * candidates[index].friction * lambdaN[index];
        });
        const impulse = Array(6).fill(0);
        candidates.forEach((entry, index) => { addScaled(impulse, entry.normalRow, lambdaN[index]); addScaled(impulse, entry.tangentRow, lambdaT[index]); });
        const qPost = qFree.map((value, index) => value + multiply(inverseMass, impulse)[index]);
        const postNormal = normalRows.map((row) => denseDot(row, qPost)); const postTangent = tangentRows.map((row) => denseDot(row, qPost));
        const postComplementarity = postNormal.map((value, index) => value + normalBias[index]);
        const feasibleNormal = lambdaN.every((value) => value >= -1e-9) && postComplementarity.every((value) => value >= -1e-8);
        const feasibleComplementarity = lambdaN.every((value, index) => value <= 1e-9 || Math.abs(postComplementarity[index]) <= 1e-6);
        const feasibleFriction = candidates.every((entry, index) => Math.abs(lambdaT[index]) <= entry.friction * Math.max(0, lambdaN[index]) + 1e-9)
          && activeNormals.every((index) => frictionModes[index] === 0
            ? Math.abs(postTangent[index]) <= 1e-6
            : frictionModes[index] * postTangent[index] <= 1e-7);
        if (!feasibleNormal || !feasibleComplementarity || !feasibleFriction) return;
        const impulseNorm = Math.hypot(...lambdaN, ...lambdaT);
        // Coulomb's maximum-dissipation selection: among feasible contact
        // modes choose the least post-impulse generalized kinetic energy;
        // impulse norm is only a deterministic tie-breaker.
        const postMetric = denseDot(qPost, multiply(massMatrix, qPost));
        if (!best || postMetric < best.postMetric - 1e-12
          || (Math.abs(postMetric - best.postMetric) <= 1e-12 && impulseNorm < best.impulseNorm)) {
          best = { lambdaN, lambdaT, postNormal, postComplementarity, postTangent, qPost, impulseNorm, postMetric };
        }
      };
      recurseFriction(0);
    }
    if (!best) {
      stats.last = { types: candidates.map((entry) => entry.role), freeNormal, freeTangent, solution: best };
      // Once this solver owns a physical episode, infeasibility is a transaction
      // failure, never permission to unmask a stale Rapier manifold. The caller
      // rejects/refines this attempt before world.step and ultimately reports a
      // solver-domain stop if no finite active set exists.
      if (ownerWasActive) return {
        ownerStep: true, failure: '持久多接触 MLCP 无可行互补解',
        contactEntries: [], auditRows: [], types: candidates.map((entry) => entry.role),
      };
      physics.mlcpV2ClusterActive = false;
      return null;
    }
    if (!ownerWasActive && !best.lambdaN.some((value) => value > 1e-10)) {
      stats.last = { types: candidates.map((entry) => entry.role), freeNormal, freeTangent, solution: best };
      physics.mlcpV2ClusterActive = false;
      return null;
    }

    const impulseOnlyPre = qFree; const impulseOnlyPost = best.qPost;
    // Compute kinetic energy directly from physical bodies/reduced coordinate M.
    const preEnergy = .5 * denseDot(impulseOnlyPre, multiply(massMatrix, impulseOnlyPre));
    const postEnergy = .5 * denseDot(impulseOnlyPost, multiply(massMatrix, impulseOnlyPost));

    // Rapier integrates Qext below, so write q + Δq_contact, not qFree +
    // Δq_contact. Otherwise motor/gravity would be integrated twice.
    const contactDelta = best.qPost.map((value, index) => value - qFree[index]);
    const qApplied = qdot.map((value, index) => value + contactDelta[index]);
    robot.setLinvel({ x: qApplied[0], y: 0 }, true);
    weapon.setLinvel({ x: qApplied[0], y: 0 }, true); weapon.setAngvel(qApplied[1], true);
    fork.setLinvel({ x: qApplied[0] - qApplied[2] * forkComOffset.y, y: qApplied[2] * forkComOffset.x }, true); fork.setAngvel(qApplied[2], true);
    target.setLinvel({ x: qApplied[3], y: qApplied[4] }, true); target.setAngvel(qApplied[5], true);
    stats.solvedCalls += 1;
    candidates.forEach((entry, index) => {
      if (best.lambdaN[index] <= 1e-10) return;
      stats.contactTypes[entry.role] = (stats.contactTypes[entry.role] || 0) + 1;
      stats.totalNormalImpulse += best.lambdaN[index]; stats.totalFrictionImpulse += Math.abs(best.lambdaT[index]);
      stats.maxNormalResidual = Math.max(stats.maxNormalResidual, Math.max(0, -best.postComplementarity[index]));
      stats.maxComplementarityResidual = Math.max(stats.maxComplementarityResidual, Math.abs(best.lambdaN[index] * best.postComplementarity[index]));
      stats.maxFrictionConeResidual = Math.max(stats.maxFrictionConeResidual, Math.max(0, Math.abs(best.lambdaT[index]) - entry.friction * best.lambdaN[index]));
    });
    stats.maxKineticEnergyGain = Math.max(stats.maxKineticEnergyGain, Math.max(0, postEnergy - preEnergy));
    const contactEntries = candidates.map((entry, index) => {
      const impulse = Math.max(0, best.lambdaN[index]);
      // A separating zero-impulse overlap is still the same physical contact
      // episode. Preserve its identity/penetration telemetry; the production
      // event classifier still requires >1e-8 N*s before counting a hit.
      if (impulse <= 1e-10 && entry.gap >= 0) return null;
      const role = entry.role === 'weapon-target' ? 'weapon'
        : entry.role === 'fork-target' ? 'fork'
          : entry.role === 'target-floor' ? 'floor' : 'forkFloor';
      const feature = role === 'weapon'
        ? classifyWeaponContact(entry.point, physics, entry.collider)
        : { index: 0, toothOrder: null, isTooth: false };
      return {
        role, other: entry.collider || (role === 'forkFloor' ? physics.forkGroundCollider : physics.floorCollider), point: entry.point, normal: entry.normal,
        impulse, tangentImpulse: Math.abs(best.lambdaT[index]), tangentImpulseSigned: best.lambdaT[index],
        penetration: Math.max(0, -entry.gap), geometricGap: Math.max(0, entry.gap),
        solverPenetration: Math.max(0, -entry.gap), combinedSkin: 0,
        manifoldPointIndex: index, suppressContactEvent: false, mlcpV2: true, ...feature,
      };
    }).filter(Boolean);
    stats.last = { types: candidates.map((entry) => entry.role), gaps: candidates.map((entry) => entry.gap),
      freeNormal, freeTangent, lambdaN: best.lambdaN, lambdaT: best.lambdaT,
      normalBias, postNormal: best.postNormal, postComplementarity: best.postComplementarity,
      postTangent: best.postTangent, preEnergy, postEnergy,
      auditRows: candidates.map((entry, index) => ({ role: entry.role, row: entry.normalRow,
        predictedPost: best.postNormal[index], normalBias: normalBias[index], lambdaN: best.lambdaN[index] })),
    };
    return { ...stats.last, ownerStep: true, fullCluster: true, postWorldSolve, contactEntries };
  }

  function* stepPhysicsRigIterator(task) {
    const sim = state.sim; const physics = sim.physics;
    task.sim = sim; task.physics = physics; task.tickStartTime = sim.time;
    syncRigStateFromPhysics();
    const proximity = rigNearCadContact();
    const spatialResolution = Math.max(
      .00002,
      Math.min(RIG_INITIAL_CAD_SWEEP, CAD_COLLISION_CHORD * .25, targetThickness() * .05),
    );
    const angularSteps = proximity.nearWeapon
      ? Math.ceil(Math.abs(physics.weaponBody.angvel()) * activeWeaponRadius() * FIXED_DT / spatialResolution)
      : 1;
    // After a hit the long target may spin much faster than its centre moves.
    // Omitting that angular surface sweep let a 140 mm target corner travel
    // several millimetres through the weapon/fork/floor inside one nominal
    // fixed tick, even though the weapon's own CAD sweep was finely resolved.
    const targetOmegaAtTickStart = physics.targetBody.angvel();
    const targetAngularSteps = (proximity.nearWeapon || proximity.nearFork || proximity.nearFloor)
      ? Math.ceil(Math.abs(targetOmegaAtTickStart) * proximity.targetHalfDiagonal * FIXED_DT / spatialResolution)
      : 1;
    const forkAngularSteps = Math.ceil(Math.abs(physics.forkBody.angvel()) * Math.max(proximity.forkRadius, .001) * FIXED_DT / spatialResolution);
    const omegaAtTickStart = physics.weaponBody.angvel();
    // Rapier's per-integration rotation clamp is evaluated after forces have
    // accelerated the body.  Looking only at the start speed lets a real-start
    // motor get stuck at pi/(4 dt) (15,000 rpm for dt=0.5 ms). Predict the
    // torque-driven end speed so subdivision occurs before that hidden clamp.
    const predictedOmega = omegaAtTickStart
      + weaponMotorTorqueAt(omegaAtTickStart) / Math.max(state.metrics.inertia, 1e-12) * FIXED_DT;
    const rotationSteps = Math.ceil(Math.max(
      Math.max(Math.abs(omegaAtTickStart), Math.abs(predictedOmega)),
      Math.abs(targetOmegaAtTickStart),
      Math.abs(physics.forkBody.angvel()),
    ) * FIXED_DT / RAPIER_MAX_ROTATION_PER_STEP);
    const linearSteps = proximity.nearWeapon || proximity.nearFork
      ? Math.ceil(proximity.relativeLinearSpeed * FIXED_DT / spatialResolution)
      : 1;
    const requestedSubsteps = Math.max(1, rotationSteps, angularSteps, targetAngularSteps, forkAngularSteps, linearSteps);
    if (requestedSubsteps > RIG_MAX_SUBSTEPS) {
      stopRigSolver(`本步需要 ${requestedSubsteps} 个 Rapier 子步才能把 CAD 边界位移限制在 ${format(spatialResolution * 1000, 3)} mm 内，超过上限 ${RIG_MAX_SUBSTEPS}；仿真在推进前停止，未放宽精度或穿透。`);
      return { status: 'stopped', safeTerminal: true, failureDomain: 'rigid-solver' };
    }
    // A resumable tick is one transaction. Cancellation is legal only at a
    // complete Rapier substep boundary, and always restores this tick start.
    const worldSnapshot = snapshotRapierRig(physics);
    const bookkeepingSnapshot = snapshotRigBookkeeping();
    if (!worldSnapshot || !bookkeepingSnapshot) {
      stopRigInternal('无法取得完整 Rapier 固定步快照；仿真已在推进前停止。');
      return { status: 'stopped', safeTerminal: true, failureDomain: 'internal' };
    }
    const rejected = { ...sim.rigRejectedPenetration };
    task.worldSnapshot = worldSnapshot; task.bookkeepingSnapshot = bookkeepingSnapshot;
    task.requestedSubsteps = requestedSubsteps; task.rejected = rejected;
    let finalAttempt = null; let lastFailure = null; let restored = false;
    let previousMaterialSignature = null; let materialConvergence = null;

    const runAttempt = function* runRigAttempt(substeps) {
      const intervalDt = FIXED_DT / substeps; let elapsed = 0; let lastContacts = [];
      task.attemptSubsteps = substeps; task.intervalDt = intervalDt;
      task.substepIndex = 0; task.elapsed = 0; task.lastContacts = [];
      for (let index = 0; index < substeps; index += 1) {
        const before = rigEnergySnapshot();
        physics.robotBody.resetForces(true); physics.robotBody.resetTorques(true);
        physics.weaponBody.resetForces(true); physics.weaponBody.resetTorques(true);
        physics.forkBody.resetForces(true); physics.forkBody.resetTorques(true);
        const actuation = applyRigActuation(intervalDt);
        task.mlcpV2PostFailure = null;
        task.mlcpV2ActuationWork = null;
        task.mlcpV2 = solveRigContactMlcpV2(physics, intervalDt, 'probe');
        if (task.mlcpV2?.failure) {
          yield { status: 'substep', index, failed: true };
          return {
            ok: false, failureDomain: 'rigid-solver', invalidRoles: [task.mlcpV2.failure], intervalDt, elapsed, substeps,
            contactDiagnostics: (task.mlcpV2.types || []).map((role) => ({ role, mlcpV2Failure: true })),
          };
        }
        task.mlcpV2SolverGroups = null;
        let mlcpV2StepError = null;
        if (task.mlcpV2?.ownerStep) {
          const owned = [physics.targetCollider, ...(physics.weaponColliders || []), ...(physics.forkColliders || [])];
          task.mlcpV2SolverGroups = owned.map((collider) => [collider, collider.solverGroups()]);
        }
        try {
          if (task.mlcpV2?.ownerStep) {
          // The owned cluster includes both tool pairs and both floor pairs;
          // collision groups stay query-visible, while solver groups prevent a
          // second sequential impulse pass from overwriting the common KKT solve.
          const ownsFullCluster = Boolean(task.mlcpV2.fullCluster);
          physics.targetCollider.setSolverGroups(rapierInteractionGroups(RIG_GROUP_TARGET, ownsFullCluster ? 0 : RIG_GROUP_FLOOR));
          (physics.weaponColliders || []).forEach((collider) => collider.setSolverGroups(rapierInteractionGroups(RIG_GROUP_WEAPON, 0)));
          (physics.forkColliders || []).forEach((collider) => collider.setSolverGroups(rapierInteractionGroups(RIG_GROUP_FORK, ownsFullCluster ? 0 : RIG_GROUP_FLOOR)));
        }
        physics.world.integrationParameters.dt = intervalDt;
          physics.world.step();
          // The pre-world pass only chooses pair ownership. With owned pairs
          // masked, Rapier advances external forces and revolute joints once;
          // solve the contact KKT exactly once from that actual free velocity.
          // No second impulse pass, time advance, or position projection occurs.
          task.mlcpV2ActuationWork = rigActuationWork(actuation, intervalDt);
          if (task.mlcpV2?.probe && task.mlcpV2.fullCluster) {
            const solved = solveRigContactMlcpV2(physics, intervalDt, 'post');
            if (solved?.failure) {
              // Replace the probe before any telemetry pass: probe rows contain
              // roles only. Keeping them here would hide the physical domain
              // failure behind a TypeError and skip the ownership restore.
              task.mlcpV2 = solved;
              task.mlcpV2PostFailure = solved.failure;
            } else task.mlcpV2 = solved || { ownerStep: true, fullCluster: true, postWorldSolve: true,
              contactEntries: [], auditRows: [] };
          }
          if (task.mlcpV2?.auditRows?.some((entry) => !Array.isArray(entry.row))) {
            task.mlcpV2PostFailure = '持久多接触 MLCP 返回了不完整的互补审计行';
          } else if (task.mlcpV2?.auditRows) {
            const targetVelocity = physics.targetBody.linvel();
            const qActual = [physics.robotBody.linvel().x, physics.weaponBody.angvel(), physics.forkBody.angvel(),
              targetVelocity.x, targetVelocity.y, physics.targetBody.angvel()];
            const denseAuditDot = (left, right) => left.reduce((sum, value, axis) => sum + value * right[axis], 0);
            const actual = task.mlcpV2.auditRows.map((entry) => {
              const normal = denseAuditDot(entry.row, qActual); const complementarity = normal + entry.normalBias;
              return { role: entry.role, predicted: entry.predictedPost, actual: normal,
                normalBias: entry.normalBias, lambdaN: entry.lambdaN, complementarity };
            });
            const stats = window.__MlcpV2Stats;
            if (stats) {
              stats.maxRapierPostClosing = Math.max(stats.maxRapierPostClosing || 0, ...actual.map((entry) => Math.max(0, -entry.actual)));
              stats.maxRapierPostDelta = Math.max(stats.maxRapierPostDelta || 0, ...actual.map((entry) => Math.abs(entry.actual - entry.predicted)));
              stats.maxRapierPostComplementarityViolation = Math.max(stats.maxRapierPostComplementarityViolation || 0,
                ...actual.map((entry) => Math.max(0, -entry.complementarity)));
              stats.maxRapierPostComplementarityProduct = Math.max(stats.maxRapierPostComplementarityProduct || 0,
                ...actual.map((entry) => Math.abs(entry.lambdaN * entry.complementarity)));
              if (actual.some((entry) => entry.complementarity < -1e-6
                || Math.abs(entry.lambdaN * entry.complementarity) > 1e-6)) {
                task.mlcpV2PostFailure = '持久多接触 MLCP 在世界步后的互补残差未收敛';
              }
              stats.lastRapierPost = actual;
            }
          }
          task.worldAdvanced = true;
        } catch (error) {
          // Preserve the original simulation/program error, but defer rethrowing
          // until every collider has had an independent restoration attempt.
          mlcpV2StepError = error;
        } finally {
          // Solver ownership is transactional even when Rapier, MLCP, or audit
          // code throws. A leaked zero filter would silently disable later
          // collision pairs outside the rejected attempt.
          let mlcpV2RestoreError = null;
          for (const [collider, groups] of task.mlcpV2SolverGroups || []) {
            try { collider.setSolverGroups(groups); }
            catch (error) { if (!mlcpV2RestoreError) mlcpV2RestoreError = error; }
          }
          if (mlcpV2StepError) throw mlcpV2StepError;
          if (mlcpV2RestoreError) throw mlcpV2RestoreError;
        }
        if (task.mlcpV2PostFailure) {
          yield { status: 'substep', index, failed: true };
          return { ok: false, failureDomain: 'rigid-solver', invalidRoles: [task.mlcpV2PostFailure], intervalDt, elapsed, substeps };
        }
        // Freeze motor work at the end of Rapier's force-integration segment.
        // Material and boundary impulses below are instantaneous collision
        // events; including their post-impact velocities in the trapezoidal
        // motor integral would attribute collision braking to the motor and
        // corrupt the energy ledger.
        const actuationWork = Number.isFinite(task.mlcpV2ActuationWork)
          ? task.mlcpV2ActuationWork
          : rigActuationWork(actuation, intervalDt);
        syncRigStateFromPhysics();
        const afterWorld = rigEnergySnapshot();
        const constraintEnergyExchange = rigMechanicalEnergy(before) + actuationWork
          - rigMechanicalEnergy(afterWorld);
        const rapierContacts = collectRigContacts(physics);
        // Solver-owned manifolds remain query-visible but have zero Rapier
        // impulses. Replace, rather than append, those pair records so one
        // physical tooth pass produces one episode and one momentum ledger.
        const contacts = task.mlcpV2?.ownerStep
          ? [
            ...rapierContacts.filter((contact) => !(task.mlcpV2.fullCluster ? ['weapon', 'fork', 'floor'] : ['weapon', 'fork']).includes(contact.role)),
            ...(task.mlcpV2.contactEntries || []),
          ]
          : rapierContacts;
        const unsupportedMaterialContact = state.params.contactModel === 'material'
          && !physics.materialResponseEnabled
          && contacts.find((contact) => contact.role === 'weapon'
            && contact.geometricGap <= RIG_CONTACT_EVENT_MAX_GAP
            && (contact.impulse > 1e-10 || contact.tangentImpulse > 1e-10));
        if (unsupportedMaterialContact) {
          // Rapier has provisionally solved this substep as a rigid contact, but
          // the enclosing fixed tick is transactional.  Refuse the unsupported
          // material claim here; the caller restores the tick-start snapshot,
          // so none of that rigid impulse reaches the target or playback cache.
          yield { status: 'substep', index, failed: true };
          return {
            ok: false,
            failureDomain: 'material-model',
            retryable: false,
            modelDomain: true,
            invalidRoles: [
              state.params.paramWeaponEnabled
                ? '参数化刀齿发生了承载接触，但其连续塑性剐蹭、沟槽历史与去料表面尚未通过验证；材料模式不会用完全刚体冲量冒充有限材料响应'
                : '导入 CAD 武器发生了承载接触，但连续最早接触 TOI、有限塑性/断裂与变形后材料表面尚未通过验证；材料模式不会用完全刚体冲量冒充有限材料响应',
            ],
            intervalDt,
            elapsed,
            substeps,
            contactDiagnostics: [{
              role: 'weapon',
              isTooth: Boolean(unsupportedMaterialContact.isTooth),
              toothOrder: unsupportedMaterialContact.toothOrder,
              penetration: unsupportedMaterialContact.penetration,
              gap: unsupportedMaterialContact.geometricGap,
              provisionalRigidImpulse: Math.hypot(
                unsupportedMaterialContact.impulse || 0,
                unsupportedMaterialContact.tangentImpulse || 0,
              ),
            }],
          };
        }
        const materialResult = applyMaterialContactLimits(contacts, before, intervalDt, physics);
        if (!materialResult.ok) yield { status: 'substep', index, failed: true };
        if (!materialResult.ok) return {
          ok: false,
          failureDomain: materialResult.failureDomain || 'material-model',
          retryable: true,
          invalidRoles: [materialResult.reason || '材料接触子步无法收敛'],
          intervalDt, elapsed, substeps,
        };
        const manifoldPenetration = contacts.reduce((byRole, contact) => {
          const materialAllowance = state.params.contactModel === 'material' && contact.role === 'weapon'
            ? Math.max(0, contact.material?.allowedPenetration || 0)
            : 0;
          const residual = Math.max(0, contact.penetration - materialAllowance);
          byRole[contact.role] = Math.max(byRole[contact.role] || 0, residual);
          if (contact.role === 'weapon') sim.materialMaxIntrusion = Math.max(sim.materialMaxIntrusion, contact.penetration);
          return byRole;
        }, { floor: 0, fork: 0, weapon: 0 });
        const hasForkContact = contacts.some((contact) => contact.role === 'fork');
        const hasWeaponContact = contacts.some((contact) => contact.role === 'weapon');
        const exactCadPenetration = {
          fork: effectiveToolWidthZ('fork') <= 1e-9
            ? 0
            : !proximity.nearFork && !hasForkContact
            ? 0
            : exactCadLoopTargetPenetration(physics.forkLoop, physics.forkBody, physics.targetBody),
          // A damaged material lane can contain holes and disconnected pieces;
          // its exact gate is handled by the material polygon/allowance path.
          // The rigid reference target remains a rectangle, so use the actual
          // CAD outer loop instead of any Rapier convex-decomposition feature.
          weapon: !proximity.nearWeapon && !hasWeaponContact
            ? 0
            : !physics.materialResponseEnabled
              // The hard 0.08 mm gate is a geometric statement.  Rapier's
              // convex-manifold depth can jump between features even when a
              // microsecond substep moved only a few micrometres.  Use every
              // active triangle/CAD loop as the ruler; Rapier remains the
              // coupled impulse solver.  Multi-tooth parameter geometry must
              // check all teeth rather than only the first collider.
              ? exactWeaponLoopsTargetPenetration(physics)
              : manifoldPenetration.weapon,
        };
        const hasFloorManifold = contacts.some((contact) => contact.role === 'floor');
        const penetration = {
          // Use the same raw collider geometry that Rapier solved.  Recomputing
          // a rotated corner from the single-precision body pose can differ by
          // a few nanometres exactly at the hard 0.08 mm gate and falsely reject
          // an otherwise valid step.  The analytic corner remains a guard when
          // no floor manifold exists at all (for example a missed broad-phase
          // contact); this changes no tolerance and never uses solver/skin
          // distance as geometric penetration.
          // A predictive floor manifold may still report zero raw overlap while
          // a fast rotating rectangle corner has already crossed the support
          // plane.  Accepting that tick makes the next retry start from an
          // already embedded state, so refinement can never recover.  Keep the
          // raw Rapier distance, but always pair it with the exact rotated-corner
          // support test and use the stricter result.  This retains the same
          // 0.08 mm hard limit; it changes neither contact forces nor tolerance.
          floor: Math.max(
            manifoldPenetration.floor,
            Math.max(0, targetSupportY() - targetLowestPoint()),
          ),
          forkFloor: proximity.nearForkFloor
            ? exactLoopFloorPenetration(physics.forkLoop, physics.forkBody, groundY())
            : 0,
          fork: exactCadPenetration.fork,
          weapon: exactCadPenetration.weapon,
        };
        const outOfBandTool = contacts
          .filter((contact) => (contact.role === 'fork' || contact.role === 'weapon')
            && contact.impulse > 1e-10
            && contact.geometricGap > RIG_CONTACT_EVENT_MAX_GAP)
          .sort((a, b) => b.impulse - a.impulse)[0];
        const ambiguousParametricRoot = contacts.find((contact) => contact.role === 'weapon'
          && contact.parametricOverlapCount > 1
          && (contact.penetration > 0 || contact.impulse > 1e-10));
        const jointError = sim.rigJointError;
        const forkJointError = sim.rigForkJointError;
        const invalidRoles = [];
        if (penetration.floor > RIG_FLOOR_PENETRATION_TOLERANCE) invalidRoles.push(`地面 ${format(penetration.floor * 1000, 4)} mm`);
        if (penetration.forkFloor > RIG_FLOOR_PENETRATION_TOLERANCE) invalidRoles.push(`叉子—真实地面 ${format(penetration.forkFloor * 1000, 4)} mm`);
        if (penetration.fork > RIG_CAD_PENETRATION_TOLERANCE) invalidRoles.push(`叉子 ${format(penetration.fork * 1000, 4)} mm`);
        if (penetration.weapon > RIG_CAD_PENETRATION_TOLERANCE) invalidRoles.push(`武器 ${format(penetration.weapon * 1000, 4)} mm`);
        if (jointError > rigJointErrorTolerance()) invalidRoles.push(`转轴误差 ${format(jointError * 1000, 4)} mm（上限 ${format(rigJointErrorTolerance() * 1000, 4)} mm）`);
        if (forkJointError > rigForkJointErrorTolerance()) invalidRoles.push(`叉子铰点误差 ${format(forkJointError * 1000, 4)} mm（上限 ${format(rigForkJointErrorTolerance() * 1000, 4)} mm）`);
        if (outOfBandTool) invalidRoles.push(`${outOfBandTool.role === 'weapon' ? '武器' : '叉子'}预测接触间隙 ${format(outOfBandTool.geometricGap * 1000, 4)} mm`);
        if (ambiguousParametricRoot) invalidRoles.push(`测试齿根部同时进入 ${ambiguousParametricRoot.parametricOverlapCount} 个显式三角齿实体；不能把重复约束当作独立命中`);
        if (invalidRoles.length) {
          rejected.floor = Math.max(rejected.floor, penetration.floor);
          rejected.forkFloor = Math.max(rejected.forkFloor || 0, penetration.forkFloor);
          rejected.fork = Math.max(rejected.fork, penetration.fork);
          rejected.weapon = Math.max(rejected.weapon, penetration.weapon);
          yield { status: 'substep', index, failed: true };
          return {
            ok: false, failureDomain: 'rigid-solver', invalidRoles, intervalDt, elapsed, substeps,
            contactDiagnostics: contacts
              .filter((entry) => entry.role === 'weapon' || entry.role === 'fork')
              .map((entry) => ({
                role: entry.role, isTooth: entry.isTooth, cutFollower: Boolean(entry.cutFollower),
                toothOrder: entry.toothOrder, penetration: entry.penetration,
                allowedPenetration: entry.material?.allowedPenetration || 0,
                gap: entry.geometricGap, impulse: entry.impulse,
                model: entry.material?.model || null,
              })),
          };
        }
        sim.rigMaxPenetration.floor = Math.max(sim.rigMaxPenetration.floor, penetration.floor);
        sim.rigMaxPenetration.forkFloor = Math.max(sim.rigMaxPenetration.forkFloor || 0, penetration.forkFloor);
        sim.rigMaxPenetration.fork = Math.max(sim.rigMaxPenetration.fork, penetration.fork);
        sim.rigMaxPenetration.weapon = Math.max(sim.rigMaxPenetration.weapon, penetration.weapon);
        sim.rigMaxJointError = Math.max(sim.rigMaxJointError, jointError);
        sim.rigMaxForkJointError = Math.max(sim.rigMaxForkJointError, forkJointError);
        elapsed += intervalDt;
        registerRigContacts(contacts, before, sim.time + elapsed, actuationWork, intervalDt, {
          constraintEnergyExchange,
          massRemovalEnergy: materialResult.massRemovalEnergy || 0,
        });
        lastContacts = contacts;
        task.substepIndex = index + 1; task.elapsed = elapsed; task.lastContacts = contacts;
        yield { status: 'substep', index, failed: false };
      }
      return { ok: true, elapsed, substeps, lastContacts };
    };

    for (let refinement = 0; refinement <= RIG_MAX_REFINEMENT; refinement += 1) {
      task.refinement = refinement;
      const attemptSubsteps = requestedSubsteps * (2 ** refinement);
      if (attemptSubsteps > RIG_MAX_SUBSTEPS) {
        const previousReason = lastFailure?.invalidRoles?.join('；');
        lastFailure = {
          failureDomain: lastFailure?.failureDomain || 'rigid-solver',
          invalidRoles: [previousReason, `下一次加密需要 ${attemptSubsteps} 个子步（上限 ${RIG_MAX_SUBSTEPS}）`].filter(Boolean),
          refinement,
          substeps: attemptSubsteps,
        };
        break;
      }
      if (refinement > 0) {
        restored = Boolean(worldSnapshot && restoreRapierRig(physics, worldSnapshot));
        if (!restored) {
          lastFailure = {
            failureDomain: 'internal',
            invalidRoles: ['加密重算前无法恢复固定步世界快照'],
            refinement,
            substeps: attemptSubsteps,
          };
          break;
        }
        restoreRigBookkeeping(bookkeepingSnapshot);
        syncRigStateFromPhysics();
      }
      const attempt = yield* runAttempt(attemptSubsteps);
      if (attempt.ok) {
        const signature = state.params.contactModel === 'material'
          ? materialTickSignature(bookkeepingSnapshot)
          : null;
        if (previousMaterialSignature || signature?.hasResponse) {
          if (!worldSnapshot || !bookkeepingSnapshot) {
            lastFailure = {
              failureDomain: 'material-model',
              invalidRoles: ['材料接触固定步没有完整世界快照，无法执行双分辨率收敛验证'],
              refinement,
              substeps: attemptSubsteps,
            };
            break;
          }
          if (!previousMaterialSignature) {
            previousMaterialSignature = signature;
            lastFailure = {
              failureDomain: 'material-model',
              invalidRoles: ['材料响应需以 2× 子步重新求解同一固定步，验证删料、功和终态收敛'],
              refinement,
              substeps: attemptSubsteps,
            };
            task.lastFailure = lastFailure;
            yield { status: 'attempt-transition', refinement };
            continue;
          }
          materialConvergence = compareMaterialTickSignatures(previousMaterialSignature, signature);
          if (!materialConvergence.converged) {
            previousMaterialSignature = signature;
            lastFailure = {
              failureDomain: 'material-model',
              invalidRoles: [`材料双分辨率未收敛：${materialConvergence.reason}`],
              refinement,
              substeps: attemptSubsteps,
              materialConvergence,
            };
            task.lastFailure = lastFailure;
            yield { status: 'attempt-transition', refinement };
            continue;
          }
        }
        finalAttempt = { ...attempt, materialConvergence };
        task.finalAttempt = finalAttempt;
        break;
      }
      lastFailure = { ...attempt, refinement };
      task.lastFailure = lastFailure;
      // A missing finite-material law is not a discretisation error.  Retrying
      // the same provisional rigid collision at 2x, 4x, ... resolution cannot
      // turn it into validated scraping physics, so terminate this transaction
      // immediately and let the common rollback path restore its start state.
      if ((attempt.failureDomain === 'material-model' || attempt.modelDomain) && attempt.retryable === false) break;
      if (!worldSnapshot) break;
    }

    sim.rigRejectedPenetration.floor = Math.max(sim.rigRejectedPenetration.floor, rejected.floor);
    sim.rigRejectedPenetration.forkFloor = Math.max(sim.rigRejectedPenetration.forkFloor || 0, rejected.forkFloor || 0);
    sim.rigRejectedPenetration.fork = Math.max(sim.rigRejectedPenetration.fork, rejected.fork);
    sim.rigRejectedPenetration.weapon = Math.max(sim.rigRejectedPenetration.weapon, rejected.weapon);
    if (!finalAttempt) {
      const rolledBack = Boolean(worldSnapshot && restoreRapierRig(physics, worldSnapshot));
      if (rolledBack) { restoreRigBookkeeping(bookkeepingSnapshot); syncRigStateFromPhysics(); }
      sim.rigRejectedPenetration.floor = Math.max(sim.rigRejectedPenetration.floor, rejected.floor);
      sim.rigRejectedPenetration.forkFloor = Math.max(sim.rigRejectedPenetration.forkFloor || 0, rejected.forkFloor || 0);
      sim.rigRejectedPenetration.fork = Math.max(sim.rigRejectedPenetration.fork, rejected.fork);
      sim.rigRejectedPenetration.weapon = Math.max(sim.rigRejectedPenetration.weapon, rejected.weapon);
      sim.lastRigFailure = lastFailure ? clonePlaybackPlain(lastFailure) : null;
      const reason = lastFailure?.invalidRoles?.join('；') || '未知收敛错误';
      const failureDomain = lastFailure?.failureDomain
        || (lastFailure?.modelDomain ? 'material-model' : 'rigid-solver');
      if (failureDomain === 'material-model') {
        stopRigModelDomain(
          `${reason}。${rolledBack ? '已回滚到本固定步开始，未提交临时刚体冲量或靶子能量跳变。' : '本固定步没有可用快照，结果不可用。'}如需查看完全刚性碰撞，只能显式选择“理想刚体上限对照”；1/10/20 齿材料剐蹭不会按齿数缩放冲量。`,
          '材料接触缺少已验证响应',
        );
        return { status: 'stopped', safeTerminal: rolledBack, failureDomain, modelDomain: true };
      }
      if (failureDomain === 'internal') {
        stopRigInternal(`${reason}。${rolledBack ? '已回滚到本固定步开始，未提交异常尝试态。' : '本固定步没有可用快照，结果不可用。'}`);
        return { status: 'stopped', safeTerminal: rolledBack, failureDomain };
      }
      stopRigSolver(`Rapier 整个固定步经 ${lastFailure?.refinement ?? 0} 级加密后仍未在数值容差内收敛：${reason}。${rolledBack ? '已回滚至该固定步开始' : '该步没有可用快照'}并停止，不作位置搬运。`);
      return { status: 'stopped', safeTerminal: rolledBack, failureDomain };
    }
    if (!rebuildMaterialTargetCollider(physics)) {
      const rolledBack = Boolean(worldSnapshot && restoreRapierRig(physics, worldSnapshot));
      if (rolledBack) { restoreRigBookkeeping(bookkeepingSnapshot); syncRigStateFromPhysics(); }
      sim.lastRigFailure = {
        failureDomain: 'material-model',
        invalidRoles: ['材料缺口无法重建为有效的剩余靶子碰撞轮廓'],
      };
      stopRigModelDomain(`材料缺口无法重建为有效的剩余靶子碰撞轮廓；${rolledBack ? '已回滚本固定步并' : ''}停止，未用完整矩形继续阻挡刀齿。`, '材料轮廓超出有效域');
      return { status: 'stopped', safeTerminal: rolledBack, failureDomain: 'material-model' };
    }
    sim.materialConvergenceLastTick = finalAttempt.materialConvergence
      ? clonePlaybackPlain(finalAttempt.materialConvergence)
      : null;
    if (finalAttempt.materialConvergence && sim.rigWeaponEpisode) {
      sim.rigWeaponEpisode.materialTickConverged = true;
      sim.rigWeaponEpisode.materialConvergenceChecks = number(sim.rigWeaponEpisode.materialConvergenceChecks) + 1;
    }
    sim.rigSubstepsLastTick = finalAttempt.substeps;
    sim.time += FIXED_DT;
    sim.forkEngaged = finalAttempt.lastContacts.some((contact) => contact.role === 'fork');
    sim.targetPushedByFork = sim.targetPushedByFork || sim.forkEngaged;
    if (sim.trail.length === 0 || sim.time - sim.trail[sim.trail.length - 1].time > .016) sim.trail.push({ ...sim.target.pos, time: sim.time });
    if (sim.trail.length > 100) sim.trail.shift();
    const endTime = simulationEndTime();
    if (sim.time >= endTime - FIXED_DT * 1e-6) {
      sim.running = false; sim.completed = true;
      addEvent(`到达设定计算时长 ${endTime.toFixed(4)} s，自动暂停。`, 'done'); updateStatus('计算区间完成', 'paused');
    }
    return { status: 'committed' };
  }

  function beginRigTickTask({ trajectory = null, generation = null } = {}) {
    const sim = state.sim;
    const task = {
      trajectory, generation, sim, physics: sim?.physics, tickStartTime: sim?.time,
      pendingEvents: [], eventTarget: trajectory?.events || state.eventEntries,
      worldSnapshot: null, bookkeepingSnapshot: null, requestedSubsteps: 0,
      refinement: 0, attemptSubsteps: 0, intervalDt: 0, substepIndex: 0,
      elapsed: 0, lastContacts: [], lastFailure: null, finalAttempt: null,
      rejected: null, done: false, cancelled: false, eventsCommitted: false,
      resumeCount: 0, yieldCount: 0, substepBoundaries: 0, worldAdvanced: false,
    };
    task.iterator = stepPhysicsRigIterator(task);
    return task;
  }

  function rigTickTaskIdentityValid(task) {
    if (!task || task.cancelled || task.done || state.sim !== task.sim || task.physics !== task.sim?.physics) return false;
    if (!task.trajectory) return true;
    return state.trajectory === task.trajectory
      && task.trajectory.generation === task.generation
      && task.trajectory.workSim === task.sim;
  }

  function commitRigTickEvents(task) {
    if (task.eventsCommitted) return;
    task.eventTarget.push(...task.pendingEvents);
    task.eventsCommitted = true;
    if (!task.trajectory?.building) renderEventEntries(task.eventTarget);
  }

  function releaseRigTickTask(task) {
    task.worldSnapshot = null;
    task.bookkeepingSnapshot = null;
    task.lastContacts = [];
    task.iterator = null;
  }

  function resumeRigTickTask(task, deadline = Infinity, maxBoundaries = Infinity) {
    if (!rigTickTaskIdentityValid(task)) return { status: 'cancelled' };
    const previousActive = state.activeRigTick;
    state.activeRigTick = task;
    task.resumeCount += 1;
    let progressed = 0;
    try {
      for (;;) {
        if (progressed > 0 && (progressed >= maxBoundaries || performance.now() >= deadline)) {
          task.yieldCount += 1;
          return { status: 'yielded' };
        }
        const next = task.iterator.next();
        if (next.done) {
          task.done = true;
          task.result = next.value || { status: task.sim.completed ? 'stopped' : 'committed' };
          commitRigTickEvents(task);
          releaseRigTickTask(task);
          return task.result;
        }
        progressed += 1;
        task.substepBoundaries += 1;
      }
    } finally {
      state.activeRigTick = previousActive;
    }
  }

  function cancelRigTickTask(task) {
    if (!task || task.done || task.cancelled) return false;
    const visibleSim = state.sim; const previousActive = state.activeRigTick;
    state.sim = task.sim; state.activeRigTick = task;
    try {
      if (task.worldSnapshot && restoreRapierRig(task.physics, task.worldSnapshot)) {
        restoreRigBookkeeping(task.bookkeepingSnapshot);
        syncRigStateFromPhysics();
      }
      task.pendingEvents.length = 0;
      task.cancelled = true; task.iterator?.return?.(); task.done = true;
      releaseRigTickTask(task);
      return true;
    } finally {
      state.activeRigTick = previousActive; state.sim = visibleSim;
    }
  }

  function abortRigTickTask(task, error) {
    const previousActive = state.activeRigTick;
    state.activeRigTick = task;
    try {
      const rolledBack = Boolean(task.worldSnapshot && restoreRapierRig(task.physics, task.worldSnapshot));
      if (rolledBack) { restoreRigBookkeeping(task.bookkeepingSnapshot); syncRigStateFromPhysics(); }
      task.pendingEvents.length = 0;
      task.sim.running = false; task.sim.completed = true;
      task.sim.failureDomain = 'internal';
      task.sim.solverDomainStopped = false; task.sim.modelDomainStopped = false;
      task.sim.creationError = error?.message || '轨迹计算失败';
      task.sim.lastRigFailure = {
        failureDomain: 'internal',
        invalidRoles: [task.sim.creationError],
        exceptionName: error?.name || 'Error',
      };
      addEvent(`轨迹计算失败：${task.sim.creationError}${rolledBack ? '；已回滚到固定步起点' : ''}`, 'warning');
      updateStatus('内部异常安全停止', 'warning');
      task.iterator?.return?.(); task.done = true; task.result = {
        status: 'stopped', error: task.sim.creationError,
        failureDomain: 'internal',
        safeTerminal: rolledBack || !task.worldAdvanced,
      };
      commitRigTickEvents(task); releaseRigTickTask(task);
      return task.result;
    } finally {
      state.activeRigTick = previousActive;
    }
  }

  function stepPhysicsRig() {
    let task = null;
    try {
      task = beginRigTickTask();
      const result = resumeRigTickTask(task, Infinity, Infinity);
      if (result.status === 'yielded') throw new Error('Infinite-budget rig tick yielded unexpectedly.');
      return result;
    } catch (error) {
      // Direct BiteSim.advance()/stepPhysics() previously let a Rapier or MLCP
      // exception escape with a partially advanced fixed tick.  Route it through
      // the same transactional abort used by the trajectory worker.
      if (task) return abortRigTickTask(task, error);
      state.sim.creationError = error?.message || '直接刚体计算失败';
      stopRigInternal(`直接刚体计算失败：${state.sim.creationError}；异常发生在世界推进前，未提交固定步状态。`);
      return { status: 'stopped', error: state.sim.creationError, failureDomain: 'internal', safeTerminal: true };
    }
  }

  function stepPhysicsToi() {
    const sim = state.sim;
    let remaining = FIXED_DT; let contactForUi = null;
    const active = new Set(sim.activeContactIds); const handled = new Set();
    const targetCooldown = TARGET_CONTACT_COOLDOWN;
    let impactCount = 0;
    for (; remaining > 1e-9 && impactCount < MAX_CCD_IMPACTS_PER_STEP; impactCount += 1) {
      const start = snapshotMotionState();
      advancePhysicalSegment(remaining);
      const predicted = detectWeaponContacts(); const first = predicted[0];
      const isNewImpactTooth = first?.isImpactSample && !active.has(first.index) && !handled.has(first.index);
      const eligible = first
        && (!first.isImpactSample || isNewImpactTooth)
        && sim.time - (sim.toothCooldowns.get(first.index) ?? -Infinity) >= TOOTH_CONTACT_COOLDOWN
        && sim.time - sim.lastImpactTime >= targetCooldown;
      if (!first) {
        // The predicted state contains no CAD boundary crossing, so it is a
        // valid accepted segment.
        remaining = 0;
        break;
      }
      if (!eligible) {
        // Never accept a predicted end state that already crossed a contact
        // merely because its tooth is cooling down. Roll back to the actual
        // TOI and suspend this unresolved persistent contact instead.
        const hitDt = clamp(remaining * first.sweepFraction, 0, remaining);
        restoreMotionState(start);
        if (hitDt > 1e-9) advancePhysicalSegment(hitDt);
        const contact = materialiseWeaponContact(first);
        contactForUi = contact;
        sim.running = false; sim.completed = true; sim.weaponBlocked = true;
        addEvent('持续 DXF 接触落在命中冷却窗口内；仿真已回退到首次 TOI 并暂停，未继续推进穿透。', 'warning');
        updateStatus('持续接触待建模', 'warning');
        remaining = 0;
        break;
      }
      const hitDt = clamp(remaining * first.sweepFraction, 0, remaining);
      restoreMotionState(start);
      if (hitDt > 1e-9) advancePhysicalSegment(hitDt);
      const contact = materialiseWeaponContact(first);
      if (contact.isImpactSample && triggerImpact(contact)) contact.impact = sim.lastImpact;
      else {
        // A real DXF outer edge arrived first, but it is not a tooth edge. It
        // blocks the solid blade without pretending that a grazing rim bite
        // can inject tooth-strike energy into the target.
        contact.impact = applyBlockingResponse(contact) || null;
        projectCadOutOfTarget();
        // The ideal rigid constraint has no remaining prescribed motion to
        // advance through this same CAD boundary during the current step.
        remaining = 0;
      }
      contactForUi = contact; active.add(contact.index); handled.add(contact.index);
      remaining -= hitDt;
      // A zero-fraction impact is already separated, so consume a tiny slice
      // instead of asking the same interval to report the same boundary again.
      if (hitDt <= 1e-9) remaining = Math.max(0, remaining - 1e-8);
    }
    if (remaining > 1e-9 && impactCount >= MAX_CCD_IMPACTS_PER_STEP) {
      sim.running = false; sim.completed = true;
      sim.failureDomain = 'rigid-solver';
      sim.solverDomainStopped = true; sim.modelDomainStopped = false;
      annotateRigFailure(sim, sim.failureDomain, `同一固定步内需要超过 ${MAX_CCD_IMPACTS_PER_STEP} 次 DXF TOI 接触`);
      addEvent(`同一 ${format(FIXED_DT * 1000, 3)} ms 步内需要超过 ${MAX_CCD_IMPACTS_PER_STEP} 次 DXF TOI 接触；为避免未检测穿透，仿真已停止。请降低转速、增大靶厚或缩短步长。`, 'warning');
      updateStatus('接触密度超出求解有效域', 'warning');
      remaining = 0;
    } else if (remaining > 1e-9) advancePhysicalSegment(remaining);
    // A hard cap avoids pathological high-RPM loops. Any residual exterior
    // overlap gets only non-energetic separation, never another energy pulse.
    projectCadOutOfTarget();
    finishPhysicsTick(contactForUi, active);
  }

  function stepPhysics() {
    if (!state.sim || state.sim.completed) return;
    if (state.sim?.physics?.rigModel) {
      stepPhysicsRig();
      return;
    }
    stepPhysicsToi();
    return;
    const sim = state.sim; const dt = FIXED_DT;
    updateDrive(dt);
    sim.robotTravel += sim.driveSpeed * dt;
    sim.forkEngaged = false; sim.forkSupportLift = 0;
    if (sim.targetLaunched) updateFreeTarget(dt);
    else {
      const forkPushing = applyForkContact(dt);
      // Before contact the target stays at its field placement.  A fork-only
      // interaction is constrained to a grounded slide; only a tooth impact
      // calls `triggerImpact` and releases it into the flight solver.
      if (forkPushing || sim.targetPushedByFork) updateGroundedTarget(dt);
      else { sim.target.pos.x = number(state.params.targetSceneX) / 1000; sim.target.pos.y = targetRestY(); sim.target.vel = point(0, 0); sim.target.angle = 0; sim.target.omega = 0; sim.target.grounded = true; }
    }
    recoverWeaponSpeed(dt);
    sim.lastWeaponAngleStep = signedOmega() * dt;
    sim.angle += sim.lastWeaponAngleStep;
    if (Math.abs(sim.angle) > Math.PI * 8) sim.angle %= Math.PI * 2;
    // A blade may reach the target before a particular fork geometry does.
    // Do not artificially require a prior fork contact to arm weapon impacts.
    const contacts = detectWeaponContacts();
    const active = new Set(contacts.map((contact) => contact.index));
    const targetCooldown = TARGET_CONTACT_COOLDOWN;
    const entering = contacts.filter((contact) => !sim.activeContactIds.has(contact.index) && sim.time - (sim.toothCooldowns.get(contact.index) ?? -Infinity) >= TOOTH_CONTACT_COOLDOWN && sim.time - sim.lastImpactTime >= targetCooldown);
    entering.forEach((contact) => {
      if (triggerImpact(contact)) contact.impact = sim.lastImpact;
    });
    // Store the target's end pose after any impulse so the next tooth sweep
    // begins from the same target state that the previous tip sample used.
    sim.previousTargetPose = snapshotTargetPose(sim.target);
    sim.activeContactIds = active;
    sim.currentContact = contacts.length ? contacts[0] : null;
    if (sim.currentContact && !sim.currentContact.impact) sim.currentContact.impact = estimateImpact(sim.currentContact);
    sim.time += dt;
    const endTime = simulationEndTime();
    if (sim.time >= endTime - FIXED_DT * 1e-6) {
      sim.running = false; sim.completed = true;
      addEvent(`到达设定计算时长 ${endTime.toFixed(4)} s，自动暂停。`, 'done'); updateStatus('计算区间完成', 'paused');
    }
  }

  function resetSimulation(message) {
    invalidateTrajectory();
    disposeTargetPhysics(state.sim?.physics);
    state.metrics = computeMetrics();
    state.sim = createSimulation();
    state.accumulator = 0;
    updateReadouts(); drawScene();
    if (state.sim.creationError) {
      addEvent(`刚体场景无效：${state.sim.creationError}`, 'warning');
      updateStatus('刚体场景参数无效', 'warning');
      showToast(state.sim.creationError, 'error');
    } else {
      updateStatus('内置模型就绪', 'ready');
      if (message) showToast(message);
    }
  }

  function updateReadouts() {
    state.metrics = computeMetrics();
    const m = state.metrics;
    setText('weaponRpm', format(m.weaponRpm)); setText('angularVelocity', format(m.angularVelocity)); setText('tipSpeed', format(m.tipSpeed));
    setText('vehicleSpeed', format(m.theoreticalVehicleSpeed));
    setText('biteTheoretical', m.cadToothCount ? format(m.biteIdeal) : '无齿');
    setText('biteIdeal', m.cadToothCount ? format(m.biteInstant) : '无齿');
    setText('weaponEnergy', format(m.weaponEnergy));
    setText('weaponShaftTorque', Number.isFinite(m.weaponCurrentLimitedTorque) ? format(m.weaponCurrentLimitedTorque) : '未限流'); setText('weaponPowerOut', format(m.weaponPowerOut)); setText('driveTraction', format(m.driveTraction));
    setText('liveWeaponRpm', format(Math.abs(signedOmega()) * 60 / (Math.PI * 2)));
    setText('liveWeaponEnergy', format(.5 * m.inertia * signedOmega() ** 2));
    setText('liveDriveSpeed', format(m.instantaneousDriveSpeed)); setText('drivePowerOut', format(m.drivePowerOut));
    const floorFriction = targetFloorFriction(); const contactFriction = weaponTargetFriction(); const forkFriction = shovelTargetFriction(); const activeFloorRestitution = getFloorMaterial().restitution;
    setText('contactFriction', format(contactFriction)); setText('shovelFriction', format(forkFriction)); setText('floorFriction', `${format(floorFriction.static)} / ${format(floorFriction.kinetic)}`); setText('floorRestitution', format(activeFloorRestitution)); setText('driveSlip', format(state.sim.driveSlip * 100));
    setText('materialReadout', `${getWeaponMaterial().label}→${getMaterial().label} μk ${format(contactFriction)}；${getShovelMaterial().label}→${getMaterial().label} μk ${format(forkFriction)}；${getMaterial().label}→${getFloorMaterial().label} μs 参考 ${format(floorFriction.static)} / 当前 Rapier 求解 μ ${format(floorFriction.kinetic)}（干燥、清洁、常温估算 × ${format(frictionScale())}；高级面板收起时仍使用同一数值）。`);
    const material = targetMaterialProperties(); const geometryMass = targetGeometryMass(); const enteredMass = positive(state.params.targetMass, .01);
    const massDifference = enteredMass > 0 ? Math.abs(geometryMass - enteredMass) / enteredMass : Infinity;
    const massModeLabel = state.params.targetMassMode === 'geometry' ? '尺寸×密度' : '实测/等效质量';
    const consistency = `尺寸推算 ${format(geometryMass, 4)} kg；输入 ${format(enteredMass, 4)} kg；运动采用${massModeLabel} ${format(effectiveTargetMass(), 4)} kg。`;
    setText('materialConsistencyReadout', `${consistency}${massDifference > .2 ? ' 两者相差超过 20%，请核对 Z 宽、密度或确认输入质量包含支架/夹具。' : ' 质量与尺寸基本自洽。'}`);
    $('#materialConsistencyReadout')?.classList.toggle('warning', massDifference > .2);
    const massConstraint = m.weaponMassConsistent
      ? `底盘剩余质量 ${format(m.chassisMassBudget, 4)} kg。`
      : `质量预算无效：扣除武器和叉子后，底盘质量必须大于 0 kg。`;
    const shapeLabel = m.weaponMassPropertySource === 'cad-even-odd-uniform-lamina'
      ? 'CAD 闭环薄板（按奇偶规则扣孔）'
      : (m.weaponMassPropertySource === 'parametric-uniform-lamina' ? '参数齿实体薄板' : (m.weaponMassPropertySource === 'cad-uniform-boundary' ? '开放 CAD 均匀线框近似' : '半径圆盘后备近似'));
    setText('weaponMassReadout', `当前旋转组件 ${format(m.weaponMass, 6)} kg；按${shapeLabel}自动得到 I = ${format(m.inertiaInput, 3)} kg·mm²、回转半径 ${format(m.weaponRadiusOfGyration * 1000, 3)} mm；${massConstraint}质量会按画面几何均匀分布，若轮毂、带轮、轴或紧固件的真实分布明显不同，应以实测惯量模型替代本估算。初始质量示例 ${format(DEFAULT_WEAPON_MASS, 6)} kg。`);
    $('#weaponMassReadout')?.classList.toggle('warning', !m.weaponMassConsistent);
    const runtimeMode = runtimeContactMode();
    const modelLabel = runtimeMode === 'rigid-upper-bound'
      ? '理想刚体上限对照'
      : (runtimeMode === 'material-model-domain-gated'
        ? '有限材料安全门（首次承载接触前回滚停止）'
        : (runtimeMode === 'no-removal-boundary'
        ? '非穿透零删料边界'
        : (runtimeMode === 'cutting-safety-gated'
          ? '可追溯切削安全门（非穿透零删料）'
          : (runtimeMode === 'finite-plastic-edge' ? '参数齿有限塑性接触（零删料）' : '可追溯局部材料切削'))));
    const minChipLabel = material.minChipThickness > 0 && material.minChipSource
      ? `${format(material.minChipThickness * 1000, 5)} mm（${material.minChipSource}）`
      : '未知（非穿透零删料边界）';
    setText('materialModelReadout', `${modelLabel}；靶材 ${material.label}：ρ ${format(material.density, 0)} kg/m³、E ${format(material.youngModulus)} GPa（资料值，当前未参与求解）、屈服 ${format(material.yieldStrength)} MPa、剪切 ${format(material.shearStrength)} MPa、Gc ${format(material.fractureEnergy, 0)} J/m²${material.fractureEnergy > 0 ? '' : '（未给，不能作真实断裂量）'}、h_min ${minChipLabel}。武器/叉子有效 Z 重叠 ${format(effectiveToolWidthZ('weapon') * 1000)} / ${format(effectiveToolWidthZ('fork') * 1000)} mm。来源：${material.source}；边界：${material.validity}。`);
    setText('simTime', `t = ${state.sim.time.toFixed(4)} s`);
    const validity = ' · Rapier 2D 联立刚体 + 真实 DXF 实体接触';
    const weaponStartLabel = state.params.weaponStartMode === 'setSpeed' ? `武器初速 ${format(m.weaponCommandFraction * 100, 0)}%` : '武器真实加速';
    const driveStartLabel = state.params.driveStartMode === 'setSpeed' ? `行驶初速 ${format(m.driveCommandFraction * 100, 0)}%` : '行驶真实加速';
    setText('stepReadout', `固定步长 ${(FIXED_DT * 1000).toFixed(1)} ms · 瞬时车速 ${format(m.instantaneousDriveSpeed)} m/s · ${weaponStartLabel} · ${driveStartLabel} · 理想满转时间下限 ${format(m.spinupLowerBound)} s${validity}`);
    const pivot = state.drawings.weapon.pivot;
    const baseOrigin = weaponBaseSceneOrigin(); const sceneOrigin = weaponSceneOrigin(); const clearance = groundClearanceStatus();
    setText('axisReadout', `武器局部原点 / 当前轴心：DXF X ${format(pivot.x * 1000, 2)} mm · Y ${format(pivot.y * 1000, 2)} mm；场地初始 X ${format(baseOrigin.x * 1000, 1)} · Y ${format(baseOrigin.y * 1000, 1)} mm，前进后 X ${format(sceneOrigin.x * 1000, 1)} mm。叉子 X/Y 只相对武器局部原点；刀/叉最小离地 ${format(Math.min(clearance.weaponClearance, clearance.shovelClearance) * 1000, 1)} mm。`);
    if (state.params.paramWeaponEnabled) {
      const geometry = parameterWeaponGeometry();
      const inputRakeDeg = number(state.params.paramToothPhaseDeg);
      const designNote = Math.abs(inputRakeDeg) <= 15
        ? '处于文档所述常用 0–15° 范围内'
        : (Math.abs(inputRakeDeg) > 20 ? '超过 20°，需单独校核齿根支撑、耐久和材料工况' : '介于常用与大前角区域，请校核齿根支撑');
      setText('paramWeaponGeometryReadout', `当前：${geometry.toothCount} 齿等分 · R ${format(geometry.outerRadius * 1000, 2)} mm · 迎击面 L ${format(geometry.workLength * 1000, 2)} mm · 背撑 t ${format(geometry.width * 1000, 2)} mm · 最终装配物理前角 ${format(inputRakeDeg, 2)}°。镜像已在构形时补偿，不改变正负语义；${designNote}；第 0 齿圆周方位仅由武器初始角度控制。`);
    }
    if (state.params.paramForkEnabled) {
      const geometry = parameterForkGeometry();
      const forkBounds = getShovelBoundsWorld();
      setText('paramForkGeometryReadout', `当前理想线叉：O (0, 0) mm · 尖端 T (${format(geometry.tip.x * 1000, 2)}, 0) mm · 长度 ${format(geometry.tipDistance * 1000, 2)} mm · 世界角度 ${format(forkWorldAngle() * 180 / Math.PI, 3)}° · 最低点相对真实地面 ${format((forkBounds.minY - groundY()) * 1000, 4)} mm。初始装配只绕固定铰点使 T 接地；质量由显式输入给出，质心与惯量按均匀细杆计算。`);
    }
    const preset = ROBOT_PRESETS[state.params.robotPreset] || ROBOT_PRESETS['1.36kg'];
    const weaponPreset = WEAPON_MOTOR_PRESETS[state.params.weaponMotorPreset]?.label || '自定义武器';
    const drivePreset = DRIVE_MOTOR_PRESETS[state.params.driveMotorPreset]?.label || '自定义行驶';
    setText('presetReadout', `${preset.label}：${weaponPreset}；${drivePreset}。靶子与全部电机数值均为可编辑工程条件。`);
    updateCollisionReadout(); updateTransport();
  }

  function updateCollisionReadout() {
    const sim = state.sim; const live = sim.currentContact && sim.currentContact.impact; const impact = live || sim.lastImpact; const readout = $('#impactReadout');
    const failureDomain = resolvedFailureDomain(sim);
    readout.classList.toggle('active', Boolean(live));
    setText('targetPositionX', format(sim.target.pos.x * 1000, 2));
    setText('targetPositionY', format(sim.target.pos.y * 1000, 2));
    setText('targetFlightSpeed', format(length(sim.target.vel)));
    setText('targetSpin', format(sim.target.omega));
    setText('hitCount', String(sim.hitCount));
    setText('bodyImpactCount', String(sim.bodyImpactCount));
   if (!impact || failureDomain) {
     const stoppedByMaterialDomain = failureDomain === 'material-model';
     const stoppedBySolverDomain = failureDomain === 'rigid-solver';
     const stoppedByInternalDomain = failureDomain === 'internal';
     setText('collisionState', stoppedByInternalDomain
       ? '内部异常安全停止'
       : (stoppedByMaterialDomain
         ? '材料模型安全停止'
         : (stoppedBySolverDomain ? '刚体求解安全停止' : '等待接触')));
     setText('impactSummary', stoppedByInternalDomain
       ? `计算内部异常；末固定步已事务回滚，异常尝试态未提交。原因：${sim.lastRigFailure?.invalidRoles?.join('；') || sim.creationError || '请查看事件记录'}。`
       : (stoppedByMaterialDomain
         ? '已检测到首次承载接触，但当前材料模型还不能可靠计算该接触；整个末固定步已回滚，临时刚体冲量、靶子速度和能量均未提交。这不是刀或叉被保留在靶内。'
         : (stoppedBySolverDomain
           ? `末固定步未通过 0.08 mm 几何与关节收敛硬门，已事务回滚并停止；原因：${sim.lastRigFailure?.invalidRoles?.join('；') || '请查看事件记录'}。`
           : (sim.forkEngaged ? '叉子真实前缘接触，等待刀齿越过' : '靶子正相对武器接近'))));
      ['contactPoint', 'penetrationDepth', 'relativeSpeed', 'impactEnergy', 'impactImpulse', 'geometryPenetration', 'contactGap', 'rotorEnergyLoss', 'energyResidual', 'constraintEnergy', 'chipEnergy', 'energyValidity', 'effectiveWidthZ', 'uncutThickness', 'cuttingForce', 'materialWork', 'boundaryWork', 'materialValidity'].forEach((id) => setText(id, '—'));
     if (stoppedByInternalDomain) {
       setText('energyValidity', '内部异常；失败固定步已回滚，未提交尝试态');
       setText('materialValidity', '内部异常停止；不作刚体收敛或材料响应结论');
     } else if (stoppedByMaterialDomain) {
       setText('energyValidity', '未计算；末固定步已完整回滚');
       setText('materialValidity', '缺少经连续接触验证的材料响应；未作击飞或切削结论');
     } else if (stoppedBySolverDomain) {
       setText('energyValidity', '未提交失败尝试态');
       setText('materialValidity', '刚体数值域停止；不得把冻结画面当作稳定接触');
     }
     return;
    }
    if (impact.blocking) {
      setText('collisionState', `刀体 / 背板碰撞 #${impact.bodyImpactNumber ?? sim.bodyImpactCount}`);
      setText('impactSummary', `source ${impact.sourceIndex ?? '未知'}，法向冲量 0 N·s；真实 DXF 外轮廓先接触，缺少该处厚度、柔度和材料失效数据，工况已在边界暂停；未计入牙齿命中或 bite。`);
      setText('contactPoint', `${format(impact.contact.x * 1000, 0)}, ${format(impact.contact.y * 1000, 0)}`);
      setText('penetrationDepth', '0'); setText('relativeSpeed', format(Math.abs(impact.normalVelocity)));
      setText('impactEnergy', '—'); setText('impactImpulse', '—'); setText('geometryPenetration', '0'); setText('contactGap', '0'); setText('rotorEnergyLoss', '—'); setText('energyResidual', '—'); setText('constraintEnergy', '—'); setText('chipEnergy', '—'); setText('energyValidity', '不适用');
      setText('effectiveWidthZ', '—'); setText('uncutThickness', '—'); setText('cuttingForce', '—'); setText('materialWork', '—'); setText('boundaryWork', '—'); setText('materialValidity', '非识别齿段；未作切削结论');
      return;
    }
    setText('collisionState', impact.bodyContact
      ? `${live ? '当前' : '上次'}刀体 / 背板碰撞 #${impact.bodyImpactNumber ?? sim.bodyImpactCount} · ${impact.time.toFixed(3)} s`
      : (live ? `当前牙齿命中 #${impact.toothHitNumber ?? sim.hitCount}` : `上次牙齿命中 #${impact.toothHitNumber ?? sim.hitCount} · ${impact.time.toFixed(3)} s`));
    const energyLedger = Number.isFinite(impact.targetEnergyGain)
      ? `靶动能 ${impact.targetEnergyGain >= 0 ? '+' : ''}${format(impact.targetEnergyGain)} J；转子${impact.rotorEnergyLoss >= 0 ? '损失' : '增加'} ${format(Math.abs(impact.rotorEnergyLoss))} J`
      : '保底 TOI 路径读数';
    setText('impactSummary', impact.bodyContact
      ? `主 source ${impact.sourceIndex ?? '未知'}，episode 总法向冲量 ${format(impact.normalImpulse, 6)} N·s；真实 DXF 刀体 / 背板发生刚体接触；未计入牙齿命中或 bite；${energyLedger}`
      : `主 source ${impact.sourceIndex ?? '未知'}，episode 总法向冲量 ${format(impact.normalImpulse, 6)} N·s；${sim.targetLaunched ? '靶子已离开等效支撑；' : ''}${energyLedger}`);
    setText('contactPoint', `${format(impact.contact.x * 1000, 0)}, ${format(impact.contact.y * 1000, 0)}`);
    setText('penetrationDepth', Number.isFinite(impact.feedBite) ? format(impact.feedBite * 1000) : '—'); setText('relativeSpeed', format(Math.abs(impact.normalVelocity)));
    setText('impactEnergy', format(impact.transferredEnergy)); setText('impactImpulse', format(impact.impulse));
    setText('geometryPenetration', format(impact.penetration * 1000));
    setText('contactGap', Number.isFinite(impact.contactGap) ? format(impact.contactGap * 1000, 4) : '—');
    setText('rotorEnergyLoss', Number.isFinite(impact.rotorEnergyLoss) ? format(impact.rotorEnergyLoss) : '—');
    const residual = Number.isFinite(impact.unclassifiedEnergy) ? impact.unclassifiedEnergy : null;
    setText('energyResidual', Number.isFinite(residual) ? format(residual) : '—');
    setText('constraintEnergy', Number.isFinite(impact.constraintEnergyExchange) ? format(impact.constraintEnergyExchange) : '—');
    setText('chipEnergy', Number.isFinite(impact.massRemovalEnergy) ? format(impact.massRemovalEnergy) : '—');
    const energyValidity = impact.episodeComplete
      ? (impact.energyConverged
        ? `通过：|残差| ≤ ${format(impact.energyTolerance, 5)} J`
        : `未收敛：数值增能/残差超过 ${format(impact.energyTolerance, 5)} J，切削与击飞量无效`)
      : '接触尚未结束，等待完整能量账';
    setText('energyValidity', energyValidity);
    setText('effectiveWidthZ', Number.isFinite(impact.effectiveWidthZ) ? format(impact.effectiveWidthZ * 1000) : '—');
    setText('uncutThickness', Number.isFinite(impact.uncutThickness) ? format(impact.uncutThickness * 1000) : '—');
    setText('cuttingForce', Number.isFinite(impact.cuttingForce) ? format(impact.cuttingForce) : '—');
    setText('materialWork', Number.isFinite(impact.materialWork) ? format(impact.materialWork) : '—');
    setText('boundaryWork', Number.isFinite(impact.boundaryWork) ? format(impact.boundaryWork) : '—');
    const materialValidity = impact.materialValidity || (impact.contactModel === 'rigid' ? '理想刚体上限' : '—');
    setText('materialValidity', impact.episodeComplete && !impact.energyConverged
      ? `${materialValidity}；本次能量账未通过数值收敛门，定量结果不得用于理论结论`
      : materialValidity);
  }

  function updateTransport() {
    const sim = state.sim; const button = $('#playPause');
    const step = $('#stepSimulation'); const seek = $('#timelineSeek');
    const building = Boolean(state.trajectory?.building);
    if (building) {
      button.disabled = true; if (step) step.disabled = true; if (seek) seek.disabled = true;
      button.querySelector('.play-symbol').textContent = '…';
      button.querySelector('span:last-child').textContent = '计算中';
      $('.shovel-tick').style.opacity = '.35'; $('.weapon-tick').style.opacity = '.35';
      return;
    }
    button.disabled = false; if (step) step.disabled = false; if (seek) seek.disabled = false;
    button.querySelector('.play-symbol').textContent = sim.running ? 'Ⅱ' : '▶'; button.querySelector('span:last-child').textContent = sim.running ? '暂停' : '播放';
    const endTime = simulationEndTime();
    const timelineTime = clamp(sim.time, 0, endTime);
    $('#timelineProgress').style.width = `${clamp(timelineTime / endTime * 100, 0, 100)}%`;
    // The public horizon can end on any 0.5 ms physics tick.  Keep the range in
    // milliseconds for the UI, but do not round those half-ticks to whole ms.
    if (seek) {
      const millisecondsPerTick = FIXED_DT * 1000;
      seek.max = String(Math.round(endTime / FIXED_DT) * millisecondsPerTick);
      seek.value = String(Math.round(timelineTime / FIXED_DT) * millisecondsPerTick);
    }
    setText('timelineReadout', `${timelineTime.toFixed(4)} / ${endTime.toFixed(4)} s`);
    setText('timelineEndLabel', `${endTime.toFixed(4)} s`);
    $('.shovel-tick').style.opacity = sim.forkContact ? '1' : '.35'; $('.weapon-tick').style.opacity = (sim.hitCount || sim.bodyImpactCount) ? '1' : '.35';
  }

  function updateStatus(text, type = 'ready') { const status = $('#appStatus'); status.className = `status-pill ${type}`; status.querySelector('span').textContent = text; }
  function showToast(message, kind = '') { const toast = $('#toast'); toast.textContent = message; toast.className = `toast ${kind}`; requestAnimationFrame(() => toast.classList.add('show')); clearTimeout(state.toastTimer); state.toastTimer = setTimeout(() => toast.classList.remove('show'), 3200); }
  function activeEventEntries() {
    return state.activeRigTick?.pendingEvents
      ?? (state.trajectory?.building ? state.trajectory.events : state.eventEntries);
  }

  function renderEventEntries(entries = state.eventEntries, cursor = entries.length, floor = 0) {
    const log = $('#eventLog');
    if (!log) return;
    log.replaceChildren();
    const first = Math.max(floor, cursor - 14);
    entries.slice(first, cursor).reverse().forEach((entry) => {
      const li = document.createElement('li'); li.dataset.type = entry.type;
      const time = document.createElement('time'); time.textContent = `${number(entry.time).toFixed(3)} s`;
      const copy = document.createElement('span'); copy.textContent = entry.text;
      li.append(time, copy); log.append(li);
    });
  }

  function addEvent(text, type = 'info') {
    const entries = activeEventEntries();
    entries.push({ time: state.sim ? number(state.sim.time) : 0, text: String(text), type });
    // A build owns an isolated event stream. Future events are revealed only
    // when playback reaches their cached frame.
    if (!state.activeRigTick && !state.trajectory?.building) renderEventEntries(entries);
  }

  function clearEvents() {
    const trajectory = state.trajectory;
    if (trajectory?.building) {
      trajectory.events.length = 0; state.eventEntries = []; renderEventEntries();
      return;
    }
    if (trajectory?.ready) {
      const frame = trajectory.frames[state.playheadTick];
      trajectory.eventFloor = frame?.eventCursor ?? 0;
      renderEventEntries(trajectory.events, frame?.eventCursor ?? 0, trajectory.eventFloor);
      return;
    }
    state.eventEntries = [];
    renderEventEntries();
  }

  function normaliseEngineeringReadouts() {
    const theoreticalEnergy = $('#weaponEnergy');
    const theoreticalCard = theoreticalEnergy?.closest('.metric-card');
    if (theoreticalCard) {
      const label = theoreticalCard.querySelector('span');
      if (label) label.textContent = '理论满转储能';
      if (!$('#liveWeaponEnergy')) {
        const liveCard = document.createElement('article');
        liveCard.className = 'metric-card accent-violet';
        const liveLabel = document.createElement('span'); liveLabel.textContent = '实时武器动能';
        const value = document.createElement('strong'); value.id = 'liveWeaponEnergy'; value.textContent = '—';
        const unit = document.createElement('small'); unit.textContent = 'J';
        liveCard.append(liveLabel, value, unit); theoreticalCard.insertAdjacentElement('afterend', liveCard);
      }
    }
    const ratio = $('[data-param="weaponGearRatio"]');
    if (ratio) {
      ratio.title = 'i = 电机转速 ÷ 武器转速；100:1 减速填 100，1:100 加速填 0.01。';
      const field = ratio.closest('.field');
      if (field && !field.querySelector('.gear-ratio-hint')) {
        const hint = document.createElement('small'); hint.className = 'gear-ratio-hint';
        hint.textContent = 'i=电机÷武器；100=100:1减速，0.01=1:100加速';
        field.append(hint);
      }
    }
  }

  function updateGeometryScaleWarning() {
    const warning = $('#geometryScaleWarning');
    if (!warning) return;
    const presetKey = state.appliedRobotPreset;
    const retainedCad = [];
    if (!state.params.paramWeaponEnabled) retainedCad.push('武器');
    if (!state.params.paramForkEnabled) retainedCad.push('叉子');
    const mismatched = Boolean(presetKey && presetKey !== '1.36kg' && retainedCad.length);
    warning.classList.toggle('hidden', !mismatched);
    if (!mismatched) {
      warning.textContent = '';
      return;
    }
    const label = ROBOT_PRESETS[presetKey]?.label || presetKey;
    warning.textContent = `几何未缩放：已套用“${label}”的质量、动力与推荐靶参数，但${retainedCad.join('和')}的当前 DXF 仍保留原始尺寸。请导入对应量级图纸，或显式启用并填写测试几何。`;
  }

  function syncParametricUi() {
    const weaponEnabled = Boolean(state.params.paramWeaponEnabled);
    const forkEnabled = Boolean(state.params.paramForkEnabled);
    const shell = $('#appShell');
    if (shell) {
      shell.classList.toggle('param-weapon-enabled', weaponEnabled);
      shell.classList.toggle('param-fork-enabled', forkEnabled);
      shell.dataset.paramWeaponEnabled = String(weaponEnabled);
      shell.dataset.paramForkEnabled = String(forkEnabled);
    }
    $('#paramWeaponSettings')?.classList.toggle('hidden', !weaponEnabled);
    $('#paramForkSettings')?.classList.toggle('hidden', !forkEnabled);
    $('#weaponDrawingCard')?.classList.toggle('parametric-active', weaponEnabled);
    $('#shovelDrawingCard')?.classList.toggle('parametric-active', forkEnabled);
    updateGeometryScaleWarning();
    const axisButton = $('#setAxis');
    if (axisButton) {
      axisButton.disabled = weaponEnabled;
      axisButton.title = weaponEnabled
        ? '测试齿的转轴固定为局部 (0,0) mm；关闭测试齿后可继续点选 DXF 转轴。'
        : '在画布上选择武器 DXF 转轴。';
    }
    ['shovelOffsetX', 'shovelOffsetY'].forEach((key) => {
      const input = $(`[data-param="${key}"]`);
      if (!input) return;
      input.disabled = false;
      input.title = '叉子铰点/局部原点相对武器原点的固定坐标；世界姿态由重力和接触决定。';
    });
    if (weaponEnabled && state.axisPicking) setAxisPicking(false);
  }

  function syncInputs() {
    synchroniseDerivedWeaponRadius();
    synchroniseAutomaticTargetMass();
    // Keep the compatibility parameter and read-only field synchronized before
    // the generic input pass. This never changes the user's weaponMass draft.
    if (state.drawings.weapon) automaticWeaponMassProperties();
    $$('[data-param]').forEach((input) => {
      // A numeric field is a draft while the user is typing.  Keep that draft
      // (including an empty string) out of the committed physics state and do
      // not overwrite it if an unrelated UI refresh happens before commit.
      const activeNumericDraft = input.type === 'number'
        && document.activeElement === input
        && input.dataset.numericDraft === 'dirty';
      if (activeNumericDraft) return;
      const value = state.params[input.dataset.param];
      if (input.type === 'checkbox') input.checked = Boolean(value);
      else if (value !== undefined) input.value = value;
      if (input.type === 'number') delete input.dataset.numericDraft;
    });
    syncParametricUi();
    const targetMassInput = $('[data-param="targetMass"]');
    if (targetMassInput) {
      targetMassInput.disabled = state.params.targetMassMode === 'geometry';
      const activeManualDraft = !targetMassInput.disabled
        && document.activeElement === targetMassInput
        && targetMassInput.dataset.numericDraft === 'dirty';
      if (!activeManualDraft) {
        targetMassInput.value = state.params.targetMass;
        if (targetMassInput.disabled) delete targetMassInput.dataset.numericDraft;
      }
      targetMassInput.title = targetMassInput.disabled ? '当前数值由 X×Y×Z 尺寸与材料密度实时计算并自动回填；切回实测/等效质量后可编辑。' : '靶子及其夹具参与运动的实测/等效质量。';
    }
    const inertiaField = $('[data-param="weaponInertia"]');
    if (inertiaField) {
      inertiaField.step = '0.1'; inertiaField.min = '0.001';
      inertiaField.readOnly = true; inertiaField.setAttribute('aria-readonly', 'true');
      inertiaField.value = state.params.weaponInertia;
      inertiaField.title = '只读：由武器旋转组件质量和当前 CAD/参数齿几何绕武器原点自动积分计算。';
      delete inertiaField.dataset.numericDraft;
      const unit = inertiaField.parentElement?.querySelector('b');
      if (unit) unit.textContent = 'kg·mm²';
    }
    const weaponMassField = $('[data-param="weaponMass"]');
    if (weaponMassField) {
      weaponMassField.step = '0.001'; weaponMassField.min = '0.000001';
      weaponMassField.title = '唯一可编辑的武器质量参数：完整旋转组件（刀体、轮毂、紧固件、带轮/轴中随武器刚体运动的部分）。转动惯量按当前几何自动重算。';
    }
    const radiusInput = $('[data-param="tipRadius"]');
    if (radiusInput) {
      const parameterMode = Boolean(state.params.paramWeaponEnabled);
      const locked = parameterMode || hasClosedWeaponCadOutline();
      radiusInput.disabled = locked;
      radiusInput.title = parameterMode
        ? '测试齿使用“轴心至齿尖外缘半径”，此 DXF 后备半径不参与当前求解。'
        : (locked ? '闭合 DXF 的实际轮廓已锁定此半径；请在图纸中修改刀齿几何。' : '未闭合 DXF 时可作为速度与 bite 的后备半径。');
      const deriveButton = $('#useDerivedRadius');
      if (deriveButton) {
        deriveButton.disabled = locked;
        deriveButton.textContent = parameterMode ? '当前使用测试半径' : (locked ? '已由图纸锁定' : '使用图纸推导半径');
        deriveButton.title = parameterMode ? '关闭测试齿后可恢复 DXF 半径操作。' : (locked ? '闭合 DXF 已使用真实 CAD 外轮廓推导半径。' : '用当前图纸的最大半径更新后备读数。');
      }
    }
    updateMotorPresetNotes();
  }
  const PARAMETRIC_GEOMETRY_PARAMS = new Set([
    'paramWeaponEnabled', 'paramForkEnabled', 'paramToothCount', 'paramToothPhaseDeg', 'paramToothLength', 'paramToothWorkLength', 'paramToothWidth',
    'paramForkTipDistance', 'edgeRadius',
  ]);
  const GROUND_AFFECTING_PARAMS = new Set([
    'weaponSceneY', 'shovelOffsetY', 'tipRadius', 'weaponMirrorY', 'shovelMirrorY',
    'paramWeaponEnabled', 'paramForkEnabled', 'paramToothLength', 'paramToothWorkLength', 'paramToothWidth', 'paramToothPhaseDeg',
    'paramForkTipDistance',
  ]);
  function clearanceMessage(status) {
    const offenders = [];
    if (!status.forkPose?.valid && status.forkPose?.reason) offenders.push(status.forkPose.reason);
    if (status.weaponClearance < -GEOMETRY_CLEARANCE_EPS) offenders.push(`武器旋转包络低于地板 ${format(-status.weaponClearance * 1000, 2)} mm`);
    if (status.shovelClearance < -GEOMETRY_CLEARANCE_EPS) offenders.push(`叉子低于地板 ${format(-status.shovelClearance * 1000, 2)} mm`);
    return `${offenders.join('；')}。场地地面固定为 Y = 0 mm。`;
  }
  function flashInvalid(input, message) {
    const holder = input?.closest?.('.field, .mirror-mini') || input;
    if (holder) {
      holder.classList.remove('invalid');
      // Restart the animation when a user retries the same invalid value.
      void holder.offsetWidth;
      holder.classList.add('invalid');
      window.setTimeout(() => holder.classList.remove('invalid'), 900);
    }
    showToast(message, 'error');
  }
  function validateGroundClearance(input) {
    const status = groundClearanceStatus();
    if (status.valid) return true;
    flashInvalid(input, clearanceMessage(status));
    return false;
  }
  function alignGeometryToFloor() {
    const radius = state.params.paramWeaponEnabled
      ? activeWeaponRadius()
      : Math.max(positive(number(state.params.tipRadius) / 1000, .001), getMaxWeaponRadius());
    state.params.weaponSceneY = Math.round((groundY() + radius * 1.05) * 1000 * 100) / 100;
    // Fork offset is an installation dimension. Never rewrite it to create a
    // visual floor fit; the independent hinged body settles under gravity.
  }
  function setAdvancedEnabled(enabled) {
    state.advancedEnabled = Boolean(enabled);
    $('#appShell').classList.toggle('advanced-open', state.advancedEnabled);
    const button = $('#toggleAdvanced');
    button.classList.toggle('active', state.advancedEnabled);
    button.setAttribute('aria-expanded', String(state.advancedEnabled));
    button.querySelector('span').textContent = state.advancedEnabled ? '⌃' : '⌄';
    // This is deliberately presentation-only: opening or closing an editor
    // must not reset time, rebuild bodies, or select a different solver law.
    updateReadouts();
  }
  function resetAfterChange(label) { clearEvents(); resetSimulation(); addEvent(`参数已更新：${label}；仿真从 t = 0 重新开始。`); showToast('参数已实时重算，仿真已重置。'); }

  function updateMotorPresetNotes() {
    [['weapon', WEAPON_MOTOR_PRESETS], ['drive', DRIVE_MOTOR_PRESETS]].forEach(([group, source]) => {
      const preset = source[state.params[`${group}MotorPreset`]] || source.custom;
      const target = $(`#${group}MotorPresetNote`);
      if (!target) return;
      const sourceText = preset.source ? `来源/状态：${preset.source}。` : '';
      target.textContent = `${sourceText}${preset.condition || '请按具体型号和实测条件填写。'}`;
      const links = [[preset.sourceUrl, '官方来源'], [preset.dataUrl, '原始数据']]
        .filter(([url]) => Boolean(url));
      links.forEach(([url, label], index) => {
        const anchor = document.createElement('a');
        anchor.href = url;
        anchor.target = '_blank';
        anchor.rel = 'noopener noreferrer';
        anchor.textContent = label;
        target.append(document.createTextNode(index ? '｜' : ' '), anchor);
      });
    });
  }

  function applyMotorPreset(group, key) {
    const source = group === 'weapon' ? WEAPON_MOTOR_PRESETS : DRIVE_MOTOR_PRESETS; const preset = source[key];
    if (!preset || key === 'custom') { updateMotorPresetNotes(); return; }
    if (preset.apply === false) {
      updateMotorPresetNotes();
      showToast(preset.condition, 'warning');
      return;
    }
    const metadata = new Set(['label', 'apply', 'source', 'sourceUrl', 'dataUrl', 'condition']);
    Object.entries(preset).forEach(([name, value]) => { if (!metadata.has(name)) state.params[name] = value; });
    updateMotorPresetNotes();
  }
  function applyRobotPreset() {
    const preset = ROBOT_PRESETS[state.params.robotPreset]; if (!preset) return;
    const legacyPresetInertia = positive(preset.weaponInertia, 0) * 1e-6;
    Object.entries(preset).forEach(([name, value]) => { if (name !== 'label' && name !== 'targetCenterY' && name !== 'restitution' && name !== 'weaponInertia') state.params[name] = value; });
    if (state.params.robotPreset !== '110kg') {
      // The supplied 55 mm CAD belongs to the 1.36 kg robot. Only the 110 kg
      // example deliberately switches to its explicit 250 mm parameter tool;
      // returning to another class restores the retained CAD sources.
      state.params.paramWeaponEnabled = false;
      state.params.paramForkEnabled = false;
      state.params.targetSceneX = DEFAULTS.targetSceneX;
    }
    const targetWidthByClass = { '150g': 15, '220g': 20, '454g': 30, '1.36kg': 40, '5lb': 50, '13.6kg': 80, '110kg': 200 };
    // Migrate old presets without keeping a second editable degree of freedom:
    // when a preset has no explicit mass, choose the mass which makes its active
    // geometry reproduce the historical inertia. Future edits then use mass only.
    if (!Number.isFinite(Number(preset.weaponMass)) && legacyPresetInertia > 0) {
      const shape = activeWeaponMassShape();
      if (shape.radiusOfGyrationSquared > 1e-12) state.params.weaponMass = legacyPresetInertia / shape.radiusOfGyrationSquared;
    }
    automaticWeaponMassProperties();
    state.params.targetWidthZ = targetWidthByClass[state.params.robotPreset] ?? state.params.targetWidthZ;
    applyTargetMaterialPreset(state.params.targetMaterial);
    // A closed imported CAD outline remains the geometry source of truth.
    synchroniseDerivedWeaponRadius();
    // Keep the imported CAD envelope and fork on/above the fixed floor.
    alignGeometryToFloor();
    const toolFront = Math.max(
      weaponBaseSceneOrigin().x + activeWeaponRadius(),
      getShovelFrontWorld(),
    );
    const minimumTargetCentre = toolFront + targetLength() / 2 + .01;
    state.params.targetSceneX = Math.max(number(state.params.targetSceneX) / 1000, minimumTargetCentre) * 1000;
    state.appliedRobotPreset = state.params.robotPreset;
    syncInputs(); resetAfterChange(`${preset.label} 量级与推荐靶子`);
  }

  function handleParameterChange(event) {
    const input = event.currentTarget; const key = input.dataset.param; if (!key) return;
    const previous = state.params[key];
    const textual = ['dxfUnit', 'targetMaterial', 'weaponMaterial', 'shovelMaterial', 'floorMaterial', 'robotPreset', 'weaponMotorPreset', 'driveMotorPreset', 'weaponStartMode', 'driveStartMode', 'contactModel', 'targetMassMode', 'targetFractureSource', 'targetMinChipSource'];
    if (input.type === 'checkbox') state.params[key] = input.checked;
    else if (textual.includes(key)) state.params[key] = input.value;
    else {
      const parsed = Number(input.value);
      if (input.value === '' || !Number.isFinite(parsed) || !input.checkValidity()) {
        const wasBlank = input.value.trim() === '';
        input.value = previous;
        const label = input.closest('label')?.querySelector('span')?.textContent?.trim() || key;
        if (wasBlank) {
          // An empty field is a normal intermediate editing state.  If focus
          // leaves before the user enters a replacement, quietly restore the
          // last committed value instead of treating deletion as an error.
          input.closest('.field')?.classList.remove('invalid');
          return;
        }
        const reason = wasBlank ? '不能为空' : '格式或范围无效';
        flashInvalid(input, `${label}${reason}；已恢复为 ${previous}`);
        return;
      }
      state.params[key] = parsed;
    }
    const automaticMassChanged = synchroniseAutomaticTargetMass();
    if (automaticMassChanged) {
      const targetMassInput = $('[data-param="targetMass"]');
      if (targetMassInput && document.activeElement !== targetMassInput) targetMassInput.value = state.params.targetMass;
    }
    syncParametricUi();
    if (PARAMETRIC_GEOMETRY_PARAMS.has(key)) {
      const geometryStatus = parameterGeometryStatus();
      if (!geometryStatus.valid) {
        state.params[key] = previous;
        syncInputs(); updateReadouts(); drawScene(); flashInvalid(input, geometryStatus.reason);
        return;
      }
      if (key === 'paramWeaponEnabled' || key === 'paramForkEnabled') renderDrawingStatus();
    }
    if (key === 'tipRadius' && !state.params.paramWeaponEnabled && hasClosedWeaponCadOutline()) {
      synchroniseDerivedWeaponRadius(); syncInputs(); showToast('闭合 DXF 已锁定图纸推导半径；牙形与碰撞不使用手工半径。'); return;
    }
    if (key === 'timeScale') { updateReadouts(); return; }
    if (GROUND_AFFECTING_PARAMS.has(key) && !validateGroundClearance(input)) {
      state.params[key] = previous;
      syncInputs(); updateReadouts(); drawScene();
      return;
    }
    if (key === 'robotPreset') { updateReadouts(); showToast('已选择量级；点击“套用量级 + 推荐靶子”应用。'); return; }
    if (key === 'targetMaterial') { applyTargetMaterialPreset(input.value); synchroniseAutomaticTargetMass(); syncInputs(); resetAfterChange('靶子材料与可追溯强度预设'); return; }
    if (key === 'targetMassMode') { synchroniseAutomaticTargetMass(); syncInputs(); resetAfterChange('靶子质量采用方式'); return; }
    if (key === 'weaponMotorPreset') { applyMotorPreset('weapon', input.value); syncInputs(); resetAfterChange('武器电机方案'); return; }
    if (key === 'driveMotorPreset') { applyMotorPreset('drive', input.value); syncInputs(); resetAfterChange('行驶电机方案'); return; }
    if (key === 'dxfUnit') {
      reprocessImportedDrawings();
      if (!validateGroundClearance(input)) {
        state.params[key] = previous; reprocessImportedDrawings(); syncInputs(); updateReadouts(); drawScene();
        return;
      }
      syncInputs(); resetAfterChange('DXF 绘图单位'); return;
    }
    resetAfterChange(input.closest('.field')?.querySelector('span')?.textContent?.trim() || key);
  }

  function modal(name, visible) { $('#modalBackdrop').classList.toggle('hidden', !visible); $('#dwgModal').classList.add('hidden'); $('#notesModal').classList.add('hidden'); if (visible) $(`#${name}`).classList.remove('hidden'); }
  function setAxisPicking(enabled) { state.axisPicking = enabled; $('#canvasWrap').classList.toggle('axis-picking', enabled); $('#axisHint').classList.toggle('hidden', !enabled); const button = $('#setAxis'); button.classList.toggle('primary', enabled); button.classList.toggle('secondary', !enabled); button.textContent = enabled ? '取消设置转轴' : '在画布上设置转轴'; if (enabled) showToast('点击武器轮廓上的位置，将其设为新的武器局部原点 / 转轴；场景原点不会移动。'); }

  // ASCII DXF importer. DWG stays intentionally unsupported without an external adapter.
  function pairsFromDxf(text) { const lines = text.replace(/^\uFEFF/, '').split(/\r?\n/); const pairs = []; for (let i = 0; i + 1 < lines.length; i += 2) { const code = Number.parseInt(lines[i].trim(), 10); if (Number.isFinite(code)) pairs.push({ code, value: lines[i + 1].trim() }); } return pairs; }
  function detectedDxfUnit(text) { const pairs = pairsFromDxf(text); for (let i = 0; i < pairs.length; i += 1) if (pairs[i].value === '$INSUNITS') for (let j = i + 1; j < Math.min(i + 12, pairs.length); j += 1) if (pairs[j].code === 70) return INSUNITS[Number(pairs[j].value)] || null; return null; }
  function entityChunks(text) {
    const pairs = pairsFromDxf(text); const chunks = []; let section = null; let index = 0;
    while (index < pairs.length) { const pair = pairs[index]; if (pair.code === 0 && pair.value.toUpperCase() === 'SECTION') { section = pairs[index + 1]?.code === 2 ? pairs[index + 1].value.toUpperCase() : null; index += 2; continue; } if (pair.code === 0 && pair.value.toUpperCase() === 'ENDSEC') { section = null; index += 1; continue; } if (section === 'ENTITIES' && pair.code === 0) { const entityPairs = []; const type = pair.value.toUpperCase(); index += 1; while (index < pairs.length && pairs[index].code !== 0) entityPairs.push(pairs[index++]); chunks.push({ type, pairs: entityPairs }); continue; } index += 1; }
    return chunks;
  }
  function firstNumber(pairs, code, fallback = NaN) { const found = pairs.find((pair) => pair.code === code); return found ? number(found.value, fallback) : fallback; }
  function xyFromPairs(pairs) { const points = []; let current = null; pairs.forEach((pair) => { if (pair.code === 10) { if (current && Number.isFinite(current.x) && Number.isFinite(current.y)) points.push(current); current = { x: number(pair.value, NaN), y: NaN }; } else if (pair.code === 20 && current) { current.y = number(pair.value, NaN); if (Number.isFinite(current.x) && Number.isFinite(current.y)) { points.push(current); current = null; } } }); return points; }
  const REJECTED_DXF_GEOMETRY_TYPES = new Set([
    'SPLINE', 'ELLIPSE', 'INSERT', 'MINSERT', 'MLINE', 'HATCH', 'SOLID', '3DFACE', 'TRACE', 'REGION', 'BODY', 'ACAD_PROXY_ENTITY',
  ]);
  function hasNonzeroDxfBulge(pairs) {
    return pairs.some((pair) => pair.code === 42 && Math.abs(number(pair.value, 0)) > 1e-12);
  }
  function parseDxf(text) {
    if (!/\bSECTION\b/i.test(text) || !/\bENTITIES\b/i.test(text)) throw new Error('未找到 ASCII DXF 的 ENTITIES 段。');
    const chunks = entityChunks(text); const paths = []; const unsupported = new Set(); const rejectedGeometry = new Set();
    for (let i = 0; i < chunks.length; i += 1) { const e = chunks[i];
      if (REJECTED_DXF_GEOMETRY_TYPES.has(e.type)) { rejectedGeometry.add(e.type); continue; }
      if (e.type === 'LINE') { const start = point(firstNumber(e.pairs, 10), firstNumber(e.pairs, 20)); const end = point(firstNumber(e.pairs, 11), firstNumber(e.pairs, 21)); if ([start.x, start.y, end.x, end.y].every(Number.isFinite)) paths.push({ type: 'line', start, end }); }
      else if (e.type === 'LWPOLYLINE') { if (hasNonzeroDxfBulge(e.pairs)) rejectedGeometry.add('LWPOLYLINE bulge（组码 42）'); const points = xyFromPairs(e.pairs); if (points.length > 1) paths.push({ type: 'polyline', points, closed: Boolean(firstNumber(e.pairs, 70, 0) & 1) }); }
      else if (e.type === 'POLYLINE') { const flags = firstNumber(e.pairs, 70, 0); if (flags & (8 | 16 | 64)) rejectedGeometry.add('3D / 网格 POLYLINE'); const points = []; let j = i + 1; while (j < chunks.length && chunks[j].type === 'VERTEX') { if (hasNonzeroDxfBulge(chunks[j].pairs)) rejectedGeometry.add('POLYLINE / VERTEX bulge（组码 42）'); const x = firstNumber(chunks[j].pairs, 10); const y = firstNumber(chunks[j].pairs, 20); if (Number.isFinite(x) && Number.isFinite(y)) points.push(point(x, y)); j += 1; } if (points.length > 1) paths.push({ type: 'polyline', points, closed: Boolean(flags & 1) }); i = j - 1; }
      else if (e.type === 'CIRCLE') { const center = point(firstNumber(e.pairs, 10), firstNumber(e.pairs, 20)); const radius = firstNumber(e.pairs, 40); if ([center.x, center.y, radius].every(Number.isFinite) && radius > 0) paths.push({ type: 'circle', center, radius }); }
      else if (e.type === 'ARC') { const center = point(firstNumber(e.pairs, 10), firstNumber(e.pairs, 20)); const radius = firstNumber(e.pairs, 40); const startAngle = radians(firstNumber(e.pairs, 50)); const endAngle = radians(firstNumber(e.pairs, 51)); if ([center.x, center.y, radius, startAngle, endAngle].every(Number.isFinite) && radius > 0) paths.push({ type: 'arc', center, radius, startAngle, endAngle }); }
      else if (!['VERTEX', 'SEQEND', 'EOF'].includes(e.type)) unsupported.add(e.type);
    }
    if (rejectedGeometry.size) throw new Error(`DXF 包含当前不能无损解析的轮廓几何：${[...rejectedGeometry].join('、')}。请在 CAD 中炸开 BLOCK / INSERT，并把曲线转为 LINE / ARC 或无 bulge 的 POLYLINE 后重新导入；本页不会静默改变牙形。`);
    if (!paths.length) throw new Error('DXF 中没有可用于轮廓的 LINE / POLYLINE / CIRCLE / ARC 实体。');
    return { paths, entityCount: chunks.length, unsupported: [...unsupported] };
  }
  function scalePath(path, scale) { const output = clonePath(path); if (output.points) output.points = output.points.map((p) => scalePoint(p, scale)); if (output.start) output.start = scalePoint(output.start, scale); if (output.end) output.end = scalePoint(output.end, scale); if (output.center) output.center = scalePoint(output.center, scale); if (Number.isFinite(output.radius)) output.radius *= scale; return output; }
  function selectedScale(detected) { const unit = state.params.dxfUnit === 'auto' ? (detected || 'mm') : state.params.dxfUnit; return { unit, scale: UNIT_SCALES[unit] || .001 }; }
  function createImportedDrawing(role, name, rawText) { const parsed = parseDxf(rawText); const detectedUnit = detectedDxfUnit(rawText); const scale = selectedScale(detectedUnit); return makeDrawing(parsed.paths.map((path) => scalePath(path, scale.scale)), { role, name, rawText, rawFileName: name, detectedUnit, unit: scale.unit, unitScale: scale.scale, entityCount: parsed.entityCount, unsupported: parsed.unsupported, sourceKind: 'imported', sourceFormat: `DXF · ${scale.unit}` }); }
  function reprocessImportedDrawings() { for (const role of ['shovel', 'weapon']) { const old = state.drawings[role]; if (!old?.rawText) continue; const pivotRaw = scalePoint(old.pivot, 1 / old.unitScale); const originRaw = scalePoint(old.origin, 1 / old.unitScale); const next = createImportedDrawing(role, old.rawFileName || old.name, old.rawText); next.pivot = scalePoint(pivotRaw, next.unitScale); next.origin = scalePoint(originRaw, next.unitScale); state.drawings[role] = next; } renderDrawingStatus(); }
  function drawingStatus(role) {
    const d = state.drawings[role]; const name = role === 'shovel' ? '叉子' : '武器'; const b = d.bounds;
    const parametric = role === 'shovel' ? state.params.paramForkEnabled : state.params.paramWeaponEnabled;
    if (parametric) return `当前求解：${role === 'shovel' ? '测试叉面线段' : '测试齿形'} · DXF「${d.name}」已保留`;
    const source = d.sourceKind === 'builtin' ? '内置' : '已导入';
    const extra = d.unsupported?.length ? `；略过 ${d.unsupported.slice(0, 2).join(', ')}` : '';
    return `${source} · ${name} · ${d.entityCount} 实体 · ${format(b.width * 1000, 0)}×${format(b.height * 1000, 0)} mm${extra}`;
  }
  function renderDrawingStatus() {
    ['shovel', 'weapon'].forEach((role) => {
      const card = $(`#${role}DrawingCard`); card.classList.toggle('loaded', true); card.classList.remove('error');
      $(`#${role}DrawingStatus`).textContent = drawingStatus(role);
    });
    const weapon = state.params.paramWeaponEnabled ? '测试齿形' : state.drawings.weapon.name;
    const fork = state.params.paramForkEnabled ? '测试叉面' : state.drawings.shovel.name;
    $('#fileSummary').textContent = `当前求解：${weapon} · ${fork}`;
  }
  async function importDrawing(file, role) { if (!file.name.toLowerCase().endsWith('.dxf')) { showToast('此版本请先将图纸导出为 ASCII DXF。', 'warning'); return; } try { const raw = await file.text(); state.drawings[role] = createImportedDrawing(role, file.name, raw); if (role === 'weapon') state.params.tipRadius = Math.max(1, Math.round(getMaxWeaponRadius() * 1000)); syncInputs(); renderDrawingStatus(); clearEvents(); resetSimulation(); addEvent(`已导入 ${role === 'shovel' ? '叉子' : '武器'} DXF：${file.name}`); showToast('DXF 已导入，保留 CAD (0,0) mm 作为局部原点。'); } catch (error) { const card = $(`#${role}DrawingCard`); card.classList.add('error'); $(`#${role}DrawingStatus`).textContent = `解析失败：${error.message}`; showToast(`DXF 解析失败：${error.message}`, 'error'); } }

  function resizeCanvas() { if (!state.canvas) return; const rect = state.canvas.getBoundingClientRect(); const dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1)); const width = Math.round(rect.width * dpr); const height = Math.round(rect.height * dpr); if (state.canvas.width !== width || state.canvas.height !== height) { state.canvas.width = width; state.canvas.height = height; } state.ctx.setTransform(dpr, 0, 0, dpr, 0, 0); if (!state.camera.center || !state.camera.scale) fitCamera(rect.width, rect.height); drawScene(); }
  function collectScenePoints() {
    const centre = weaponSceneOrigin();
    const shovel = state.params.paramForkEnabled
      ? parameterForkGeometry().points.map(forkWorldFromLocal)
      : state.drawings.shovel.paths.flatMap(samplePath).map(shovelWorld);
    const weapon = state.params.paramWeaponEnabled
      ? parameterWeaponGeometry().segments.flatMap((segment) => segment.loop.map((candidate) => add(centre, rotate(candidate, state.sim.angle))))
      : state.drawings.weapon.paths.flatMap(samplePath).map(weaponWorld);
    const target = targetCorners(); const trail = state.sim.trail.map((p) => point(p.x, p.y)); const r = state.metrics.radius;
    return [...shovel, ...weapon, ...target, ...trail, add(centre, point(-r, -r)), add(centre, point(r, r)), centre, point(0, 0)];
  }
  function fitCamera(width, height) {
    const rect = state.canvas?.getBoundingClientRect(); const w = width || rect?.width || 1; const h = height || rect?.height || 1; const points = collectScenePoints(); let minX = Infinity; let maxX = -Infinity; let minY = Infinity; let maxY = -Infinity;
    points.forEach((p) => { minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x); minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y); });
    minY = Math.min(minY, groundY() - .025); const padX = Math.max(.06, (maxX - minX) * .15); const padY = Math.max(.05, (maxY - minY) * .22);
    minX -= padX; maxX += padX; minY -= padY; maxY += padY;
    const scale = clamp(Math.min((w - 44) / Math.max(maxX - minX, .1), (h - 54) / Math.max(maxY - minY, .1)), 50, 3200);
    state.camera.center = point((minX + maxX) / 2, (minY + maxY) / 2); state.camera.scale = scale; state.camera.fitScale = scale;
  }
  function buildView(width, height) { if (!state.camera.center || !state.camera.scale) fitCamera(width, height); return { width, height, scale: state.camera.scale, cx: state.camera.center.x, cy: state.camera.center.y }; }
  function screenPoint(world) { const v = state.view; return point((world.x - v.cx) * v.scale + v.width / 2, v.height / 2 - (world.y - v.cy) * v.scale); }
  function worldPointFromCanvas(x, y) { const view = state.view; return point((x - view.width / 2) / view.scale + view.cx, (view.height / 2 - y) / view.scale + view.cy); }
  function worldPointFromEvent(event) { const rect = state.canvas.getBoundingClientRect(); return worldPointFromCanvas(event.clientX - rect.left, event.clientY - rect.top); }
  function setupContext() { const ctx = state.ctx; const rect = state.canvas.getBoundingClientRect(); const dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1)); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, rect.width, rect.height); return rect; }
  function drawGrid(ctx, view) { if (!state.showGrid) return; const step = view.scale > 1500 ? .01 : view.scale > 750 ? .02 : view.scale > 300 ? .05 : .1; const minX = view.cx - view.width / 2 / view.scale; const maxX = view.cx + view.width / 2 / view.scale; const minY = view.cy - view.height / 2 / view.scale; const maxY = view.cy + view.height / 2 / view.scale; ctx.save(); const floor = screenPoint(point(0, groundY())); ctx.fillStyle = 'rgba(78, 94, 78, .23)'; ctx.fillRect(0, Math.max(0, floor.y), view.width, Math.max(0, view.height - floor.y)); ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(122,142,171,.11)'; for (let x = Math.ceil(minX / step) * step; x <= maxX; x += step) { const s = screenPoint(point(x, 0)); ctx.beginPath(); ctx.moveTo(s.x, 0); ctx.lineTo(s.x, view.height); ctx.stroke(); } for (let y = Math.ceil(minY / step) * step; y <= maxY; y += step) { const s = screenPoint(point(0, y)); ctx.beginPath(); ctx.moveTo(0, s.y); ctx.lineTo(view.width, s.y); ctx.stroke(); } ctx.strokeStyle = 'rgba(38,215,195,.48)'; ctx.beginPath(); const xAxis = screenPoint(point(0, 0)); ctx.moveTo(0, xAxis.y); ctx.lineTo(view.width, xAxis.y); const yAxis = screenPoint(point(0, 0)); ctx.moveTo(yAxis.x, 0); ctx.lineTo(yAxis.x, view.height); ctx.stroke(); ctx.setLineDash([5, 5]); ctx.strokeStyle = 'rgba(135,180,139,.72)'; ctx.lineWidth = 1.4; ctx.beginPath(); ctx.moveTo(0, floor.y); ctx.lineTo(view.width, floor.y); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = 'rgba(157, 179, 205, .78)'; ctx.font = '10px system-ui'; ctx.fillText('X + 前方', view.width - 48, xAxis.y - 8); ctx.fillText('Y +', yAxis.x + 8, 14); ctx.fillText('场景 (0,0) mm', xAxis.x + 7, xAxis.y - 8); ctx.fillText(`${getFloorMaterial().label} 地面`, 10, floor.y - 7); ctx.restore(); }
  function drawPath(ctx, path, transform, options = {}) { const points = samplePath(path); if (!points.length) return; ctx.beginPath(); points.forEach((source, index) => { const s = screenPoint(transform(source)); if (index === 0) ctx.moveTo(s.x, s.y); else ctx.lineTo(s.x, s.y); }); if (path.closed || path.type === 'circle') ctx.closePath(); if (options.fill && (path.closed || path.type === 'circle')) ctx.fill(); if (options.stroke !== false) ctx.stroke(); }
  function drawShovel(ctx) { const d = state.drawings.shovel; ctx.save(); ctx.lineWidth = 1.55; ctx.strokeStyle = '#b4e9e2'; ctx.fillStyle = 'rgba(38,215,195,.07)'; d.paths.forEach((path) => drawPath(ctx, path, shovelWorld, { fill: path.closed })); const o = screenPoint(forkWorldOrigin()); ctx.strokeStyle = 'rgba(38,215,195,.72)'; ctx.beginPath(); ctx.arc(o.x, o.y, 4, 0, Math.PI * 2); ctx.stroke(); ctx.fillStyle = '#9cfff0'; ctx.font = '600 10px system-ui'; ctx.fillText('叉子铰点 / 局部原点', o.x + 7, o.y - 7); ctx.restore(); }
  function drawWeapon(ctx) { const origin = weaponSceneOrigin(); const centre = screenPoint(origin); const r = state.metrics.radius * state.view.scale; const d = state.drawings.weapon; ctx.save(); if (state.showEnvelope) { ctx.setLineDash([5, 5]); ctx.strokeStyle = 'rgba(255,157,66,.6)'; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(centre.x, centre.y, r, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]); } ctx.strokeStyle = '#ffc083'; ctx.fillStyle = 'rgba(255,124,49,.08)'; ctx.lineWidth = 1.55; d.paths.forEach((path) => drawPath(ctx, path, weaponWorld, { fill: path.closed })); allToothTips().forEach((tip) => { const end = screenPoint(tip.point); const start = screenPoint(add(origin, scalePoint(tip.relative, .76))); ctx.strokeStyle = 'rgba(255,191,124,.4)'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(start.x, start.y); ctx.lineTo(end.x, end.y); ctx.stroke(); ctx.fillStyle = '#ffd7ab'; ctx.beginPath(); ctx.arc(end.x, end.y, 2.8, 0, Math.PI * 2); ctx.fill(); }); ctx.fillStyle = '#38262d'; ctx.strokeStyle = '#ffe3c2'; ctx.lineWidth = 1.3; ctx.beginPath(); ctx.arc(centre.x, centre.y, Math.max(5, r * .12), 0, Math.PI * 2); ctx.fill(); ctx.stroke(); ctx.beginPath(); ctx.moveTo(centre.x - 9, centre.y); ctx.lineTo(centre.x + 9, centre.y); ctx.moveTo(centre.x, centre.y - 9); ctx.lineTo(centre.x, centre.y + 9); ctx.stroke(); ctx.fillStyle = '#ffe2c0'; ctx.font = '700 10px system-ui'; ctx.fillText('武器局部原点', centre.x + 11, centre.y - 11); ctx.restore(); }
  function drawTarget(ctx) { const corners = targetCorners().map(screenPoint); const material = getMaterial(); ctx.save(); if (state.sim.trail.length > 1) { ctx.strokeStyle = `${material.color}75`; ctx.lineWidth = 1.2; ctx.setLineDash([3, 4]); ctx.beginPath(); state.sim.trail.forEach((p, i) => { const s = screenPoint(p); if (!i) ctx.moveTo(s.x, s.y); else ctx.lineTo(s.x, s.y); }); ctx.stroke(); ctx.setLineDash([]); } ctx.fillStyle = `${material.color}45`; ctx.strokeStyle = `${material.color}df`; ctx.lineWidth = 1.5; ctx.beginPath(); corners.forEach((p, i) => { if (!i) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y); }); ctx.closePath(); ctx.fill(); ctx.stroke(); const centre = screenPoint(state.sim.target.pos); ctx.fillStyle = material.color; ctx.beginPath(); ctx.arc(centre.x, centre.y, 2.8, 0, Math.PI * 2); ctx.fill(); ctx.font = '600 10px system-ui'; ctx.fillText(`${material.short} · X ${format(targetLength() * 1000, 0)} mm · Y ${format(targetThickness() * 1000, 0)} mm`, centre.x + 8, centre.y - 10); ctx.restore(); }
  function clipSolidToFloor(ctx) {
    const floor = screenPoint(point(0, groundY())).y;
    ctx.beginPath(); ctx.rect(0, 0, state.view.width, Math.max(0, floor)); ctx.clip();
  }
  function drawForegroundFloor(ctx) {
    const floor = screenPoint(point(0, groundY())).y;
    ctx.save(); ctx.setLineDash([5, 5]); ctx.strokeStyle = 'rgba(135,180,139,.88)'; ctx.lineWidth = 1.35;
    ctx.beginPath(); ctx.moveTo(0, floor); ctx.lineTo(state.view.width, floor); ctx.stroke(); ctx.restore();
  }
  function drawShovelSolid(ctx) {
    const drawing = state.drawings.shovel;
    ctx.save();
    clipSolidToFloor(ctx);
    if (state.params.paramForkEnabled) {
      const geometry = parameterForkGeometry(); const root = screenPoint(forkWorldFromLocal(geometry.root)); const tip = screenPoint(forkWorldFromLocal(geometry.tip));
      // The thin stroke is only a screen-space aid. Rapier solves the exact
      // zero-thickness segment and no hidden side-profile width is introduced.
      ctx.strokeStyle = '#b9fff4'; ctx.lineWidth = 1.4;
      ctx.beginPath(); ctx.moveTo(root.x, root.y); ctx.lineTo(tip.x, tip.y); ctx.stroke();
      ctx.fillStyle = '#d5fff8'; [root, tip].forEach((candidate) => { ctx.beginPath(); ctx.arc(candidate.x, candidate.y, 2.5, 0, Math.PI * 2); ctx.fill(); });
      ctx.fillStyle = '#9cfff0'; ctx.font = '700 10px system-ui'; ctx.fillText('理想无厚度线叉', (root.x + tip.x) / 2 + 6, (root.y + tip.y) / 2 - 7);
    } else {
      drawSolidLoops(ctx, drawing, shovelWorld, 'rgba(31, 203, 188, .96)');
      ctx.strokeStyle = '#b9fff4'; ctx.lineWidth = 1.35;
      drawing.paths.forEach((path) => drawPath(ctx, path, shovelWorld, { fill: false }));
    }
    const hinge = screenPoint(forkWorldOrigin());
    ctx.fillStyle = '#0a393a'; ctx.strokeStyle = '#b9fff4'; ctx.lineWidth = 1.1;
    ctx.beginPath(); ctx.arc(hinge.x, hinge.y, 3.6, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.restore();
  }
  function drawWeaponSolid(ctx) {
    const origin = weaponSceneOrigin(); const centre = screenPoint(origin); const radius = state.metrics.radius * state.view.scale; const drawing = state.drawings.weapon;
    ctx.save();
    clipSolidToFloor(ctx);
    if (state.showEnvelope) { ctx.setLineDash([5, 5]); ctx.strokeStyle = 'rgba(255,157,66,.62)'; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(centre.x, centre.y, radius, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]); }
    if (state.params.paramWeaponEnabled) {
      parameterWeaponGeometry().segments.forEach((segment) => {
        const loop = segment.loop.map((candidate) => screenPoint(add(origin, rotate(candidate, state.sim.angle))));
        ctx.fillStyle = 'rgba(225, 97, 29, .92)'; ctx.strokeStyle = 'rgba(255,232,202,.72)'; ctx.lineWidth = .8;
        ctx.beginPath(); loop.forEach((candidate, index) => { if (!index) ctx.moveTo(candidate.x, candidate.y); else ctx.lineTo(candidate.x, candidate.y); });
        ctx.closePath(); ctx.fill(); ctx.stroke();
        // Highlight the literal root-to-tip working face so the entered rake
        // angle can be inspected against the tip radius on the canvas.
        const root = screenPoint(add(origin, rotate(segment.root, state.sim.angle)));
        const tip = screenPoint(add(origin, rotate(segment.tip, state.sim.angle)));
        ctx.strokeStyle = 'rgba(255,247,227,.95)'; ctx.lineWidth = 1.25;
        ctx.beginPath(); ctx.moveTo(root.x, root.y); ctx.lineTo(tip.x, tip.y); ctx.stroke();
      });
    } else {
      drawSolidLoops(ctx, drawing, weaponWorld, 'rgba(225, 97, 29, .88)');
      ctx.strokeStyle = '#ffe0bd'; ctx.lineWidth = 1.35;
      drawing.paths.forEach((path) => drawPath(ctx, path, weaponWorld, { fill: false }));
    }
    allToothTips().forEach((tip) => {
      const end = screenPoint(tip.point); const start = screenPoint(add(origin, scalePoint(tip.relative, .76)));
      ctx.strokeStyle = 'rgba(255,228,192,.50)'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(start.x, start.y); ctx.lineTo(end.x, end.y); ctx.stroke();
      ctx.fillStyle = '#fff0db'; ctx.beginPath(); ctx.arc(end.x, end.y, 2.9, 0, Math.PI * 2); ctx.fill();
    });
    ctx.fillStyle = '#251d1c'; ctx.strokeStyle = '#ffe3c2'; ctx.lineWidth = 1.25;
    if (!state.params.paramWeaponEnabled) { ctx.beginPath(); ctx.arc(centre.x, centre.y, Math.max(5, radius * .12), 0, Math.PI * 2); ctx.fill(); ctx.stroke(); }
    else { ctx.save(); ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.arc(centre.x, centre.y, 4.5, 0, Math.PI * 2); ctx.stroke(); ctx.restore(); }
    ctx.beginPath(); ctx.moveTo(centre.x - 8, centre.y); ctx.lineTo(centre.x + 8, centre.y); ctx.moveTo(centre.x, centre.y - 8); ctx.lineTo(centre.x, centre.y + 8); ctx.stroke();
    ctx.restore();
  }
  function drawTargetSolid(ctx) {
    const geometry = targetRemainingWorldGeometry().map((polygon) => polygon.map((ring) => ring.map(screenPoint)));
    const corners = geometry.flat(2); const material = getMaterial();
    if (!corners.length) return;
    ctx.save();
    clipSolidToFloor(ctx);
    if (state.sim.trail.length > 1) { ctx.strokeStyle = `${material.color}88`; ctx.lineWidth = 1.25; ctx.setLineDash([3, 4]); ctx.beginPath(); state.sim.trail.forEach((p, index) => { const screen = screenPoint(p); if (!index) ctx.moveTo(screen.x, screen.y); else ctx.lineTo(screen.x, screen.y); }); ctx.stroke(); ctx.setLineDash([]); }
    if (targetHasUncutZBacking()) {
      const backing = targetCorners().map(screenPoint);
      ctx.fillStyle = `${material.color}38`; ctx.strokeStyle = `${material.color}7a`; ctx.lineWidth = .8;
      ctx.beginPath(); backing.forEach((p, index) => { if (!index) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y); }); ctx.closePath(); ctx.fill(); ctx.stroke();
    }
    const minX = Math.min(...corners.map((p) => p.x)); const maxX = Math.max(...corners.map((p) => p.x));
    const shade = ctx.createLinearGradient(minX, 0, maxX, 0); shade.addColorStop(0, `${material.color}d8`); shade.addColorStop(.55, `${material.color}ff`); shade.addColorStop(1, `${material.color}b8`);
    ctx.fillStyle = shade; ctx.beginPath();
    geometry.forEach((polygon) => polygon.forEach((ring) => {
      ring.forEach((p, index) => { if (!index) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y); });
      ctx.closePath();
    }));
    ctx.fill('evenodd');
    // The visible solid and the triangulated Rapier body share this exact
    // polygonal difference; there is no hidden row-grid silhouette.
    ctx.strokeStyle = `${material.color}42`; ctx.lineWidth = .45; ctx.stroke();
    const centre = screenPoint(state.sim.target.pos); ctx.fillStyle = '#ffffff'; ctx.beginPath(); ctx.arc(centre.x, centre.y, 2.35, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#ddecff'; ctx.font = '600 10px system-ui'; ctx.fillText(`${material.short} · X ${format(targetLength() * 1000, 0)} · Y ${format(targetThickness() * 1000, 0)} · Z ${format(targetWidthZ() * 1000, 0)} mm`, centre.x + 8, centre.y - 10);
    ctx.restore();
  }
  function drawArrow(ctx, start, end, color) { const a = screenPoint(start); const b = screenPoint(end); const angle = Math.atan2(b.y - a.y, b.x - a.x); ctx.save(); ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke(); ctx.beginPath(); ctx.moveTo(b.x, b.y); ctx.lineTo(b.x - 7 * Math.cos(angle - .4), b.y - 7 * Math.sin(angle - .4)); ctx.lineTo(b.x - 7 * Math.cos(angle + .4), b.y - 7 * Math.sin(angle + .4)); ctx.closePath(); ctx.fill(); ctx.restore(); }
  function drawAnnotations(ctx) {
    const sim = state.sim;
    ctx.save();
    if (sim.forkEngaged && Number.isFinite(state.metrics.biteInstant)) {
      const bite = state.metrics.biteInstant / 1000;
      const targetLeft = Math.min(...targetCorners().map((p) => p.x));
      const y = sim.target.pos.y + targetThickness() / 2 + .03;
      const a = point(targetLeft - bite, y); const b = point(targetLeft, y);
      const sa = screenPoint(a); const sb = screenPoint(b);
      ctx.strokeStyle = '#71f0e0'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(sa.x, sa.y); ctx.lineTo(sb.x, sb.y); ctx.stroke();
      [sa, sb].forEach((p) => { ctx.beginPath(); ctx.moveTo(p.x, p.y - 4); ctx.lineTo(p.x, p.y + 4); ctx.stroke(); });
      ctx.fillStyle = '#a6fff3'; ctx.font = '600 10px system-ui';
      ctx.fillText(`当前相对进给 ${format(state.metrics.biteInstant)} mm/齿`, (sa.x + sb.x) / 2 - 46, sa.y - 7);
    }
    const contact = sim.currentContact; if (contact) { const s = screenPoint(contact.point); const pulse = 10 + Math.sin(sim.time * 34) * 2; ctx.fillStyle = 'rgba(251,113,133,.2)'; ctx.beginPath(); ctx.arc(s.x, s.y, pulse, 0, Math.PI * 2); ctx.fill(); ctx.fillStyle = '#ff7188'; ctx.beginPath(); ctx.arc(s.x, s.y, 4.5, 0, Math.PI * 2); ctx.fill(); const normalEnd = add(contact.point, scalePoint(contact.normal, .04)); drawArrow(ctx, contact.point, normalEnd, '#ff8497'); ctx.fillStyle = '#ffc8d0'; ctx.font = '700 10px system-ui'; ctx.fillText(`接触 vₙ ${format(contact.impact?.relativeSpeed ?? contact.relativeSpeed)} m/s`, s.x + 9, s.y - 12); }
    if (sim.targetLaunched && length(sim.target.vel) > .02) drawArrow(ctx, sim.target.pos, add(sim.target.pos, scalePoint(normalise(sim.target.vel), Math.min(.08, length(sim.target.vel) * .025))), '#72dff2');
    ctx.restore(); }
  function drawPhase() {
    const badge = $('#phaseBadge'); const sim = state.sim; const failureDomain = resolvedFailureDomain(sim);
    if (failureDomain === 'internal') { badge.textContent = '内部异常安全停止'; badge.className = 'phase-badge warning'; }
    else if (failureDomain === 'material-model') { badge.textContent = '材料模型安全停止'; badge.className = 'phase-badge warning'; }
    else if (failureDomain === 'rigid-solver') { badge.textContent = '刚体求解安全停止'; badge.className = 'phase-badge warning'; }
    else if (sim.completed) { badge.textContent = '设定时长完成'; badge.className = 'phase-badge done'; }
    else if (sim.currentContact?.role === 'weapon') {
      const bodyContact = sim.currentContact.impact?.bodyContact ?? !sim.currentContact.isTooth;
      badge.textContent = bodyContact
        ? '刀体 / 背板接触'
        : (sim.currentContact.impact ? '武器牙形接触 / 命中' : '武器牙形接触');
      badge.className = 'phase-badge weapon';
    } else if (sim.currentContact?.role === 'fork' || sim.forkEngaged) { badge.textContent = '叉子刚体接触'; badge.className = 'phase-badge shovel'; }
    else if (sim.targetLaunched) { badge.textContent = '靶子离开支撑'; badge.className = 'phase-badge weapon'; }
    else { badge.textContent = '接近中'; badge.className = 'phase-badge approach'; }
  }
  function drawScene() { if (!state.ctx || !state.sim || !state.metrics) return; const rect = setupContext(); if (!rect.width || !rect.height) return; state.view = buildView(rect.width, rect.height); const ctx = state.ctx; const gradient = ctx.createLinearGradient(0, 0, 0, rect.height); gradient.addColorStop(0, '#0a1320'); gradient.addColorStop(.72, '#0c1725'); gradient.addColorStop(1, '#0e1b2b'); ctx.fillStyle = gradient; ctx.fillRect(0, 0, rect.width, rect.height); drawGrid(ctx, state.view); drawTargetSolid(ctx); drawShovelSolid(ctx); drawWeaponSolid(ctx); drawForegroundFloor(ctx); drawAnnotations(ctx); drawPhase(); setText('zoomReadout', `缩放 ${(state.view.scale / Math.max(state.camera.fitScale || state.view.scale, 1)).toFixed(2)}×`); }
  function updateTouchCamera(event) {
    const pointers = state.camera.touchPointers;
    if (!pointers?.has(event.pointerId) || !state.view) return false;
    event.preventDefault();
    pointers.set(event.pointerId, { x: event.clientX, y: event.clientY });
    const active = [...pointers.entries()];
    if (active.length >= 2) {
      const first = active[0][1]; const second = active[1][1];
      const distance = Math.max(1, Math.hypot(second.x - first.x, second.y - first.y));
      const midpoint = point((first.x + second.x) / 2, (first.y + second.y) / 2);
      if (!state.camera.pinch) {
        const rect = state.canvas.getBoundingClientRect();
        state.camera.pinch = {
          startDistance: distance,
          startScale: state.view.scale,
          anchor: worldPointFromCanvas(midpoint.x - rect.left, midpoint.y - rect.top),
        };
      }
      const pinch = state.camera.pinch; const rect = state.canvas.getBoundingClientRect();
      const nextScale = clamp(pinch.startScale * distance / pinch.startDistance, 50, 3200);
      const x = midpoint.x - rect.left; const y = midpoint.y - rect.top;
      state.camera.center = point(
        pinch.anchor.x - (x - rect.width / 2) / nextScale,
        pinch.anchor.y + (y - rect.height / 2) / nextScale,
      );
      state.camera.scale = nextScale; state.camera.dragging = null;
      $('#canvasWrap').classList.add('panning'); drawScene();
      return true;
    }
    const drag = state.camera.dragging;
    if (drag?.pointerId === event.pointerId) {
      const dx = event.clientX - drag.lastX; const dy = event.clientY - drag.lastY;
      if (dx || dy) {
        state.camera.center.x -= dx / state.view.scale;
        state.camera.center.y += dy / state.view.scale;
        drag.lastX = event.clientX; drag.lastY = event.clientY; drag.moved = true;
        $('#canvasWrap').classList.add('panning'); drawScene();
      }
    }
    return true;
  }
  function onCanvasMove(event) {
    if (!state.view) return;
    if (event.pointerType === 'touch' && updateTouchCamera(event)) return;
    if (state.camera.dragging) {
      const drag = state.camera.dragging; const dx = event.clientX - drag.lastX; const dy = event.clientY - drag.lastY;
      if (dx || dy) {
        state.camera.center.x -= dx / state.view.scale; state.camera.center.y += dy / state.view.scale;
        drag.lastX = event.clientX; drag.lastY = event.clientY; drag.moved = true;
        $('#canvasWrap').classList.add('panning'); drawScene();
      }
      return;
    }
    const p = worldPointFromEvent(event); const relative = subtract(p, weaponSceneOrigin());
    setText('cursorReadout', `场景 X ${format(p.x * 1000, 1)} / Y ${format(p.y * 1000, 1)} mm · 相对武器 X ${format(relative.x * 1000, 1)} / Y ${format(relative.y * 1000, 1)} mm`);
  }
  function onCanvasClick(event) { if (!state.axisPicking) return; const clicked = worldPointFromEvent(event); const localMirrored = rotate(subtract(clicked, weaponSceneOrigin()), -state.sim.angle); const sourceDelta = point(localMirrored.x * (state.params.weaponMirrorX ? -1 : 1), localMirrored.y * (state.params.weaponMirrorY ? -1 : 1)); state.drawings.weapon.pivot = add(state.drawings.weapon.pivot, sourceDelta); state.params.tipRadius = Math.max(1, Math.round(getMaxWeaponRadius() * 1000)); syncInputs(); setAxisPicking(false); resetAfterChange('画布点选武器局部原点'); }
  function adjustZoom(factor, event = null) { const rect = state.canvas.getBoundingClientRect(); const x = event ? event.clientX - rect.left : rect.width / 2; const y = event ? event.clientY - rect.top : rect.height / 2; const anchor = worldPointFromCanvas(x, y); const nextScale = clamp(state.view.scale * factor, 50, 3200); state.camera.center = point(anchor.x - (x - rect.width / 2) / nextScale, anchor.y + (y - rect.height / 2) / nextScale); state.camera.scale = nextScale; drawScene(); }
  function fitScene() { const rect = state.canvas.getBoundingClientRect(); fitCamera(rect.width, rect.height); drawScene(); showToast('已适配场景；滚轮可围绕鼠标缩放，中键或 Shift 拖动可平移。'); }
  function onCanvasPointerDown(event) {
    if (state.axisPicking) return;
    if (event.pointerType === 'touch') {
      event.preventDefault();
      state.camera.touchPointers.set(event.pointerId, { x: event.clientX, y: event.clientY });
      state.canvas.setPointerCapture?.(event.pointerId);
      if (state.camera.touchPointers.size === 1) {
        state.camera.dragging = { pointerId: event.pointerId, lastX: event.clientX, lastY: event.clientY, moved: false };
        state.camera.pinch = null;
      } else {
        state.camera.dragging = null; state.camera.pinch = null;
      }
      $('#canvasWrap').classList.add('pan-ready');
      return;
    }
    if (event.button === 1 || (event.button === 0 && event.shiftKey)) {
      event.preventDefault();
      state.camera.dragging = { pointerId: event.pointerId, lastX: event.clientX, lastY: event.clientY, moved: false };
      state.canvas.setPointerCapture?.(event.pointerId); $('#canvasWrap').classList.add('pan-ready');
    }
  }
  function onCanvasAxisPick(event) {
    if (!state.axisPicking) return;
    const oldPivot = point(state.drawings.weapon.pivot.x, state.drawings.weapon.pivot.y);
    const oldRadius = state.params.tipRadius;
    const clicked = worldPointFromEvent(event);
    const localMirrored = rotate(subtract(clicked, weaponSceneOrigin()), -state.sim.angle);
    const sourceDelta = point(localMirrored.x * (state.params.weaponMirrorX ? -1 : 1), localMirrored.y * (state.params.weaponMirrorY ? -1 : 1));
    state.drawings.weapon.pivot = add(state.drawings.weapon.pivot, sourceDelta);
    state.params.tipRadius = Math.max(1, Math.round(getMaxWeaponRadius() * 1000));
    if (!validateGroundClearance($('[data-param="tipRadius"]'))) {
      state.drawings.weapon.pivot = oldPivot;
      state.params.tipRadius = oldRadius;
      syncInputs();
      return;
    }
    syncInputs(); setAxisPicking(false); resetAfterChange('画布点选武器局部原点');
  }
  function onCanvasPointerUp(event) {
    if (state.axisPicking && event.pointerType === 'touch') { onCanvasAxisPick(event); return; }
    if (event.pointerType === 'touch') {
      state.camera.touchPointers.delete(event.pointerId); state.camera.pinch = null;
      const remaining = [...state.camera.touchPointers.entries()];
      if (remaining.length === 1) {
        const [pointerId, position] = remaining[0];
        state.camera.dragging = { pointerId, lastX: position.x, lastY: position.y, moved: false };
      } else if (!remaining.length) {
        state.camera.dragging = null; $('#canvasWrap').classList.remove('panning', 'pan-ready');
      }
      return;
    }
    const drag = state.camera.dragging;
    if (drag && drag.pointerId === event.pointerId) {
      state.camera.dragging = null; $('#canvasWrap').classList.remove('panning', 'pan-ready'); return;
    }
    if (state.axisPicking && event.button === 0) onCanvasAxisPick(event);
  }

  function clonePlaybackPlain(value, seen = new WeakMap()) {
    if (value === null || typeof value !== 'object') return value;
    if (seen.has(value)) return seen.get(value);
    if (Array.isArray(value)) {
      const output = []; seen.set(value, output);
      value.forEach((entry) => output.push(clonePlaybackPlain(entry, seen)));
      return output;
    }
    const prototype = Object.getPrototypeOf(value);
    // Rapier bodies/colliders/worlds are WASM wrappers and must never enter a
    // playback frame. Contacts retain only their plain engineering telemetry.
    if (prototype !== Object.prototype && prototype !== null) return undefined;
    const output = {}; seen.set(value, output);
    Object.entries(value).forEach(([key, entry]) => {
      if (key === 'other' || key === 'physics') return;
      const cloned = clonePlaybackPlain(entry, seen);
      if (cloned !== undefined) output[key] = cloned;
    });
    return output;
  }

  const PLAYBACK_SCALARS = Object.freeze([
    'time', 'angle', 'forkAngle', 'forkOmega', 'forkGrounded', 'forkContact', 'forkEngaged', 'forkSupportLift',
    'targetPushedByFork', 'targetLaunched', 'robotTravel', 'driveSpeed',
    'driveSlip', 'weaponOmega', 'lastWeaponAngleStep', 'lastImpactTime', 'lastBodyImpactTime',
    'lastBlockingTime', 'weaponBlocked', 'modelDomainStopped', 'failureDomain', 'hitCount', 'bodyImpactCount',
    'lastEventHit', 'lastEventBodyImpact', 'blockingAnnounced', 'rigMinFloorClearance', 'rigForkMinFloorClearance', 'rigJointError', 'rigForkJointError', 'rigMaxJointError', 'rigMaxForkJointError',
    'rigSubstepsLastTick', 'materialMaxIntrusion', 'completed', 'solverDomainStopped', 'creationError',
  ]);

  function capturePlaybackFrame(trajectory) {
    const sim = state.sim; const previousFrame = trajectory.frames[trajectory.frames.length - 1];
    const frame = {};
    PLAYBACK_SCALARS.forEach((key) => { frame[key] = sim[key]; });
    frame.forkOrigin = clonePlaybackPlain(sim.forkOrigin);
    frame.forkVelocity = clonePlaybackPlain(sim.forkVelocity);
    frame.target = clonePlaybackPlain(sim.target);
    frame.currentContact = clonePlaybackPlain(sim.currentContact);
    frame.lastRigFailure = clonePlaybackPlain(sim.lastRigFailure);
    if (sim.lastImpact === trajectory.lastImpactSource) frame.lastImpact = trajectory.lastImpactClone;
    else {
      trajectory.lastImpactSource = sim.lastImpact;
      trajectory.lastImpactClone = clonePlaybackPlain(sim.lastImpact);
      frame.lastImpact = trajectory.lastImpactClone;
    }
    frame.rigMaxPenetration = { ...sim.rigMaxPenetration };
    frame.rigRejectedPenetration = { ...sim.rigRejectedPenetration };
    const previousDamage = previousFrame?.materialDamage;
    if (previousDamage && previousDamage.version === sim.materialDamage?.version) {
      // Geometry is immutable between damage versions, but contact-pass state
      // (active tooth/time/angular travel) and plastic-ploughing work can change
      // without deleting a chip.  Share only the heavy versioned arrays and
      // capture a fresh scalar shell and energy ledger for every frame.
      frame.materialDamage = {
        ...sim.materialDamage,
        depths: previousDamage.depths,
        rightDepths: previousDamage.rightDepths,
        segments: previousDamage.segments,
        geometry: previousDamage.geometry,
        pending: previousDamage.pending,
        rightPending: previousDamage.rightPending,
        cuts: previousDamage.cuts,
      };
      frame.materialStats = { ...sim.materialStats };
    } else {
      frame.materialDamage = cloneMaterialDamage(sim.materialDamage);
      frame.materialStats = { ...sim.materialStats };
    }
    const lastTrail = sim.trail[sim.trail.length - 1]; const previousTrail = previousFrame?.trail;
    const previousLast = previousTrail?.[previousTrail.length - 1];
    const unchangedTrail = previousTrail && previousTrail.length === sim.trail.length
      && ((!lastTrail && !previousLast) || (lastTrail?.time === previousLast?.time && lastTrail?.x === previousLast?.x && lastTrail?.y === previousLast?.y));
    frame.trail = unchangedTrail ? previousTrail : sim.trail.map((entry) => ({ ...entry }));
    frame.eventCursor = trajectory.events.length;
    return frame;
  }

  function applyPlaybackFrame(index, { render = true, preserveRunning = false } = {}) {
    const trajectory = state.trajectory;
    if (!trajectory || (!trajectory.ready && !trajectory.building) || !trajectory.frames.length) return false;
    const bounded = clamp(Math.round(index), 0, trajectory.frames.length - 1); const frame = trajectory.frames[bounded];
    const running = preserveRunning && Boolean(state.sim.running);
    PLAYBACK_SCALARS.forEach((key) => { state.sim[key] = frame[key]; });
    state.sim.forkOrigin = clonePlaybackPlain(frame.forkOrigin);
    state.sim.forkVelocity = clonePlaybackPlain(frame.forkVelocity);
    state.sim.target = clonePlaybackPlain(frame.target);
    state.sim.currentContact = clonePlaybackPlain(frame.currentContact);
    state.sim.lastRigFailure = clonePlaybackPlain(frame.lastRigFailure);
    state.sim.lastImpact = clonePlaybackPlain(frame.lastImpact);
    state.sim.rigMaxPenetration = { ...frame.rigMaxPenetration };
    state.sim.rigRejectedPenetration = { ...frame.rigRejectedPenetration };
    // Playback frames are immutable; sharing the versioned snapshot avoids a
    // deep copy on every animation frame. Direct physics mode always rebuilds
    // an independent world/state before any mutation.
    state.sim.materialDamage = frame.materialDamage;
    state.sim.materialStats = frame.materialStats;
    state.sim.trail = frame.trail;
    state.sim.running = running;
    state.playheadTick = bounded;
    renderEventEntries(trajectory.events, frame.eventCursor, trajectory.eventFloor);
    if (render) { updateReadouts(); drawScene(); }
    return true;
  }

  function detachVisibleTrajectoryEvents(trajectory) {
    if (!trajectory?.ready) return;
    const cursor = trajectory.frames[state.playheadTick]?.eventCursor ?? 0;
    state.eventEntries = trajectory.events.slice(trajectory.eventFloor, cursor).map((entry) => ({ ...entry }));
  }

  function invalidateTrajectory() {
    const trajectory = state.trajectory;
    if (trajectory?.ready) detachVisibleTrajectoryEvents(trajectory);
    if (trajectory?.pendingRigTick) cancelRigTickTask(trajectory.pendingRigTick);
    if (trajectory?.workSim?.physics) {
      disposeTargetPhysics(trajectory.workSim.physics);
      trajectory.workSim.physics = null;
    }
    state.trajectoryGeneration += 1;
    if (state.trajectoryTimer) window.clearTimeout(state.trajectoryTimer);
    state.trajectoryTimer = 0; state.trajectory = null; state.playheadTick = 0; state.accumulator = 0;
    const seek = $('#timelineSeek'); if (seek) seek.disabled = false;
    const clear = $('#clearEvents'); if (clear) clear.disabled = false;
  }

  function makePlaybackPhysicsDescriptor(physics) {
    if (!physics) return null;
    return {
      playbackDescriptor: true,
      rigModel: Boolean(physics.rigModel),
      world: { lengthUnit: physics.world?.lengthUnit },
      weaponLoop: clonePlaybackPlain(physics.weaponLoop),
      weaponColliders: new Array(physics.weaponColliders?.length || (physics.weaponCollider ? 1 : 0)).fill(null),
      forkColliders: new Array(physics.forkColliders?.length || (physics.forkCollider ? 1 : 0)).fill(null),
      weaponMode: physics.weaponMode || null,
      forkMode: physics.forkMode || null,
      forkMass: physics.forkMass,
      materialMode: Boolean(physics.materialMode),
      materialResponseEnabled: Boolean(physics.materialResponseEnabled),
      materialCuttingRequested: Boolean(physics.materialCuttingRequested),
      materialCuttingEnabled: Boolean(physics.materialCuttingEnabled),
    };
  }

  function makeVisiblePlaybackShell(workSim) {
    const visible = {
      ...workSim,
      // The visible shell contains display metadata only. The live WASM world
      // remains exclusively owned by trajectory.workSim until build finishes.
      physics: makePlaybackPhysicsDescriptor(workSim.physics),
      previousTips: new Map(), activeContactIds: new Set(), toothCooldowns: new Map(),
      rigActiveContacts: new Set(), rigWeaponEpisode: null,
    };
    state.sim = visible;
    applyPlaybackFrame(0, { render: false });
    state.sim.running = false;
    return visible;
  }

  function paintTrajectoryProgress(force = false) {
    const trajectory = state.trajectory;
    if (!trajectory?.building) return;
    const now = performance.now();
    if (!force && now - trajectory.lastProgressPaint < 80) return;
    trajectory.lastProgressPaint = now;
    const endTime = simulationEndTime();
    const computedTime = clamp(trajectory.workSim.time, 0, endTime);
    $('#timelineProgress').style.width = `${computedTime / endTime * 100}%`;
    setText('timelineReadout', `正在计算 ${computedTime.toFixed(4)} / ${endTime.toFixed(4)} s`);
    setText('timelineEndLabel', `${endTime.toFixed(4)} s`);
    updateStatus(`正在计算设定轨迹 ${Math.round(computedTime / endTime * 100)}%`, 'running');
  }

  function finishTrajectoryBuild(generation) {
    const trajectory = state.trajectory;
    if (!trajectory?.building || trajectory.generation !== generation) return;
    const terminalFailureDomain = resolvedFailureDomain(trajectory.workSim);
    trajectory.building = false; trajectory.ready = true; state.trajectoryTimer = 0;
    trajectory.duration = trajectory.frames[trajectory.frames.length - 1]?.time || 0;
    if (trajectory.workSim?.physics) {
      disposeTargetPhysics(trajectory.workSim.physics);
      trajectory.workSim.physics = null;
    }
    trajectory.workSim = null;
    state.eventEntries = trajectory.events;
    const requested = clamp(trajectory.pendingSeekTick || 0, 0, trajectory.frames.length - 1);
    applyPlaybackFrame(requested, { render: false });
    state.sim.running = false; state.accumulator = 0; state.lastFrame = performance.now();
    const seek = $('#timelineSeek'); if (seek) seek.disabled = false;
    const clear = $('#clearEvents'); if (clear) clear.disabled = false;
    updateReadouts(); drawScene();
    if (trajectory.autoPlay && requested < trajectory.frames.length - 1) setRunning(true);
    else if (terminalFailureDomain === 'material-model') updateStatus('材料模型安全停止（未提交临时刚体冲量）', 'warning');
    else if (terminalFailureDomain === 'rigid-solver') updateStatus('刚体求解安全停止（末固定步已回滚）', 'warning');
    else if (terminalFailureDomain === 'internal') updateStatus('内部异常安全停止（末固定步已回滚）', 'warning');
    else {
      const endTime = simulationEndTime();
      updateStatus(trajectory.duration >= endTime - FIXED_DT ? '设定时长轨迹已计算' : `轨迹在 ${trajectory.duration.toFixed(4)} s 停止`, trajectory.duration >= endTime - FIXED_DT ? 'ready' : 'warning');
    }
  }

  function runTrajectoryChunk(generation) {
    const trajectory = state.trajectory;
    if (!trajectory?.building || trajectory.generation !== generation) return;
    const visibleSim = state.sim; const started = performance.now();
    const deadline = started + TRAJECTORY_CHUNK_BUDGET_MS; let ticks = 0;
    state.sim = trajectory.workSim;
    try {
      while (!state.sim.completed && ticks < TRAJECTORY_CHUNK_MAX_TICKS) {
        if (ticks > 0 && performance.now() >= deadline) break;
        if (!state.sim?.physics?.rigModel) {
          stepPhysics();
          trajectory.frames.push(capturePlaybackFrame(trajectory));
          ticks += 1;
          continue;
        }
        const task = trajectory.pendingRigTick
          || beginRigTickTask({ trajectory, generation });
        trajectory.pendingRigTick = task;
        const result = resumeRigTickTask(task, deadline, state.qaRigBoundaryLimit || Infinity);
        if (result.status === 'yielded') break;
        trajectory.pendingRigTick = null;
        if (result.safeTerminal !== false) trajectory.frames.push(capturePlaybackFrame(trajectory));
        ticks += 1;
      }
      trajectory.workSim = state.sim;
    } catch (error) {
      const task = trajectory.pendingRigTick;
      if (task) {
        const result = abortRigTickTask(task, error);
        trajectory.pendingRigTick = null;
        if (result.safeTerminal !== false) trajectory.frames.push(capturePlaybackFrame(trajectory));
      } else {
        trajectory.workSim.completed = true;
        trajectory.workSim.failureDomain = 'internal';
        trajectory.workSim.solverDomainStopped = false;
        trajectory.workSim.modelDomainStopped = false;
        trajectory.workSim.creationError = error?.message || '轨迹计算失败';
        trajectory.workSim.lastRigFailure = {
          failureDomain: 'internal',
          invalidRoles: [trajectory.workSim.creationError],
          exceptionName: error?.name || 'Error',
        };
        addEvent(`轨迹计算失败：${trajectory.workSim.creationError}`, 'warning');
      }
    } finally {
      trajectory.callbackCount += 1;
      trajectory.maxCallbackMs = Math.max(trajectory.maxCallbackMs, performance.now() - started);
      if (trajectory.pendingRigTick) trajectory.yieldCount += 1;
      state.sim = visibleSim;
    }
    if (!state.trajectory || state.trajectory.generation !== generation) return;
    paintTrajectoryProgress();
    if (trajectory.workSim.completed) { finishTrajectoryBuild(generation); return; }
    state.trajectoryTimer = window.setTimeout(() => runTrajectoryChunk(generation), 0);
  }

  function startTrajectoryBuild({ autoPlay = false, pendingSeekTick = 0 } = {}) {
    if (state.trajectory?.building) {
      state.trajectory.autoPlay ||= autoPlay;
      state.trajectory.pendingSeekTick = pendingSeekTick;
      return;
    }
    if (state.trajectory?.ready) return;
    if (state.sim.creationError) { showToast(state.sim.creationError, 'error'); return; }
    disposeTargetPhysics(state.sim?.physics);
    state.metrics = computeMetrics();
    const workSim = createSimulation(); workSim.running = false;
    if (workSim.creationError) {
      state.sim = workSim;
      addEvent(`无法计算轨迹：${workSim.creationError}`, 'warning');
      updateReadouts(); drawScene();
      showToast(workSim.creationError, 'error');
      return;
    }
    const generation = ++state.trajectoryGeneration;
    const trajectory = {
      generation, building: true, ready: false, autoPlay, pendingSeekTick, workSim,
      frames: [], events: state.eventEntries.map((entry) => ({ ...entry })), eventFloor: 0,
      lastProgressPaint: 0, lastImpactSource: undefined, lastImpactClone: null,
      pendingRigTick: null, callbackCount: 0, yieldCount: 0, maxCallbackMs: 0,
    };
    state.trajectory = trajectory;
    state.sim = workSim;
    trajectory.frames.push(capturePlaybackFrame(trajectory));
    state.sim = makeVisiblePlaybackShell(workSim);
    state.playheadTick = 0; state.accumulator = 0;
    const seek = $('#timelineSeek'); if (seek) seek.disabled = true;
    const clear = $('#clearEvents'); if (clear) clear.disabled = true;
    updateReadouts(); drawScene(); paintTrajectoryProgress(true);
    state.trajectoryTimer = window.setTimeout(() => runTrajectoryChunk(generation), 0);
  }

  function enterDirectPhysicsModeForTesting() {
    const trajectory = state.trajectory;
    if (!trajectory) return;
    const targetTick = trajectory.ready ? state.playheadTick : 0;
    invalidateTrajectory();
    disposeTargetPhysics(state.sim?.physics);
    state.eventEntries = []; renderEventEntries();
    state.metrics = computeMetrics(); state.sim = createSimulation();
    for (let index = 0; index < targetTick && !state.sim.completed; index += 1) stepPhysics();
    state.sim.running = false; state.accumulator = 0;
  }

  function seekSimulation(seconds, announce = false) {
    const requested = clamp(number(seconds), 0, simulationEndTime());
    const tick = Math.round(requested / FIXED_DT);
    if (state.trajectory?.building) {
      state.trajectory.pendingSeekTick = tick;
      return;
    }
    if (!state.trajectory?.ready) {
      startTrajectoryBuild({ pendingSeekTick: tick });
      return;
    }
    state.sim.running = false; state.accumulator = 0;
    applyPlaybackFrame(tick, { render: false });
    state.lastFrame = performance.now();
    updateReadouts(); drawScene(); updateStatus(`已定位 t = ${state.sim.time.toFixed(4)} s`, 'paused');
    if (announce) showToast(`已定位至 ${state.sim.time.toFixed(4)} s；直接读取已计算轨迹。`);
  }
  function setRunning(value) {
    if (!value) {
      if (state.trajectory?.building) state.trajectory.autoPlay = false;
      state.sim.running = false; state.accumulator = 0;
      updateStatus('回放已暂停', 'paused'); updateReadouts();
      return;
    }
    if (state.trajectory?.building) { state.trajectory.autoPlay = true; paintTrajectoryProgress(true); return; }
    if (!state.trajectory?.ready) { startTrajectoryBuild({ autoPlay: true }); return; }
    if (state.playheadTick >= state.trajectory.frames.length - 1) {
      const terminal = state.trajectory.frames[state.trajectory.frames.length - 1];
      const terminalFailureDomain = resolvedFailureDomain(terminal);
      if (terminalFailureDomain) {
        applyPlaybackFrame(state.trajectory.frames.length - 1, { render: false });
        updateStatus(terminalFailureDomain === 'internal'
          ? '内部异常安全停止：末固定步已回滚'
          : (terminalFailureDomain === 'material-model'
            ? '材料模型安全停止：该接触未计算、未提交刚体冲量'
            : '刚体求解安全停止：末固定步已回滚'), 'warning');
        showToast(terminalFailureDomain === 'internal'
          ? '计算内部异常；安全帧已保留，异常尝试态未提交。请查看事件记录。'
          : (terminalFailureDomain === 'material-model'
            ? '当前材料模型不能可靠计算这次接触；安全帧已保留。可修改工况，或显式选择“理想刚体上限对照”。'
            : '该工况超出当前刚体求解域；安全帧已保留，请修改几何或工况。'), 'warning');
        updateReadouts(); drawScene();
        return;
      }
      applyPlaybackFrame(0, { render: false });
    }
    state.sim.running = true; state.lastFrame = performance.now(); state.accumulator = 0;
    updateStatus('轨迹回放中', 'running'); updateReadouts();
  }
  function stepOnce() {
    if (state.sim.running) setRunning(false);
    if (state.trajectory?.building) return;
    if (!state.trajectory?.ready) { startTrajectoryBuild({ pendingSeekTick: 1 }); return; }
    applyPlaybackFrame(state.playheadTick + 1, { render: false });
    state.sim.running = false; state.lastFrame = performance.now();
    updateStatus(`单步至 t = ${state.sim.time.toFixed(4)} s`, 'paused'); updateReadouts(); drawScene();
  }
  function restoreDefaults() {
    Object.assign(state.params, DEFAULTS);
    state.appliedRobotPreset = DEFAULTS.robotPreset;
    state.drawings.shovel = defaultDrawing('shovel');
    state.drawings.weapon = defaultDrawing('weapon');
    alignGeometryToFloor(); syncInputs(); renderDrawingStatus(); clearEvents(); resetSimulation();
    addEvent('已恢复内置：新单牙大武器.DXF 与 厚叉子.DXF。');
    showToast('已恢复内置 CAD 模型和 1.36 kg 起点。');
  }

  function animationFrame(time) {
    if (!state.lastFrame) state.lastFrame = time;
    const elapsed = Math.min(.08, Math.max(0, (time - state.lastFrame) / 1000)); state.lastFrame = time;
    const trajectory = state.trajectory;
    if (state.sim.running && trajectory?.ready) {
      state.accumulator += elapsed * positive(state.params.timeScale, 1);
      const ticks = Math.floor(state.accumulator / FIXED_DT);
      if (ticks > 0) {
        state.accumulator -= ticks * FIXED_DT;
        const next = Math.min(state.playheadTick + ticks, trajectory.frames.length - 1);
        applyPlaybackFrame(next, { render: false, preserveRunning: true });
        if (next >= trajectory.frames.length - 1) {
          state.sim.running = false; state.accumulator = 0;
          const failureDomain = resolvedFailureDomain(state.sim);
          updateStatus(failureDomain === 'internal'
            ? '内部异常安全停止：末固定步已回滚'
            : (failureDomain === 'material-model'
              ? '材料模型安全停止：该接触未计算、未提交刚体冲量'
              : (failureDomain === 'rigid-solver'
                ? '刚体求解安全停止：末固定步已回滚'
                : '设定时长轨迹回放完成')),
          failureDomain ? 'warning' : 'paused');
        }
        updateReadouts();
      }
    }
    drawScene(); window.requestAnimationFrame(animationFrame);
  }

  function bindEvents() {
    $$('[data-param]').forEach((input) => input.addEventListener(input.tagName === 'SELECT' || input.type === 'checkbox' ? 'change' : 'input', handleParameterChange));
    $$('#shovelFile, #weaponFile').forEach((input) => input.addEventListener('change', async (event) => { const file = event.target.files?.[0]; if (file) await importDrawing(file, input.id === 'shovelFile' ? 'shovel' : 'weapon'); event.target.value = ''; }));
    $('#applyRobotPreset').addEventListener('click', applyRobotPreset); $('#restoreDefaults').addEventListener('click', restoreDefaults); $('#setAxis').addEventListener('click', () => setAxisPicking(!state.axisPicking));
    $('#useDerivedRadius').addEventListener('click', () => { state.params.tipRadius = Math.max(1, Math.round(getMaxWeaponRadius() * 1000)); syncInputs(); resetAfterChange('图纸推导刀尖半径'); });
    $('#toggleGrid').addEventListener('click', (event) => { state.showGrid = !state.showGrid; event.currentTarget.classList.toggle('active', state.showGrid); drawScene(); });
    $('#toggleEnvelope').addEventListener('click', (event) => { state.showEnvelope = !state.showEnvelope; event.currentTarget.classList.toggle('active', state.showEnvelope); drawScene(); });
    $('#zoomIn').addEventListener('click', () => adjustZoom(1.25)); $('#zoomOut').addEventListener('click', () => adjustZoom(.8)); $('#fitScene').addEventListener('click', () => { state.camera.zoom = 1; drawScene(); showToast('场景已自动适配。'); });
    $('#resetSimulation').addEventListener('click', () => { clearEvents(); resetSimulation(); addEvent('已手动重置仿真。'); }); $('#playPause').addEventListener('click', () => setRunning(!state.sim.running)); $('#stepSimulation').addEventListener('click', stepOnce); $('#clearEvents').addEventListener('click', clearEvents);
    $('#openDwgHelp').addEventListener('click', () => modal('dwgModal', true)); $('#openModelNotes').addEventListener('click', () => modal('notesModal', true)); $$('[data-close-modal]').forEach((button) => button.addEventListener('click', () => modal('dwgModal', false))); $('#modalBackdrop').addEventListener('click', (event) => { if (event.target === event.currentTarget) modal('dwgModal', false); });
    state.canvas.addEventListener('pointermove', onCanvasMove); state.canvas.addEventListener('pointerdown', onCanvasClick); state.canvas.addEventListener('wheel', (event) => { event.preventDefault(); adjustZoom(event.deltaY < 0 ? 1.14 : .88); }, { passive: false });
    window.addEventListener('resize', resizeCanvas); window.addEventListener('keydown', (event) => { const active = document.activeElement; if (active && ['INPUT', 'SELECT', 'TEXTAREA'].includes(active.tagName)) return; if (event.code === 'Space') { event.preventDefault(); setRunning(!state.sim.running); } else if (event.key === 'ArrowRight') { event.preventDefault(); stepOnce(); } else if (event.key.toLowerCase() === 'r') { event.preventDefault(); clearEvents(); resetSimulation(); addEvent('已通过快捷键重置仿真。'); } else if (event.key === 'Escape' && state.axisPicking) setAxisPicking(false); });
  }

  async function importDrawingChecked(file, role) {
    if (!file.name.toLowerCase().endsWith('.dxf')) { showToast('此版本请先将图纸导出为 ASCII DXF。', 'warning'); return; }
    // File decoding is asynchronous. Cancel any in-flight trajectory now so
    // it cannot finish against geometry that is about to be replaced.
    if (state.trajectory) resetSimulation();
    const previousDrawing = state.drawings[role];
    const previousRadius = state.params.tipRadius;
    try {
      const raw = await file.text();
      state.drawings[role] = createImportedDrawing(role, file.name, raw);
      if (role === 'weapon') state.params.tipRadius = Math.max(1, Math.round(getMaxWeaponRadius() * 1000));
      if (!validateGroundClearance($('[data-param="weaponSceneY"]'))) {
        state.drawings[role] = previousDrawing;
        state.params.tipRadius = previousRadius;
        syncInputs(); drawScene();
        return;
      }
      syncInputs(); renderDrawingStatus(); clearEvents(); resetSimulation();
      addEvent(`已导入 ${role === 'shovel' ? '叉子' : '武器'} DXF：${file.name}`);
    showToast('DXF 已导入，保留 CAD (0,0) mm 作为局部原点。');
    } catch (error) {
      state.drawings[role] = previousDrawing;
      state.params.tipRadius = previousRadius;
      const card = $(`#${role}DrawingCard`);
      card.classList.add('error');
      $(`#${role}DrawingStatus`).textContent = `解析失败：${error.message}`;
      showToast(`DXF 解析失败：${error.message}`, 'error');
    }
  }

  function bindEventsV2() {
    $$('[data-collapsible] > .group-title-row').forEach((heading) => {
      const section = heading.parentElement;
      heading.setAttribute('role', 'button'); heading.setAttribute('tabindex', '0'); heading.setAttribute('aria-expanded', String(!section.classList.contains('collapsed')));
      const toggle = () => {
        const collapsed = section.classList.toggle('collapsed');
        heading.setAttribute('aria-expanded', String(!collapsed));
      };
      heading.addEventListener('click', (event) => { if (!event.target.closest('button, input, select, label, a')) toggle(); });
      heading.addEventListener('keydown', (event) => { if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); toggle(); } });
    });
    $$('[data-param]').forEach((input) => {
      const key = input.dataset.param;
      if (input.type === 'number') {
        const commitDraft = () => {
          const committedText = String(state.params[key] ?? '');
          if (input.dataset.numericDraft !== 'dirty' && input.value === committedText) return;
          delete input.dataset.numericDraft;
          handleParameterChange({ currentTarget: input });
        };
        input.addEventListener('input', () => {
          // Empty, a leading minus sign, and other intermediate number strings
          // are legitimate editing states.  They are validated only on commit.
          input.dataset.numericDraft = 'dirty';
          input.closest('.field')?.classList.remove('invalid');
        });
        input.addEventListener('change', commitDraft);
        input.addEventListener('blur', commitDraft);
        input.addEventListener('keydown', (event) => {
          if (event.key === 'Enter') {
            event.preventDefault();
            commitDraft();
          } else if (event.key === 'Escape' && input.dataset.numericDraft === 'dirty') {
            event.preventDefault();
            delete input.dataset.numericDraft;
            input.value = state.params[key];
            input.closest('.field')?.classList.remove('invalid');
          }
        });
        return;
      }
      const eventName = input.tagName === 'SELECT' || input.type === 'checkbox' ? 'change' : 'input';
      input.addEventListener(eventName, handleParameterChange);
    });
    $$('#shovelFile, #weaponFile').forEach((input) => input.addEventListener('change', async (event) => {
      const file = event.target.files?.[0];
      if (file) await importDrawingChecked(file, input.id === 'shovelFile' ? 'shovel' : 'weapon');
      event.target.value = '';
    }));
    $('#applyRobotPreset').addEventListener('click', applyRobotPreset);
    $('#restoreDefaults').addEventListener('click', () => { setAdvancedEnabled(false); restoreDefaults(); });
    $('#toggleAdvanced').addEventListener('click', () => setAdvancedEnabled(!state.advancedEnabled));
    $('#setAxis').addEventListener('click', () => setAxisPicking(!state.axisPicking));
    $('#useDerivedRadius').addEventListener('click', () => {
      const previous = state.params.tipRadius;
      state.params.tipRadius = Math.max(1, Math.round(getMaxWeaponRadius() * 1000));
      if (!validateGroundClearance($('[data-param="tipRadius"]'))) { state.params.tipRadius = previous; syncInputs(); return; }
      syncInputs(); resetAfterChange('图纸推导刀尖半径');
    });
    $('#toggleGrid').addEventListener('click', (event) => { state.showGrid = !state.showGrid; event.currentTarget.classList.toggle('active', state.showGrid); drawScene(); });
    $('#toggleEnvelope').addEventListener('click', (event) => { state.showEnvelope = !state.showEnvelope; event.currentTarget.classList.toggle('active', state.showEnvelope); drawScene(); });
    $('#zoomIn').addEventListener('click', () => adjustZoom(1.25));
    $('#zoomOut').addEventListener('click', () => adjustZoom(.8));
    $('#fitScene').addEventListener('click', fitScene);
    $('#resetSimulation').addEventListener('click', () => { clearEvents(); resetSimulation(); addEvent('已手动重置仿真。'); });
    $('#playPause').addEventListener('click', () => setRunning(!state.sim.running));
    $('#stepSimulation').addEventListener('click', stepOnce);
    const timelineSeek = $('#timelineSeek');
    timelineSeek?.addEventListener('pointerdown', () => { if (state.sim.running) setRunning(false); });
    timelineSeek?.addEventListener('input', (event) => seekSimulation(number(event.currentTarget.value) / 1000));
    timelineSeek?.addEventListener('change', (event) => seekSimulation(number(event.currentTarget.value) / 1000, true));
    $('#clearEvents').addEventListener('click', clearEvents);
    $('#openDwgHelp').addEventListener('click', () => modal('dwgModal', true));
    $('#openModelNotes').addEventListener('click', () => modal('notesModal', true));
    $$('[data-close-modal]').forEach((button) => button.addEventListener('click', () => modal('dwgModal', false)));
    $('#modalBackdrop').addEventListener('click', (event) => { if (event.target === event.currentTarget) modal('dwgModal', false); });
    state.canvas.addEventListener('pointermove', onCanvasMove);
    state.canvas.addEventListener('pointerdown', onCanvasPointerDown);
    state.canvas.addEventListener('pointerup', onCanvasPointerUp);
    state.canvas.addEventListener('pointercancel', onCanvasPointerUp);
    state.canvas.addEventListener('wheel', (event) => { event.preventDefault(); adjustZoom(event.deltaY < 0 ? 1.14 : .88, event); }, { passive: false });
    state.canvas.addEventListener('dblclick', fitScene);
    window.addEventListener('resize', resizeCanvas);
    window.addEventListener('keydown', (event) => {
      const active = document.activeElement;
      if (active && ['INPUT', 'SELECT', 'TEXTAREA'].includes(active.tagName)) return;
      if (event.code === 'Space') { event.preventDefault(); setRunning(!state.sim.running); }
      else if (event.key === 'ArrowRight') { event.preventDefault(); stepOnce(); }
      else if (event.key.toLowerCase() === 'f') { event.preventDefault(); fitScene(); }
      else if (event.key.toLowerCase() === 'r') { event.preventDefault(); clearEvents(); resetSimulation(); addEvent('已通过快捷键重置仿真。'); }
      else if (event.key === 'Escape' && state.axisPicking) setAxisPicking(false);
    });
  }

  async function initialise() {
    await initialiseRapier();
    if (window.__qaRapierInstrumentationReady) await window.__qaRapierInstrumentationReady;
    state.canvas = $('#simCanvas'); state.ctx = state.canvas.getContext('2d'); state.drawings.shovel = defaultDrawing('shovel'); state.drawings.weapon = defaultDrawing('weapon'); alignGeometryToFloor();
    state.metrics = computeMetrics(); state.sim = createSimulation(); setAdvancedEnabled(false); normaliseEngineeringReadouts(); syncInputs(); renderDrawingStatus(); bindEventsV2(); clearEvents(); addEvent('载入内置：新单牙大武器.DXF 与 厚叉子.DXF；场景 (0,0) mm 独立，叉子只相对武器局部原点定位。'); if (state.rapier) { const backendLabel = state.rapierBackend === 'simd' ? 'SIMD 加速' : '兼容'; addEvent(`物理：Rapier 2D ${backendLabel}后端联立求解底盘、转轴武器、真实 DXF 刀/叉、靶子与支撑地面。`, state.rapierFallbackReason ? 'warning' : 'info'); if (state.rapierFallbackReason) addEvent(`当前浏览器未启用 SIMD，已自动回退兼容后端：${state.rapierFallbackReason}`, 'warning'); } else addEvent(`物理引擎未启用：${state.rapierError || '使用保底求解器'}`, 'warning'); resizeCanvas(); updateReadouts(); if (state.rapier) updateStatus(state.rapierBackend === 'simd' ? 'Rapier SIMD 已就绪' : 'Rapier 兼容后端已就绪', state.rapierFallbackReason ? 'warning' : 'ready'); else updateStatus('物理引擎未启用', 'warning');
    window.BiteSim = {
      version: '1.0.2',
      reset: () => { clearEvents(); resetSimulation(); },
      setParams(values = {}) {
        const previous = { ...state.params };
        Object.entries(migrateLegacyPublicParams(values)).forEach(([key, value]) => {
          if (Object.prototype.hasOwnProperty.call(DEFAULTS, key)) state.params[key] = value;
        });
        const geometryStatus = parameterGeometryStatus();
        const clearanceStatus = groundClearanceStatus();
        const candidateMetrics = computeMetrics();
        const massError = weaponMassConsistencyError(
          candidateMetrics,
          positive(state.params.forkMass, DEFAULTS.forkMass),
          positive(state.params.robotMass, .01),
        );
        if (!geometryStatus.valid || !clearanceStatus.valid || massError) {
          state.params = previous; syncInputs(); renderDrawingStatus();
          throw new Error(!geometryStatus.valid
            ? geometryStatus.reason
            : (!clearanceStatus.valid ? clearanceMessage(clearanceStatus) : massError));
        }
        syncInputs(); renderDrawingStatus(); resetSimulation();
        return { ...state.params };
      },
      snapshotRoundTrip() {
        const physics = state.sim?.physics;
        const snapshot = snapshotRapierRig(physics);
        const ok = Boolean(snapshot && restoreRapierRig(physics, snapshot));
        if (ok) syncRigStateFromPhysics();
        const weaponColliders = physics?.weaponColliders || [];
        const forkColliders = physics?.forkColliders || [];
        return {
          ok,
          weaponColliderCount: weaponColliders.length,
          forkColliderCount: forkColliders.length,
          weaponHandlesMapped: weaponColliders.every((collider) => physics.weaponColliderMeta?.has(collider.handle)),
          forkHandlesMapped: forkColliders.every((collider) => physics.forkColliderMeta?.has(collider.handle)),
          forkBodyRestored: Boolean(physics?.forkBody),
          forkJointRestored: Boolean(physics?.forkJoint),
          forkGroundRestored: Boolean(physics?.forkGroundCollider),
        };
      },
      advance(steps = 1) {
        enterDirectPhysicsModeForTesting();
        state.sim.running = false;
        for (let i = 0; i < Math.max(1, Math.floor(Number(steps) || 1)) && !state.sim.completed; i += 1) stepPhysics();
        updateReadouts(); drawScene();
        return {
          time: state.sim.time,
          hits: state.sim.hitCount,
          toothHits: state.sim.hitCount,
          bodyImpacts: state.sim.bodyImpactCount,
          forkContact: state.sim.forkContact,
          forkEngaged: state.sim.forkEngaged,
          targetLaunched: state.sim.targetLaunched,
          targetVelocity: { ...state.sim.target.vel },
          solverDomainStopped: Boolean(state.sim.solverDomainStopped),
          modelDomainStopped: Boolean(state.sim.modelDomainStopped),
          failureDomain: resolvedFailureDomain(state.sim),
        };
      },
      getMetrics: () => ({ ...state.metrics }),
      getState: () => ({
        time: state.sim.time,
        completed: Boolean(state.sim.completed),
        failureDomain: resolvedFailureDomain(state.sim),
        driveSpeed: state.sim.driveSpeed,
        driveSlip: state.sim.driveSlip,
        weaponOmega: state.sim.weaponOmega,
        weaponEnergy: .5 * state.metrics.inertia * state.sim.weaponOmega ** 2,
        weaponMass: state.sim.physics?.weaponMass ?? state.metrics.weaponMass,
        chassisMass: state.sim.physics?.chassisMass ?? state.metrics.chassisMassBudget,
        target: {
          pos: { ...state.sim.target.pos },
          // Preserve the internal SI pose while making the public display unit
          // explicit for diagnostics and UI regression tests.
          positionMm: {
            x: state.sim.target.pos.x * 1000,
            y: state.sim.target.pos.y * 1000,
          },
          vel: { ...state.sim.target.vel },
          angle: state.sim.target.angle,
          omega: state.sim.target.omega,
          grounded: Boolean(state.sim.target.grounded),
        },
        forkContact: state.sim.forkContact,
        forkEngaged: state.sim.forkEngaged,
        fork: {
          origin: { ...state.sim.forkOrigin },
          velocity: { ...state.sim.forkVelocity },
          angle: state.sim.forkAngle,
          omega: state.sim.forkOmega,
          grounded: Boolean(state.sim.forkGrounded),
          mass: state.sim.physics?.forkMass ?? positive(state.params.forkMass, .03),
        },
        targetPushedByFork: state.sim.targetPushedByFork,
        weaponScene: weaponSceneOrigin(),
        floorY: groundY(),
        targetSupportY: targetSupportY(),
        hitCount: state.sim.hitCount,
        toothHitCount: state.sim.hitCount,
        bodyImpactCount: state.sim.bodyImpactCount,
        lastImpact: state.sim.lastImpact ? {
          time: state.sim.lastImpact.time,
          impactKind: state.sim.lastImpact.impactKind,
          toothOrder: state.sim.lastImpact.toothOrder,
          bodyContact: Boolean(state.sim.lastImpact.bodyContact),
          toothHitNumber: state.sim.lastImpact.toothHitNumber,
          bodyImpactNumber: state.sim.lastImpact.bodyImpactNumber,
          sourceIndex: state.sim.lastImpact.sourceIndex,
          colliderIndex: state.sim.lastImpact.colliderIndex,
          classificationSourceIndex: state.sim.lastImpact.classificationSourceIndex,
          classificationColliderIndex: state.sim.lastImpact.classificationColliderIndex,
          classificationImpulse: state.sim.lastImpact.classificationImpulse,
          feedBite: state.sim.lastImpact.feedBite,
          penetration: state.sim.lastImpact.penetration,
          contactGap: state.sim.lastImpact.contactGap,
          impulse: state.sim.lastImpact.impulse,
          normalImpulse: state.sim.lastImpact.normalImpulse,
          tangentialImpulse: state.sim.lastImpact.tangentialImpulse,
          targetEnergyGain: state.sim.lastImpact.targetEnergyGain,
          rotorEnergyLoss: state.sim.lastImpact.rotorEnergyLoss,
          chassisEnergyLoss: state.sim.lastImpact.chassisEnergyLoss,
          forkEnergyLoss: state.sim.lastImpact.forkEnergyLoss,
          externalWork: state.sim.lastImpact.externalWork,
          materialWork: state.sim.lastImpact.materialWork,
          boundaryWork: state.sim.lastImpact.boundaryWork,
          constraintEnergyExchange: state.sim.lastImpact.constraintEnergyExchange,
          massRemovalEnergy: state.sim.lastImpact.massRemovalEnergy,
          unclassifiedEnergy: state.sim.lastImpact.unclassifiedEnergy,
          numericalEnergyGain: state.sim.lastImpact.numericalEnergyGain,
          energyTolerance: state.sim.lastImpact.energyTolerance,
          energyConverged: state.sim.lastImpact.energyConverged,
          materialTickConverged: state.sim.lastImpact.materialTickConverged,
          materialConvergenceChecks: state.sim.lastImpact.materialConvergenceChecks,
        } : null,
        cad: (() => {
          const geometry = weaponCadCollisionGeometry();
          return {
            sourceOutlinePoints: geometry.sourceOutline.length,
            colliderLoopPoints: state.sim.physics?.weaponLoop?.length ?? 0,
            toothCount: geometry.teeth.length,
            teeth: geometry.teeth.map((tooth) => ({ index: tooth.index, startIndex: tooth.startIndex, endIndex: tooth.endIndex })),
          };
        })(),
        activeGeometry: {
          weapon: {
            source: state.params.paramWeaponEnabled ? 'parametric' : 'cad',
            radius: activeWeaponRadius(),
            mass: state.sim.physics?.weaponMass ?? state.metrics.weaponMass,
            inertia: state.metrics.inertia,
            inertiaInputKgMm2: state.metrics.inertiaInput,
            inertiaDerivedFromMassAndGeometry: true,
            inertiaSource: state.metrics.weaponMassPropertySource,
            radiusOfGyration: state.metrics.weaponRadiusOfGyration,
            geometryCentroid: state.metrics.weaponMassShapeCentroid,
            geometryArea: state.metrics.weaponMassShapeArea,
            minimumMassFromInertiaAndRadius: state.metrics.rotorMassLowerBound,
            massInertiaRadiusConsistent: state.metrics.weaponMassConsistent,
            toothCount: activeWeaponToothCount(),
            ...(state.params.paramWeaponEnabled ? (() => {
              const geometry = parameterWeaponGeometry();
              return {
                rakeInputDeg: number(state.params.paramToothPhaseDeg),
                rakeEffectiveDeg: number(state.params.paramToothPhaseDeg),
                rakeConstructionDeg: geometry.constructionRake * 180 / Math.PI,
                workingFaceLength: geometry.workLength,
                supportThickness: geometry.width,
              };
            })() : {}),
            colliderCount: state.sim.physics?.weaponColliders?.length ?? (state.sim.physics?.weaponCollider ? 1 : 0),
            colliderMode: state.sim.physics?.weaponMode || null,
            closedCuttingGeometryAvailable: !state.params.paramWeaponEnabled && hasClosedWeaponCadOutline(),
            materialRemovalEnabled: Boolean(state.sim.physics?.materialCuttingEnabled),
            // Backward-compatible alias; unlike the old name, this now reports
            // actual runtime enablement rather than merely having a closed CAD.
            materialRemovalDefined: Boolean(state.sim.physics?.materialCuttingEnabled),
            materialDomain: state.params.paramWeaponEnabled
              ? parameterWeaponGeometry().materialValidity
              : (state.sim.physics?.materialCuttingRequested && !state.sim.physics?.materialCuttingEnabled
                ? '完整剩余材料边界的连续最早压缩 TOI 尚未验证；安全门强制非穿透零删料'
                : '闭合 CAD 牙瓣仅在连续最早压缩 TOI 安全门通过后可进入材料去除模型'),
          },
      fork: {
            source: state.params.paramForkEnabled ? 'parametric' : 'cad',
            ...(state.params.paramForkEnabled ? {
              localOrigin: { x: 0, y: 0 },
              tip: { ...parameterForkGeometry().tip },
              length: parameterForkGeometry().tipDistance,
            } : {}),
            colliderCount: state.sim.physics?.forkColliders?.length ?? (state.sim.physics?.forkCollider ? 1 : 0),
            colliderMode: state.sim.physics?.forkMode || null,
            initialAngle: state.sim.physics?.initialForkAngle ?? null,
            zeroThickness: Boolean(state.params.paramForkEnabled),
          },
        },
        physics: {
          mode: state.sim.creationError ? 'invalid-rig' : (state.sim.physics?.rigModel ? 'rapier-rig' : 'toi-fallback'),
          backend: state.rapierBackend,
          backendVersion: state.rapier?.version?.() || null,
          simd: state.rapierBackend === 'simd',
          fallback: Boolean(state.rapierFallbackReason),
          fallbackReason: state.rapierFallbackReason,
          lengthUnit: state.sim.physics?.world?.lengthUnit,
          contactSkin: RIG_CONTACT_SKIN,
          maxAcceptedPenetration: { ...state.sim.rigMaxPenetration },
          maxRejectedPenetration: { ...state.sim.rigRejectedPenetration },
          minVisibleFloorClearance: state.sim.rigMinFloorClearance,
          jointError: state.sim.rigJointError,
          forkJointError: state.sim.rigForkJointError,
          maxJointError: state.sim.rigMaxJointError,
          maxForkJointError: state.sim.rigMaxForkJointError,
          jointErrorTolerance: rigJointErrorTolerance(),
          forkJointErrorTolerance: rigForkJointErrorTolerance(),
          minForkFloorClearance: state.sim.rigForkMinFloorClearance,
          massBudget: {
            robot: positive(state.params.robotMass, .01),
            weapon: state.sim.physics?.weaponMass ?? state.metrics.weaponMass,
            fork: state.sim.physics?.forkMass ?? state.metrics.forkMassInput,
            chassis: state.sim.physics?.chassisMass ?? state.metrics.chassisMassBudget,
          },
          substepsLastTick: state.sim.rigSubstepsLastTick,
          failureDomain: resolvedFailureDomain(state.sim),
          solverDomainStopped: Boolean(state.sim.solverDomainStopped),
          modelDomainStopped: Boolean(state.sim.modelDomainStopped),
          creationError: state.sim.creationError,
          lastFailure: clonePlaybackPlain(state.sim.lastRigFailure),
          materialCuttingRequested: Boolean(state.sim.physics?.materialCuttingRequested),
          materialCuttingEnabled: Boolean(state.sim.physics?.materialCuttingEnabled),
          traceableCuttingContinuousToiEnabled: TRACEABLE_CUTTING_CONTINUOUS_TOI_ENABLED,
          runtimeContactMode: runtimeContactMode(),
          materialConvergenceLastTick: clonePlaybackPlain(state.sim.materialConvergenceLastTick),
        },
        material: (() => {
          const damage = state.sim.materialDamage; const depths = damage?.depths || []; const rightDepths = damage?.rightDepths || [];
          return {
            version: damage?.version || 0,
            depths: [...depths],
            rightDepths: [...rightDepths],
            segments: damage?.segments?.map((segments) => segments.map((segment) => [...segment])) || [],
            geometry: cloneMaterialGeometry(damage?.geometry),
            maxDepth: depths.length || rightDepths.length ? Math.max(0, ...depths, ...rightDepths) : 0,
            cutCount: damage?.cuts?.length || 0,
            activeTooth: damage?.activeTooth ?? null,
            removedArea: state.sim.materialStats?.removedArea || 0,
            removedVolume: state.sim.materialStats?.removedVolume || 0,
            removedMass: state.sim.materialStats?.removedMass || 0,
            work: state.sim.materialStats?.work || 0,
            deformationWork: state.sim.materialStats?.deformationWork || 0,
            maxIntrusion: state.sim.materialMaxIntrusion || 0,
          };
        })(),
        trajectory: {
          building: Boolean(state.trajectory?.building),
          ready: Boolean(state.trajectory?.ready),
          frames: state.trajectory?.frames?.length ?? 0,
          playheadTick: state.playheadTick,
          duration: state.trajectory?.duration ?? 0,
          requestedDuration: simulationEndTime(),
          fixedStep: FIXED_DT,
          maxDuration: MAX_SIM_TIME,
          completed: Boolean(state.sim.completed),
        },
      }),
    };
    window.BiteSim.importDxfText = (role, text, name = `${role}.dxf`) => {
      if (!['shovel', 'weapon'].includes(role)) throw new Error('role must be shovel or weapon');
      const previousDrawing = state.drawings[role]; const previousRadius = state.params.tipRadius;
      state.drawings[role] = createImportedDrawing(role, name, text);
      if (role === 'weapon') state.params.tipRadius = Math.round(getMaxWeaponRadius() * 1000);
      if (!groundClearanceStatus().valid) {
        state.drawings[role] = previousDrawing; state.params.tipRadius = previousRadius; syncInputs();
        throw new Error('Imported geometry would enter the floor. Adjust the field height or fork offset first.');
      }
      syncInputs(); renderDrawingStatus(); resetSimulation();
    };
    // Deterministic scheduling probes are exposed only for the local QA URL.
    if (new URLSearchParams(window.location.search).has('qa')) {
      window.BiteSim.__qaSetBoundaryLimit = (value) => {
        const parsed = Math.floor(Number(value));
        state.qaRigBoundaryLimit = Number.isFinite(parsed) && parsed > 0 ? parsed : null;
        return state.qaRigBoundaryLimit;
      };
      window.BiteSim.__qaStartTrajectory = () => startTrajectoryBuild({ autoPlay: false });
      window.BiteSim.__qaTrajectoryDiagnostics = () => {
        const trajectory = state.trajectory;
        return trajectory ? {
          generation: trajectory.generation,
          building: trajectory.building,
          ready: trajectory.ready,
          frames: trajectory.frames.length,
          duration: trajectory.duration,
          callbackCount: trajectory.callbackCount,
          yieldCount: trajectory.yieldCount,
          maxCallbackMs: trajectory.maxCallbackMs,
          pending: trajectory.pendingRigTick ? {
            tickStartTime: trajectory.pendingRigTick.tickStartTime,
            refinement: trajectory.pendingRigTick.refinement,
            attemptSubsteps: trajectory.pendingRigTick.attemptSubsteps,
            substepIndex: trajectory.pendingRigTick.substepIndex,
            elapsed: trajectory.pendingRigTick.elapsed,
            resumeCount: trajectory.pendingRigTick.resumeCount,
            yieldCount: trajectory.pendingRigTick.yieldCount,
            stagedEvents: trajectory.pendingRigTick.pendingEvents.length,
          } : null,
          events: clonePlaybackPlain(trajectory.events),
        } : null;
      };
      window.BiteSim.__qaCurrentFrame = () => clonePlaybackPlain(capturePlaybackFrame({
        frames: [], events: activeEventEntries(), lastImpactSource: undefined, lastImpactClone: null,
      }));
      window.BiteSim.__qaFrame = (index) => {
        const frames = state.trajectory?.frames || [];
        const bounded = clamp(Math.round(Number(index) || 0), 0, Math.max(0, frames.length - 1));
        return clonePlaybackPlain(frames[bounded] || null);
      };
      window.BiteSim.__qaApplyFrame = (index) => {
        const frames = state.trajectory?.frames || [];
        const bounded = clamp(Math.round(Number(index) || 0), 0, Math.max(0, frames.length - 1));
        applyPlaybackFrame(bounded, { render: false });
        updateReadouts(); drawScene();
        return clonePlaybackPlain(frames[bounded] || null);
      };
    }
    window.requestAnimationFrame(animationFrame);
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', initialise); else initialise();
})();
