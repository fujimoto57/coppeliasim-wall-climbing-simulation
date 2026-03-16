DESIRED = DESIRED or {
  pos = {
    LF = { -0.08178,  0.07979, -0.08000 },
    LM = { -0.11413, -0.00043, -0.06000 },
    LR = { -0.08014, -0.08313, -0.08000 },
    RR = {  0.08303, -0.08125, -0.06000 },
    RM = {  0.11425,  0.00141, -0.06000 },
    RF = {  0.08113,  0.08292, -0.08000 },
    -- Swing leg: set z > -0.075
  }
}

-- ===== Wall / base placement settings =====
local WALL = { alias='/240cmHighWall200cm', axis='X', thickness=0.15001 }
local BASE = {
  side =  1, clearance = 0.10, y = 0.00, z = 0.80,
  rpy_deg = {0,0,0}, followWall = true, initFromCurrent=false, referenceClearance=nil
}

-- Physics ON timing
local START = { enablePhysicsAfter = 0.10 }

-- ===== CSV export settings =====
local EXPORT = {
  -- Delay before exporting pose CSV after physics is enabled
  delayAfterPhysics       = 2.0,
  poseFilename            = 'pose_snapshot.csv',

  -- Start time for torque logging after physics is enable
  torqueStartAfterPhysics = 0.8,
  -- Torque logging interval
  torqueSampleEvery       = 0.02,
  torqueFilename          = 'torque_log.csv',
}

local _physicsEnabled=false; local _tEnablePhysics=nil
local _tExport=nil; local _poseExported=false

-- torque CSV control
local _tTorqueStart=nil
local _lastTorqueWrite=nil
local _torqueHeaderWritten=false

-- joint / leg handles
local J1,J2,J3 = {},{},{}
local legBase, footTip = {},{}
local legs = {'LF','LM','LR','RR','RM','RF'}
local function legIdxOf(name) for i,v in ipairs(legs) do if v==name then return i end end end
local function clamp(x,a,b) if x<a then return a elseif x>b then return x end return x end

-- wall normal acquisition
local function getWallPose()
  local wall = sim.getObject(WALL.alias)
  if wall==-1 then sim.addLog(sim.verbosity_errors,'wall not found: '..WALL.alias) return nil end
  local wc = sim.getObjectPosition(wall,-1)
  local m  = sim.getObjectMatrix(wall,-1)
  local col = (WALL.axis=='X' and 1) or (WALL.axis=='Y' and 2) or 3
  local idx = (col==1) and {1,5,9} or (col==2 and {2,6,10} or {3,7,11})
  local n = {m[idx[1]], m[idx[2]], m[idx[3]]}
  local L = math.sqrt(n[1]^2+n[2]^2+n[3]^2); if L>0 then n={n[1]/L,n[2]/L,n[3]/L} end
  return wc,n
end

-- convert world direction vector to object local coordinates
local function dirWorldToLocal(obj, v)
  local M = sim.getObjectMatrix(obj,-1)
  local X,Y,Z = {M[1],M[5],M[9]}, {M[2],M[6],M[10]}, {M[3],M[7],M[11]}
  return { X[1]*v[1]+X[2]*v[2]+X[3]*v[3],
           Y[1]*v[1]+Y[2]*v[2]+Y[3]*v[3],
           Z[1]*v[1]+Z[2]*v[2]+Z[3]*v[3] }
end

-- IK for 3-DOF leg
local Lcoxa, Lfemur, Ltibia
local function inverseKinematics(p)
  local xt = math.sqrt(p[1]^2 + p[2]^2) - Lcoxa
  local Lim= math.sqrt(xt^2 + p[3]^2)
  local c1 = (Lfemur^2 + Lim^2 - Ltibia^2) / (2*Lfemur*Lim)
  local c2 = (Lfemur^2 + Ltibia^2 - Lim^2) / (2*Lfemur*Ltibia)
  local a1 = math.atan2(p[2], p[1])
  local a2 = math.atan2(xt, math.abs(p[3])) + math.acos(clamp(c1,-1,1))
  local a3 = math.pi/2 - math.acos(clamp(c2,-1,1))
  return {a1,a2,a3}
end

-- place base with physics OFF
local function placeBase(robot)
  local wc,n = getWallPose(); if not wc then return end
  local off = (WALL.thickness*0.5 + BASE.clearance) * BASE.side
  sim.setObjectPosition(robot, -1, {wc[1]+n[1]*off, BASE.y, BASE.z})
  local yaw   = math.atan2(n[2], n[1]) + (BASE.side<0 and math.pi or 0)
  -- sim.setObjectOrientation(robot, -1, {math.rad(BASE.rpy_deg[1]), math.rad(BASE.rpy_deg[2]), yaw})
  local pr = sim.getObjectPosition(robot,-1)
  local d = (pr[1]-wc[1])*n[1] + (pr[2]-wc[2])*n[2] + (pr[3]-wc[3])*n[3] - BASE.side*(WALL.thickness*0.5)
  sim.addLog(sim.verbosity_scriptinfos, string.format('base clearance=%.3f m', d))
end

-- initialize DESIRED baseline
local function initFeetBaseline(robot, legs, legBase, footTip)
  DESIRED = DESIRED or {pos={}}; DESIRED.pos = DESIRED.pos or {}
  DESIRED._basePos = DESIRED._basePos or {}
  if DESIRED._initialized then return end
  for i,name in ipairs(legs) do
    local p = DESIRED.pos[name]
    if not p then
      local q = sim.getObjectPosition(footTip[i], legBase[i])
      DESIRED.pos[name] = {q[1],q[2],q[3]}
      DESIRED._basePos[name] = {q[1],q[2],q[3]}
    else
      DESIRED._basePos[name] = {p[1],p[2],p[3]}
    end
  end
  if not BASE.referenceClearance then
    local wc,n = getWallPose(); local p0 = sim.getObjectPosition(robot,-1)
    local clear0 = (p0[1]-wc[1])*n[1] + (p0[2]-wc[2])*n[2] + (p0[3]-wc[3])*n[3]
    BASE.referenceClearance = clear0 - BASE.side*(WALL.thickness*0.5)
  end
  DESIRED._initialized = true
end

-- apply wall-follow correction
local function applyWallFollow(legs, legBase)
  if not BASE.followWall then return end
  local _, nW = getWallPose()
  local d = BASE.clearance - (BASE.referenceClearance or 0)
  if math.abs(d) < 1e-9 then return end
  for i,name in ipairs(legs) do
    local nL = dirWorldToLocal(legBase[i], nW)
    local s  = -BASE.side * d
    local p0 = DESIRED._basePos[name]
    DESIRED.pos[name] = { p0[1] + s*nL[1], p0[2] + s*nL[2], p0[3] + s*nL[3] }
  end
end

-- compute angle displacement
local function angleDisplacement(legIdx, initAngle, targetAngle)
  local d = {}
  d[2] = initAngle[2] - targetAngle[2]
  d[3] = targetAngle[3] - initAngle[3]
  if legIdx==6 or legIdx==5 or legIdx==4 then
    d[1] = targetAngle[1] - initAngle[1]
  elseif legIdx==3 or legIdx==2 then
    if targetAngle[1] < 0 then d[1] = targetAngle[1]-initAngle[1]
    else d[1] = (targetAngle[1]-initAngle[1]) - 2*math.pi end
  else
    if targetAngle[1] < 0 then d[1] =  2*math.pi + (targetAngle[1]-initAngle[1])
    else d[1] = targetAngle[1] - initAngle[1] end
  end
  return d
end

-- switch all grippers ON/OFF via CustomDataBlock 
local function setAllGrippers(on)
  local names = {
    './LF_link_3_respondable','./LM_link_3_respondable','./LR_link_3_respondable',
    './RR_link_3_respondable','./RM_link_3_respondable','./RF_link_3_respondable'
  }
  for _,path in ipairs(names) do
    local h = sim.getObject(path)
    if h~=-1 then sim.writeCustomDataBlock(h,'gripperOn', on and 'on' or '') end
  end
end

local function setGripper(legName, on)
  local path = string.format('./%s_link_3_respondable', legName)
  local h = sim.getObject(path)
  if h~=-1 then sim.writeCustomDataBlock(h, 'gripperOn', on and 'on' or '') end
end

-- determine contact state for exportPoseCSV
local Z_SWING_TH = -0.075
local function isContactLeg(legName)
  local zdes = DESIRED._basePos[legName][3]
  return zdes <= Z_SWING_TH  -- ????true?????false
end

-- ====== utility: compute total mass / weight ======
local function getModelTotalMass(modelHandle)
  local total = 0.0
  -- traverse all Shape objects under the model: options=0
  for _,h in ipairs(sim.getObjectsInTree(modelHandle, sim.object_shape_type, 0)) do
     -- old API(sim.getShapeMass) may fail depending on environment, so also try getShapeMassAndInertia
    local ok, mass = pcall(sim.getShapeMass, h)
    if ok and type(mass) == 'number' then
      total = total + mass
    else
      local ok2, mass2 = pcall(function() return sim.getShapeMassAndInertia(h) end)
      if ok2 and type(mass2) == 'number' then total = total + mass2 end
    end
  end
  return total
end
-- ================================================

-- ---------- Graph stream initialization ---------- 
local gGraph, gStr = nil, {}
local function initTorqueGraph()
  gGraph = sim.getObject('/Graph')
  if gGraph == -1 then
    gGraph = sim.createGraph(0); sim.setObjectAlias(gGraph,'Graph',0)
  else
    if sim.clearGraph then sim.clearGraph(gGraph) end
  end
  gStr = {}
  local colors = {
    LF={1.00,0.20,0.20}, LM={0.20,1.00,0.20}, LR={0.20,0.40,1.00},
    RR={1.00,0.80,0.20}, RM={0.85,0.20,1.00}, RF={0.20,1.00,1.00},
  }
  for _,foot in ipairs(legs) do
    local c = colors[foot]
    gStr[foot] = {
      sim.addGraphStream(gGraph,string.format('%s_J1_torque (Nm)',foot),'Nm',0,c),
      sim.addGraphStream(gGraph,string.format('%s_J2_torque (Nm)',foot),'Nm',0,c),
      sim.addGraphStream(gGraph,string.format('%s_J3_torque (Nm)',foot),'Nm',0,c),
    }
  end
end

-- ---------- pose CSV export ----------
local function exportPoseCSV()
  if _poseExported then return end
  local robot = sim.getObject('./base') 
  local ref = sim.getObject('./ROBOT')
  if robot==-1 then return end

  local function posInBase(h) return sim.getObjectPosition(h, ref) end
  local function axisZInBase(j)
    local M = sim.getObjectMatrix(j, ref)
    return {M[3], M[7], M[11]}
  end

  local gW = sim.getArrayParam(sim.arrayparam_gravity)
  local gB = dirWorldToLocal(ref, {gW[1],gW[2],gW[3]})

  local sceneDir = sim.getStringParam(sim.stringparam_scene_path) or '.'
  local path = sceneDir..'/'..EXPORT.poseFilename
  local f = io.open(path, 'w')
  if not f then
    sim.addLog(sim.verbosity_errors,'[export] cannot write: '..path)
    return
  end

  f:write('# units: meters (robot base frame)\n')
  f:write(string.format('gravity_base,%.8f,%.8f,%.8f\n', gB[1],gB[2],gB[3]))
  f:write('leg,contact,foot_x,foot_y,foot_z,'..
        'j1_x,j1_y,j1_z,j1_axis_x,j1_axis_y,j1_axis_z,'..
        'j2_x,j2_y,j2_z,j2_axis_x,j2_axis_y,j2_axis_z,'..
        'j3_x,j3_y,j3_z,j3_axis_x,j3_axis_y,j3_axis_z\n')
  
  -- This is for diciding contact, ground or swing
  local Z_SWING_TH = -0.075

  for i,leg in ipairs(legs) do
    local pF = posInBase(footTip[i])
    local pJ1 = posInBase(J1[i])
    local pJ2 = posInBase(J2[i])
    local pJ3 = posInBase(J3[i])
    local ax1 = axisZInBase(J1[i])
    local ax2 = axisZInBase(J2[i])
    local ax3 = axisZInBase(J3[i])
    local zdes = DESIRED._basePos[leg][3]
    local contact = (zdes > Z_SWING_TH) and 0 or 1
    f:write(string.format('%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.8f,%.8f,%.8f,%.6f,%.6f,%.6f,%.8f,%.8f,%.8f,%.6f,%.6f,%.6f,%.8f,%.8f,%.8f\n',
      leg, contact, pF[1],pF[2],pF[3], pJ1[1],pJ1[2],pJ1[3], ax1[1],ax1[2],ax1[3], pJ2[1],pJ2[2],pJ2[3], ax2[1],ax2[2],ax2[3], pJ3[1],pJ3[2],pJ3[3], ax3[1],ax3[2],ax3[3]))
  end
  f:close()
  _poseExported = true
  sim.addLog(sim.verbosity_scriptinfos,'[export] wrote '..path)
end

-- ---------- torque CSV export ----------
local function _torqueCsvPath()
  local sceneDir = sim.getStringParam(sim.stringparam_scene_path) or '.'
  return sceneDir..'/'..EXPORT.torqueFilename
end

local function _writeTorqueHeader(path)
  local f = io.open(path,'w'); if not f then return false end
  f:write('t')
  for _,name in ipairs(legs) do
    f:write(string.format(',%s_J1,%s_J2,%s_J3', name, name, name))
  end
  f:write(',max_abs_tau_all_Nm,sum_tau_all_sq_Nm2\n'); f:close()
  return true
end

local function writeTorqueLine(t)
  local path = _torqueCsvPath()
  if not _torqueHeaderWritten then
    _torqueHeaderWritten = _writeTorqueHeader(path)
    if not _torqueHeaderWritten then
      sim.addLog(sim.verbosity_errors, '[export] cannot write: '..path); return
    end
  end
  local f = io.open(path,'a'); if not f then return end
  f:write(string.format('%.6f', t))
  
  local maxAbsTauAll = 0.0 
  local sumTauAllSq  = 0.0
  
  for i=1,6 do
    local t1 = -sim.getJointForce(J1[i]) or 0
    local t2 = -sim.getJointForce(J2[i]) or 0
    local t3 = -sim.getJointForce(J3[i]) or 0
    
    local taus = {t1, t2, t3}
    for k=1,3 do
      local a = math.abs(taus[k])
      if a > maxAbsTauAll then maxAbsTauAll = a end
      sumTauAllSq = sumTauAllSq + taus[k]*taus[k]
    end

    f:write(string.format(',%.9f,%.9f,%.9f', t1,t2,t3))
  end

  f:write(string.format(',%.9f,%.9f\n', maxAbsTauAll, sumTauAllSq)); f:close()
end

-- ========================= main =========================
local q_scene, firstAngle = {}, {}

function sysCall_init()
  sim = require('sim')
  sim.setBoolParam(sim.boolparam_dynamics_handling_enabled, false)

  local robot = sim.getObject('.')
  local baseHandle  = sim.getObject('./base')

 -- ====== debug output: total mass [kg] and weight [N] ======
  local m = getModelTotalMass(robot)                        -- mass [kg]
  local g = sim.getArrayParam(sim.arrayparam_gravity)       -- gravity vector [m/s^2]
  local gmag = math.sqrt(g[1]^2 + g[2]^2 + g[3]^2)          -- |g|
  local weightN = m * gmag                                  -- weight [N]
  sim.addLog(sim.verbosity_scriptinfos,
    string.format('[mass] total mass = %.6f kg, weight = %.6f N (|g|=%.6f m/s^2)',
      m, weightN, gmag))
  -- =====================================================

  -- get joint / tip handles
  J1[1]=sim.getObject('./LF_joint_1'); J2[1]=sim.getObject('./LF_joint_2'); J3[1]=sim.getObject('./LF_joint_3')
  J1[2]=sim.getObject('./LM_joint_1'); J2[2]=sim.getObject('./LM_joint_2'); J3[2]=sim.getObject('./LM_joint_3')
  J1[3]=sim.getObject('./LR_joint_1'); J2[3]=sim.getObject('./LR_joint_2'); J3[3]=sim.getObject('./LR_joint_3')
  J1[4]=sim.getObject('./RR_joint_1'); J2[4]=sim.getObject('./RR_joint_2'); J3[4]=sim.getObject('./RR_joint_3')
  J1[5]=sim.getObject('./RM_joint_1'); J2[5]=sim.getObject('./RM_joint_2'); J3[5]=sim.getObject('./RM_joint_3')
  J1[6]=sim.getObject('./RF_joint_1'); J2[6]=sim.getObject('./RF_joint_2'); J3[6]=sim.getObject('./RF_joint_3')
  for i=1,6 do
    legBase[i] = sim.getObject('./legBase'..i)
    footTip[i] = sim.getObject('./footTip'..i)
  end

  setAllGrippers(false)
  initFeetBaseline(robot, legs, legBase, footTip)
  placeBase(robot)
  applyWallFollow(legs, legBase)

  -- compute link lengths from leg 1
  local function dist(a,b) return math.sqrt((a[1]-b[1])^2+(a[2]-b[2])^2+(a[3]-b[3])^2) end
  local pJ1 = sim.getObjectPosition(J1[1],-1)
  local pJ2 = sim.getObjectPosition(J2[1],-1)
  local pJ3 = sim.getObjectPosition(J3[1],-1)
  local pFT = sim.getObjectPosition(footTip[1],-1)
  Lcoxa  = dist(pJ1,pJ2); Lfemur = dist(pJ2,pJ3); Ltibia = dist(pJ3,pFT)
  sim.addLog(sim.verbosity_scriptinfos,
    string.format("link lengths [m]: Lcoxa=%.4f  Lfemur=%.4f  Ltibia=%.4f",Lcoxa,Lfemur,Ltibia))

  -- IK initialization from DESIRED
  local initPosBase = {}
  for i=1,6 do
    initPosBase[i] = sim.getObjectPosition(footTip[i], legBase[i])
    q_scene[i] = { sim.getJointPosition(J1[i]), sim.getJointPosition(J2[i]), sim.getJointPosition(J3[i]) }
    firstAngle[i] = inverseKinematics(initPosBase[i])
  end
  for _,name in ipairs(legs) do
    if DESIRED.pos[name]==nil then
      local idx=legIdxOf(name); local p=initPosBase[idx]
      DESIRED.pos[name] = {p[1],p[2],p[3]}
    end
  end
  for i,name in ipairs(legs) do
    local qdes = inverseKinematics(DESIRED.pos[name])
    local dq   = angleDisplacement(i, firstAngle[i], qdes)
    local qabs = { q_scene[i][1]+dq[1], q_scene[i][2]+dq[2], q_scene[i][3]+dq[3] }
    sim.setJointPosition(J1[i], qabs[1]); sim.setJointPosition(J2[i], qabs[2]); sim.setJointPosition(J3[i], qabs[3])
    sim.setJointTargetPosition(J1[i], qabs[1]); sim.setJointTargetPosition(J2[i], qabs[2]); sim.setJointTargetPosition(J3[i], qabs[3])
  end

  initTorqueGraph()

  -- schedule CSV export after physics ON
  local t0 = sim.getSimulationTime()
  _tEnablePhysics = t0 + START.enablePhysicsAfter
  sim.addLog(sim.verbosity_scriptinfos, string.format('Pose ready (physics in %.2fs).', START.enablePhysicsAfter))
end

function sysCall_actuation()
  local t = sim.getSimulationTime()
  if (not _physicsEnabled) and _tEnablePhysics and t >= _tEnablePhysics then
    local robot = sim.getObject('.')
    for _,s in ipairs(sim.getObjectsInTree(robot, sim.object_shape_type, 0)) do sim.resetDynamicObject(s) end
    sim.setBoolParam(sim.boolparam_dynamics_handling_enabled, true)
    
    for _,legName in ipairs(legs) do
      setGripper(legName, isContactLeg(legName))  -- contact=ON / swing=OFF
    end
    
    _physicsEnabled = true
    -- start CSV timers
    _tExport      = t + EXPORT.delayAfterPhysics
    _tTorqueStart = t + EXPORT.torqueStartAfterPhysics
    sim.addLog(sim.verbosity_scriptinfos,'[start] physics enabled')
  end
end

function sysCall_sensing()
  -- graph update
  if gGraph then
    for i,foot in ipairs(legs) do
      sim.setGraphStreamValue(gGraph, gStr[foot][1], sim.getJointForce(J1[i]))
      sim.setGraphStreamValue(gGraph, gStr[foot][2], sim.getJointForce(J2[i]))
      sim.setGraphStreamValue(gGraph, gStr[foot][3], sim.getJointForce(J3[i]))
    end
  end

  local t = sim.getSimulationTime()

  -- export pose CSV once
  if (not _poseExported) and _tExport and t >= _tExport then
    exportPoseCSV()
  end

  -- periodic torque CSV logging
  if _physicsEnabled and _tTorqueStart and t >= _tTorqueStart then
    if (not _lastTorqueWrite) or (t - _lastTorqueWrite >= EXPORT.torqueSampleEvery - 1e-6) then
      writeTorqueLine(t)
      _lastTorqueWrite = t
    end
  end
end