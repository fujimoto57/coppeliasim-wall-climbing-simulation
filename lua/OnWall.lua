-- ===== Slip check states (world) =====
gripState = gripState or {}
anchorTipW = anchorTipW or {}
gripState = gripState or {} -- suction ON/OFF state

-- =========================
-- Keyframe CSV Logger (v1.1)
-- =========================
SCHEMA_VERSION = "v1.1"

-- CSV leg order (IMPORTANT):
-- leg0=RF(6), leg1=RM(5), leg2=RR(4), leg3=LR(3), leg4=LM(2), leg5=LF(1)
CSV_LEG_TO_SIM_LEG = {6, 5, 4, 3, 2, 1}

-- suction/contact state (log only)
contact = contact or {1,1,1,1,1,1}

-- output csv handle
logCsv = logCsv or nil

-- legacy per-step torque log (joint1 only). Default OFF.
ENABLE_LEGACY_JOINT1_LOG = false
file2 = file2 or nil

-- Torque logging mode:
-- "load"  : log -sim.getJointForce(...)  (recommended for comparing with Python load torque)
-- "motor" : log +sim.getJointForce(...)
TORQUE_LOG_MODE = "load"


local function csvEscape(s)
    if s == nil then return "" end
    s = tostring(s)
    if string.find(s, '[,"\n]') then
        s = '"' .. string.gsub(s, '"', '""') .. '"'
    end
    return s
end

local function maxAbsAndSumSq(arr)
    local maxAbs = 0.0
    local sumSq  = 0.0
    for i=1,#arr do
        local v = arr[i]
        local a = math.abs(v)
        if a > maxAbs then maxAbs = a end
        sumSq = sumSq + v*v
    end
    return maxAbs, sumSq
end

local function getPathStem(path)
    -- works for "/" and "\"
    local name = path:gsub("^.*[\\/]", "")
    local stem = name:gsub("%.[^%.]+$", "")
    return stem, name
end

-- v1.1: pose_hash (insurance)
-- raw string = rounded feet(COM_R) + contact. If bit ops available, shorten via simple FNV-1a like hash.
local function makePoseHash()
    local parts = {}
    local ROUND = 0.001 -- 1mm
    local function r(x) return math.floor(x/ROUND + 0.5) * ROUND end

    for csvIdx=1,6 do
        local leg = CSV_LEG_TO_SIM_LEG[csvIdx]
        local p = sim.getObjectPosition(simLegTips[leg], legBase) -- COM_R base
        parts[#parts+1] = string.format("%.3f_%.3f_%.3f_%d", r(p[1]), r(p[2]), r(p[3]), contact[leg] or 0)
    end
    local raw = table.concat(parts, "|")

    -- try to hash (short) if possible
    if bit32 and bit32.bxor then
        local h = 2166136261
        for i=1,#raw do
            h = bit32.bxor(h, raw:byte(i))
            h = (h * 16777619) % 4294967296
        end
        return string.format("%08x", h)
    elseif (_VERSION and (string.find(_VERSION, "5.3") or string.find(_VERSION, "5.4"))) then
        local h = 2166136261
        for i=1,#raw do
            h = (h ~ raw:byte(i)) % 4294967296
            h = (h * 16777619) % 4294967296
        end
        return string.format("%08x", h)
    else
        -- fallback: raw (still works as a robust key)
        return raw
    end
end

local function writeKeyframeRow(step_idx, motion_type, success, status)
    if not logCsv then return end

    local t_s = sim.getSimulationTime()

    -- COM position in world: use COM_R dummy (legBase) if it's truly COM frame
    local comW = sim.getObjectPosition(legBase, -1)

    -- base orientation in world: use base object
    local qW = sim.getObjectQuaternion(antBase, -1) -- {qx,qy,qz,qw}

    -- contact (leg0..5)
    local contactOut = {}
    for csvIdx=1,6 do
        local leg = CSV_LEG_TO_SIM_LEG[csvIdx]
        contactOut[#contactOut+1] = contact[leg] or 0
    end

    -- tau (18 columns) in leg0..5 order
    -- Note: Bullet/Newton returns motor-applied torque. For "load torque" comparison, flip sign.
    local sgn = (TORQUE_LOG_MODE == "load") and -1 or 1

    local tau = {}
    for csvIdx=1,6 do
        local leg = CSV_LEG_TO_SIM_LEG[csvIdx]
        tau[#tau+1] = sgn * sim.getJointForce(simJoint1[leg])
        tau[#tau+1] = sgn * sim.getJointForce(simJoint2[leg])
        tau[#tau+1] = sgn * sim.getJointForce(simJoint3[leg])
    end
    local maxAbs, sumSq = maxAbsAndSumSq(tau)

    -- feet (18 columns) in COM_R base, leg0..5 order
    local feet = {}
    for csvIdx=1,6 do
        local leg = CSV_LEG_TO_SIM_LEG[csvIdx]
        local p = sim.getObjectPosition(simLegTips[leg], legBase) -- COM_R
        feet[#feet+1] = p[1]
        feet[#feet+1] = p[2]
        feet[#feet+1] = p[3]
    end

    local pose_hash = makePoseHash()

    local row = {}
    row[#row+1] = csvEscape(SCHEMA_VERSION)
    row[#row+1] = tostring(step_idx)
    row[#row+1] = string.format("%.6f", t_s)
    row[#row+1] = csvEscape(motion_type or "")
    row[#row+1] = tostring(success and 1 or 0)
    row[#row+1] = csvEscape(status or "ok")

    row[#row+1] = string.format("%.6f", comW[1])
    row[#row+1] = string.format("%.6f", comW[2])
    row[#row+1] = string.format("%.6f", comW[3])

    row[#row+1] = string.format("%.6f", qW[1])
    row[#row+1] = string.format("%.6f", qW[2])
    row[#row+1] = string.format("%.6f", qW[3])
    row[#row+1] = string.format("%.6f", qW[4])

    for i=1,#contactOut do row[#row+1] = tostring(contactOut[i]) end
    for i=1,#tau do row[#row+1] = string.format("%.6f", tau[i]) end

    row[#row+1] = string.format("%.6f", maxAbs)
    row[#row+1] = string.format("%.6f", sumSq)

    for i=1,#feet do row[#row+1] = string.format("%.6f", feet[i]) end

    row[#row+1] = csvEscape(pose_hash)

    logCsv:write(table.concat(row, ",") .. "\n")
    logCsv:flush()
end

function sysCall_init()

    sim=require('sim')

    -- ===== Freeze all shapes (temporary) =====
    modelRoot = sim.getObject('.')  -- PhantomX(model)
    allShapes = sim.getObjectsInTree(modelRoot, sim.object_shape_type, 0)
    savedStatic = {}  -- shapeHandle -> saved static param

    function setAllShapesStatic(on)
        if not allShapes then return end
        for _,h in ipairs(allShapes) do
            if on then
                if savedStatic[h] == nil then
                    savedStatic[h] = sim.getObjectInt32Param(h, sim.shapeintparam_static)
                end
                sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
            else
                local prev = savedStatic[h]
                if prev ~= nil then
                    sim.setObjectInt32Param(h, sim.shapeintparam_static, prev)
                end
            end
        end
    end

    LFsuction = sim.getObject('./LF_link_3_respondable')
    LMsuction = sim.getObject('./LM_link_3_respondable')
    LRsuction = sim.getObject('./LR_link_3_respondable')
    RRsuction = sim.getObject('./RR_link_3_respondable')
    RMsuction = sim.getObject('./RM_link_3_respondable')
    RFsuction = sim.getObject('./RF_link_3_respondable')

    for kyaku=1,6,1 do
        setGripperOn(true,kyaku)
    end

    antBase=sim.getObject('./base')
    legBase=sim.getObject('./COM_R')
    sizeFactor=sim.getObjectSizeFactor(antBase)

    simJoint1={}
    simJoint2={}
    simJoint3={}
    simJoint1[1] = sim.getObject('./LF_joint_1')
    simJoint1[2] = sim.getObject('./LM_joint_1')
    simJoint1[3] = sim.getObject('./LR_joint_1')
    simJoint1[4] = sim.getObject('./RR_joint_1')
    simJoint1[5] = sim.getObject('./RM_joint_1')
    simJoint1[6] = sim.getObject('./RF_joint_1')
    simJoint2[1] = sim.getObject('./LF_joint_2')
    simJoint2[2] = sim.getObject('./LM_joint_2')
    simJoint2[3] = sim.getObject('./LR_joint_2')
    simJoint2[4] = sim.getObject('./RR_joint_2')
    simJoint2[5] = sim.getObject('./RM_joint_2')
    simJoint2[6] = sim.getObject('./RF_joint_2')
    simJoint3[1] = sim.getObject('./LF_joint_3')
    simJoint3[2] = sim.getObject('./LM_joint_3')
    simJoint3[3] = sim.getObject('./LR_joint_3')
    simJoint3[4] = sim.getObject('./RR_joint_3')
    simJoint3[5] = sim.getObject('./RM_joint_3')
    simJoint3[6] = sim.getObject('./RF_joint_3')

    simJoint1Pos={}
    simJoint2Pos={}
    simJoint3Pos={}
    for i=1,6,1 do
        simJoint1Pos[i]=sim.getObjectPosition(simJoint1[i],-1)
        simJoint2Pos[i]=sim.getObjectPosition(simJoint2[i],-1)
        simJoint3Pos[i]=sim.getObjectPosition(simJoint3[i],-1)
    end

    -- this is Coxa position within PhantomX
    baseJoint={}
    for i=1,6,1 do
        baseJoint[i]=sim.getObject('./leg'..i)
    end

    -- this is Coxa position within World
    BASEJOINT={}
    for i=1,6,1 do
        BASEJOINT[i]=sim.getObject('/COM_ref/legRef'..i)
    end

    simLegTips={}
    simLegTargets={}
    initialPos={}
    initialPosition={}
    for i=1,6,1 do
        simLegTips[i]=sim.getObject('./footTip'..i)
        simLegTargets[i]=sim.getObject('./footTarget'..i)
        initialPos[i]=sim.getObjectPosition(simLegTips[i],-1)
        initialPosition[i]=sim.getObjectPosition(simLegTips[i],baseJoint[i])
    end

    Lcoxa=math.sqrt((simJoint1Pos[1][1]-simJoint2Pos[1][1])^2+(simJoint1Pos[1][2]-simJoint2Pos[1][2])^2+(simJoint1Pos[1][3]-simJoint2Pos[1][3])^2)
    Lfemur=math.sqrt((simJoint2Pos[1][1]-simJoint3Pos[1][1])^2+(simJoint2Pos[1][2]-simJoint3Pos[1][2])^2+(simJoint2Pos[1][3]-simJoint3Pos[1][3])^2)
    Ltibia=math.sqrt((simJoint3Pos[1][1]-initialPos[1][1])^2+(simJoint3Pos[1][2]-initialPos[1][2])^2+(simJoint3Pos[1][3]-initialPos[1][3])^2)
    print("(Lcoxa,Lfemur,Ltibia)=",Lcoxa,Lfemur,Ltibia)

    cogPos={}
    cogPos = sim.getObjectPosition(antBase,-1)
    print("COG position is",cogPos)

    shoulderVector={}
    for i=1,6,1 do
        shoulderVector[i] = {simJoint1Pos[i][1] - cogPos[1],simJoint1Pos[i][2] - cogPos[2],simJoint1Pos[i][3] - cogPos[3]}
    end

    csvData={}
    local scenePath = sim.getStringParam(sim.stringparam_scene_path)
    local csvFilePath = scenePath..'/CutTree0.5Nm-grid-35-range-max-depth-4-Wall-140high.csv'  -- INPUT CSV
    inputCsvPath = csvFilePath

    local file = io.open(csvFilePath,'r')
    if file then
        for line in file:lines() do
            local values = {}
            for value in string.gmatch(line,'[^,]+')do
                table.insert(values,tonumber(value))
            end
            table.insert(csvData,values)
        end
        file:close()
    else
        sim.addLog(sim.verbosity_errors,"can not find csv file:"..csvFilePath)
    end

    Leg={}
    COG={}

    COM_ref = sim.getObject('/COM_ref')
    legRef = {}
    targetL = {}

    s = 0.001  -- mm -> m

    -- create leg position from CSV
    for leg=1,6,1 do
        Leg[leg]={}
        for j = 1,#csvData,1 do
            Leg[leg][j] = {s*csvData[j][16-3*(leg-1)], s*csvData[j][17-3*(leg-1)], s*csvData[j][18-3*(leg-1)]}
        end
    end

    -- create COM position from CSV
    for i=1,#csvData,1 do
        COG[i] = { s*csvData[i][19], s*csvData[i][20], s*csvData[i][21]} -- mm to m
    end

    -- create Dummy position by COM_ref
    for leg=1,6,1 do
        legRef[leg]  = sim.getObject(string.format('/COM_ref/legRef%d', leg))
        targetL[leg] = sim.getObject(string.format('/COM_ref/targetL%d', leg))
    end

    graph1 = sim.getObject('/Graph')

    joint1Torque = sim.addGraphStream(graph1,'joint 1 torque','Nm',0,{1,0,0})
    joint2Torque = sim.addGraphStream(graph1,'joint 2 torque','Nm',0,{0,1,0})
    joint3Torque = sim.addGraphStream(graph1,'joint 3 torque','Nm',0,{0,0,1})
    joint4Torque = sim.addGraphStream(graph1,'joint 4 torque','Nm',0,{1,1,0})
    joint5Torque = sim.addGraphStream(graph1,'joint 5 torque','Nm',0,{1,0,1})
    joint6Torque = sim.addGraphStream(graph1,'joint 6 torque','Nm',0,{0,1,1})

    -- legacy file2 (per-step) - default OFF
    if ENABLE_LEGACY_JOINT1_LOG then
        local legacyPath = scenePath .. '/jointTorque0206.txt'
        file2 = io.open(legacyPath,'w')
        file2:write('time, Leg1Joint, Leg2Joint, Leg3Joint, Leg4Joint, Leg5Joint, Leg6Joint\n\n')
    end

    -- ===== Keyframe CSV output file (name derived from input CSV) =====
    local stem = getPathStem(csvFilePath)
    local outPath = scenePath .. "/" .. stem .. "__coppelia_keyframe_log_v1_1.csv"
    logCsv = io.open(outPath, "w")
    if not logCsv then
        sim.addLog(sim.verbosity_errors, "can not open output csv: "..outPath)
    else
        logCsv:write(table.concat({
            "schema_version","step_idx","t_s","motion_type","success","status",
            "com_world_x_m","com_world_y_m","com_world_z_m",
            "base_world_qx","base_world_qy","base_world_qz","base_world_qw",
            "contact_leg0","contact_leg1","contact_leg2","contact_leg3","contact_leg4","contact_leg5",
            "tau_leg0_j1_Nm","tau_leg0_j2_Nm","tau_leg0_j3_Nm",
            "tau_leg1_j1_Nm","tau_leg1_j2_Nm","tau_leg1_j3_Nm",
            "tau_leg2_j1_Nm","tau_leg2_j2_Nm","tau_leg2_j3_Nm",
            "tau_leg3_j1_Nm","tau_leg3_j2_Nm","tau_leg3_j3_Nm",
            "tau_leg4_j1_Nm","tau_leg4_j2_Nm","tau_leg4_j3_Nm",
            "tau_leg5_j1_Nm","tau_leg5_j2_Nm","tau_leg5_j3_Nm",
            "max_abs_tau_all_Nm","sum_tau_all_sq_Nm2",
            "foot_robot_leg0_x_m","foot_robot_leg0_y_m","foot_robot_leg0_z_m",
            "foot_robot_leg1_x_m","foot_robot_leg1_y_m","foot_robot_leg1_z_m",
            "foot_robot_leg2_x_m","foot_robot_leg2_y_m","foot_robot_leg2_z_m",
            "foot_robot_leg3_x_m","foot_robot_leg3_y_m","foot_robot_leg3_z_m",
            "foot_robot_leg4_x_m","foot_robot_leg4_y_m","foot_robot_leg4_z_m",
            "foot_robot_leg5_x_m","foot_robot_leg5_y_m","foot_robot_leg5_z_m",
            "pose_hash"
        }, ",") .. "\n")
        logCsv:flush()
        print("[CSV] Keyframe log output:", outPath)
    end
end

function sysCall_sensing()

    -- graph streams (kept)
    sim.setGraphStreamValue(graph1,joint1Torque,sim.getJointForce(simJoint1[1]))
    sim.setGraphStreamValue(graph1,joint2Torque,sim.getJointForce(simJoint1[2]))
    sim.setGraphStreamValue(graph1,joint3Torque,sim.getJointForce(simJoint1[3]))
    sim.setGraphStreamValue(graph1,joint4Torque,sim.getJointForce(simJoint1[4]))
    sim.setGraphStreamValue(graph1,joint5Torque,sim.getJointForce(simJoint1[5]))
    sim.setGraphStreamValue(graph1,joint6Torque,sim.getJointForce(simJoint1[6]))

    -- legacy per-step log (OFF by default)
    if ENABLE_LEGACY_JOINT1_LOG and file2 then
        local leg1Torque=sim.getJointForce(simJoint1[1])
        local leg2Torque=sim.getJointForce(simJoint1[2])
        local leg3Torque=sim.getJointForce(simJoint1[3])
        local leg4Torque=sim.getJointForce(simJoint1[4])
        local leg5Torque=sim.getJointForce(simJoint1[5])
        local leg6Torque=sim.getJointForce(simJoint1[6])

        file2:write(string.format('%.2f, ',sim.getSimulationTime() + sim.getSimulationTimeStep()))
        file2:write(string.format('%.2f, ',leg1Torque))
        file2:write(string.format('%.2f, ',leg2Torque))
        file2:write(string.format('%.2f, ',leg3Torque))
        file2:write(string.format('%.2f, ',leg4Torque))
        file2:write(string.format('%.2f, ',leg5Torque))
        file2:write(string.format('%.2f\n',leg6Torque))
    end
end

function sysCall_cleanup()
    if logCsv then
        logCsv:close()
        logCsv = nil
    end
    if file2 then
        file2:close()
        file2 = nil
    end
end

function inverseKinematics(input)
    output={}
    xt=math.sqrt(input[1]^2+input[2]^2)-Lcoxa
    Lim=math.sqrt(xt^2+input[3]^2)
    cosa1=(Lfemur^2+Lim^2-Ltibia^2)/(2*Lfemur*Lim)
    cosa2=(Lfemur^2+Ltibia^2-Lim^2)/(2*Lfemur*Ltibia)
    output[1]=math.atan2(input[2],input[1])
    output[2]=math.atan2(xt,math.abs(input[3]))+math.acos(cosa1)
    output[3]=math.pi/2-math.acos(cosa2)
    return output
end

function angleDisplacement(leg,initialAngle,angle)
    disAngle={}
    disAngle[2]=initialAngle[2]-angle[2]
    disAngle[3]=angle[3]-initialAngle[3]
    if leg==6 then
        disAngle[1]=angle[1]-initialAngle[1]
    elseif leg==5 then
        disAngle[1]=angle[1]-initialAngle[1]
    elseif leg==4 then
        disAngle[1]=angle[1]-initialAngle[1]
    elseif leg==3 then
        if angle[1]<0 then
            disAngle[1]=angle[1]-initialAngle[1]
        else
            disAngle[1]=(angle[1]-initialAngle[1])-2*math.pi
        end
    elseif leg==2 then
        if angle[1]<0 then
            disAngle[1]=angle[1]-initialAngle[1]
        else
            disAngle[1]=(angle[1]-initialAngle[1])-2*math.pi
        end
    else
        if angle[1]<0 then
            disAngle[1]=2*math.pi+(angle[1]-initialAngle[1])
        else
            disAngle[1]=angle[1]-initialAngle[1]
        end
    end
    return disAngle
end

function setGripperOn(isOn,leg)
    -- log-only state (NO effect on simulation except logging)
    contact[leg] = isOn and 1 or 0

    if leg==1 then
        if isOn then
            sim.writeCustomDataBlock(LFsuction,'gripperOn','on')
        else
            sim.writeCustomDataBlock(LFsuction,'gripperOn','')
        end
    elseif leg==2 then
        if isOn then
            sim.writeCustomDataBlock(LMsuction,'gripperOn','on')
        else
            sim.writeCustomDataBlock(LMsuction,'gripperOn','')
        end
    elseif leg==3 then
        if isOn then
            sim.writeCustomDataBlock(LRsuction,'gripperOn','on')
        else
            sim.writeCustomDataBlock(LRsuction,'gripperOn','')
        end
    elseif leg==4 then
        if isOn then
            sim.writeCustomDataBlock(RRsuction,'gripperOn','on')
        else
            sim.writeCustomDataBlock(RRsuction,'gripperOn','')
        end
    elseif leg==5 then
        if isOn then
            sim.writeCustomDataBlock(RMsuction,'gripperOn','on')
        else
            sim.writeCustomDataBlock(RMsuction,'gripperOn','')
        end
    else
        if isOn then
            sim.writeCustomDataBlock(RFsuction,'gripperOn','on')
        else
            sim.writeCustomDataBlock(RFsuction,'gripperOn','')
        end
    end
end

---------------------------------------------------------------------------------------
local function applyIndexToDummies(idx)
    -- unit [m]
    sim.setObjectPosition(COM_ref, -1, COG[idx])

    -- unit [m]
    for leg=1,6 do
        sim.setObjectPosition(targetL[leg], legRef[leg], Leg[leg][idx])
    end
end

-- noize mergin - this number is in m
local DZ_EPS = 0.0005  -- 0.5mm

-- returns: toAttach{leg...}, toDetach{leg...}
local function classifySwitchByDz(idx)
    local toAttach = {}
    local toDetach = {}
    if idx <= 1 then return toAttach, toDetach end

    for leg=1,6,1 do
        local dz = Leg[leg][idx][3] - Leg[leg][idx-1][3]  -- z in leg coordinates [m]
        if dz > DZ_EPS then
            table.insert(toDetach, leg) -- OFF
        elseif dz < -DZ_EPS then
            table.insert(toAttach, leg) -- ON
        end
    end
    return toAttach, toDetach
end
---------------------------------------------------------------------------------------

-- wall contact plane (world): x = -0.060 [m]
local WALL_X = -0.060
local n = {-1,0,0}
local g = {0,0,-1}

local function dot(a,b) return a[1]*b[1]+a[2]*b[2]+a[3]*b[3] end
local function sub(a,b) return {a[1]-b[1], a[2]-b[2], a[3]-b[3]} end
local function cross(a,b)
    return { a[2]*b[3]-a[3]*b[2], a[3]*b[1]-a[1]*b[3], a[1]*b[2]-a[2]*b[1] }
end
local function norm(a) return math.sqrt(dot(a,a)) end
local function normalize(a)
    local na = norm(a)
    if na < 1e-9 then return {0,1,0} end
    return {a[1]/na, a[2]/na, a[3]/na}
end
local function sign(x) if x>0 then return 1 elseif x<0 then return -1 else return 0 end end

local function calcSiFromPos(pW, p0, u)
    local ri = sub(pW, p0)
    local s_i = dot(cross(ri, n), u)
    return s_i
end

local function planOrdersSi(toAttach, toDetach)
    local rCoM = sim.getObjectPosition(COM_ref, -1)
    local p0   = {WALL_X, rCoM[2], rCoM[3]}
    local u    = normalize(cross(n, g))

    local Mpeel = dot(u, cross(sub(rCoM, p0), g))
    local sgn = sign(Mpeel)
    if sgn == 0 then sgn = 1 end

    local attachData = {}
    for _,leg in ipairs(toAttach) do
        local pW = sim.getObjectPosition(targetL[leg], -1)
        local Si = calcSiFromPos(pW, p0, u)
        table.insert(attachData, {leg=leg, key = -sgn*Si})
    end
    table.sort(attachData, function(a,b) return a.key > b.key end)

    local detachData = {}
    for _,leg in ipairs(toDetach) do
        local pW = sim.getObjectPosition(simLegTips[leg], -1)
        local Si = calcSiFromPos(pW, p0, u)
        table.insert(detachData, {leg=leg, key = sgn*Si})
    end
    table.sort(detachData, function(a,b) return a.key > b.key end)

    local attachOrder, detachOrder = {}, {}
    for _,d in ipairs(attachData) do table.insert(attachOrder, d.leg) end
    for _,d in ipairs(detachData) do table.insert(detachOrder, d.leg) end
    return attachOrder, detachOrder
end

function sysCall_thread()

    -- ===== Make initial suction constraints without falling =====
    setAllShapesStatic(true)
    sim.wait(0.2)
    setAllShapesStatic(false)
    
    -- ===== timing knobs =====
    local DT = sim.getSimulationTimeStep()
    local WAIT_MOVE        = 0.60  -- wait time for leg motion (0.10 also works)
    local WAIT_SUCTION     = 0.20  -- wait time for suction ON/OFF to take effect (>= DT recommended)
    local WAIT_BETWEEN_LEG = 0.30  -- interval between switching each leg
    local WAIT_AFTER_PHASE = 1.00  -- wait after Attach phase or Detach phase
    local WAIT_SETTLE      = 3.00  -- stabilization wait after one index execution (~2s)

    ---------------------------------------------------------------

    firstAngle={}
    for i=1,6,1 do
        firstAngle[i]=inverseKinematics(initialPosition[i])
        print("leg i angle",firstAngle[i])
    end

    targetAngle={}
    q0 = {}

    sim.wait(5)

    for leg=1,6 do
        q0[leg] = {0,0,0}
        q0[leg][1] = sim.getJointPosition(simJoint1[leg])
        q0[leg][2] = sim.getJointPosition(simJoint2[leg])
        q0[leg][3] = sim.getJointPosition(simJoint3[leg])
    end

    for index=1,#csvData,1 do
        print('index =',index)

        applyIndexToDummies(index)

        local toAttach, toDetach = classifySwitchByDz(index)
        local attachOrder, detachOrder = planOrdersSi(toAttach, toDetach)

        if (#toAttach > 0) or (#toDetach > 0) then
            print(string.format("[SWITCH] idx=%d attach={%s} detach={%s}",
                index,
                table.concat(toAttach, ","),
                table.concat(toDetach, ",")
            ))
        end

        if (#toAttach==0) and (#toDetach==0) then
            for leg=1,6 do
                local pGoalL = sim.getObjectPosition(targetL[leg], BASEJOINT[leg])
                targetAngle[leg] = inverseKinematics(pGoalL)
                local dq = angleDisplacement(leg, firstAngle[leg], targetAngle[leg])
                sim.setJointTargetPosition(simJoint1[leg], q0[leg][1] + dq[1])
                sim.setJointTargetPosition(simJoint2[leg], q0[leg][2] + dq[2])
                sim.setJointTargetPosition(simJoint3[leg], q0[leg][3] + dq[3])
            end
            sim.wait(2)
        else
            -- Phase1: Attach
            for _,leg in ipairs(attachOrder) do
                local pGoalL = sim.getObjectPosition(targetL[leg], baseJoint[leg])
                targetAngle[leg] = inverseKinematics(pGoalL)
                local dq = angleDisplacement(leg, firstAngle[leg], targetAngle[leg])

                sim.setJointTargetPosition(simJoint1[leg], q0[leg][1] + dq[1])
                sim.setJointTargetPosition(simJoint2[leg], q0[leg][2] + dq[2])
                sim.setJointTargetPosition(simJoint3[leg], q0[leg][3] + dq[3])

                sim.wait(WAIT_MOVE)
                setGripperOn(true, leg)
                sim.wait(WAIT_SUCTION)
                sim.wait(WAIT_BETWEEN_LEG)
            end
            
            sim.wait(WAIT_AFTER_PHASE)

            -- Phase2: Detach
            for _,leg in ipairs(detachOrder) do
                setGripperOn(false, leg)
                sim.wait(WAIT_SUCTION)

                local pGoalL = sim.getObjectPosition(targetL[leg], baseJoint[leg])
                targetAngle[leg] = inverseKinematics(pGoalL)
                local dq = angleDisplacement(leg, firstAngle[leg], targetAngle[leg])

                sim.setJointTargetPosition(simJoint1[leg], q0[leg][1] + dq[1])
                sim.setJointTargetPosition(simJoint2[leg], q0[leg][2] + dq[2])
                sim.setJointTargetPosition(simJoint3[leg], q0[leg][3] + dq[3])

                sim.wait(WAIT_MOVE)
                sim.wait(WAIT_BETWEEN_LEG)
            end
            sim.wait(1.0)
        end

        -- settle
        sim.wait(WAIT_SETTLE)

        -- ===== Keyframe log (ONE ROW per index) =====
        local motion_type = ((#toAttach==0) and (#toDetach==0)) and "no_switch" or "switch"
        writeKeyframeRow(index - 1, motion_type, true, "ok")
    end
end
