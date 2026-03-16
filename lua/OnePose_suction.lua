sim=require'sim'

function sysCall_init() 
    self=sim.getObject('.')
    sensor=sim.getObject('./Proximity_sensor6')
    dummyA=sim.getObject('./LoopClosureDummy2')
    dummyB=sim.getObject('./RF_Dummy1')
    wall=sim.getObject('/wall1')
    --[[
    local r,d,pt,ha=sim.checkProximitySensor(sensor,wall)
    sim.setObjectParent(dummyB,ha,true)
    sim.setLinkDummy(dummyB,dummyA)
    ]]--
end

function sysCall_cleanup() 
    sim.setLinkDummy(dummyB,-1)
    sim.setObjectParent(dummyB,dummyA,true)
    sim.writeCustomDataBlock(self,'gripperOn','')
end 

function sysCall_actuation()
    local v=sim.readCustomDataBlock(self,'gripperOn')
    if v and v~='' then
        if sim.getObjectChild(dummyA,0)==dummyB then
            local r,d,pt,h=sim.checkProximitySensor(sensor,wall)
            if h and h>=0 then
                sim.setObjectParent(dummyB,h,true)
                sim.setLinkDummy(dummyB,dummyA)
            end
        end
    else
        if sim.getObjectChild(dummyA,0)~=dummyB then
            sim.setLinkDummy(dummyB,-1)
            sim.setObjectParent(dummyB,dummyA,true)
            sim.setObjectPose(dummyB,{0,0,0,0,0,0,1},dummyA)
        end
    end
end 
