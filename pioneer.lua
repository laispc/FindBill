    -- Ambient object 
function new_ambient(ambientType, ambientName)
    local this = {}
    this.type  = ambientType
    this.name  = ambientName
    this.north = nil
    this.south = nil
    this.east  = nil
    this.west  = nil
    this.path  = false
    this.visited = false
    return this
end
    -- Map object
function new_map(maptop, mission)
    -- Initialize parameters
    local this = {}
    this.rooms = {}
    this.from = nil
    this.actual_room = nil
    this.to = nil
    this.target = {}
    this.target.x = nil
    this.target.y = nil
    this.dir = 0
    this.foundPath = false
    
    -- Retrive the required room from rooms table.
    -- If the required room is not in the table, then create it
    -- and add to table.
    this.getRoomOrCreate = function(self, roomName)
        local room = self.rooms[roomName]
        
        if(room == null) then
            roomType = roomName:sub(1,1)
            if(roomType == 'C') then
                roomType = 'corredor'
            elseif(roomType == 'S') then
                roomType = 'sala'
            elseif(roomType == 'O') then
                roomType = 'outdoor'
            end
            room = new_ambient(roomType, roomName)
            self.rooms[roomName] = room
        end
        
        return room
    end
    
    -- Dijkstra to minimum path from the robot to bill.
    this.dijkstra = function(self, node, target)
        node.visited = true
        
        if(node.name == target) then
            node.path = true
            self.foundPath = true
            return
        end
        
        if(not self.foundPath 
                and node.north ~= nil
                and not node.north.visited) then
            self:dijkstra(node.north, target)
        end

        if(not self.foundPath 
                and node.south ~= nil
                and not node.south.visited) then
            self:dijkstra(node.south, target)
        end

        if(not self.foundPath 
                and node.east ~= nil
                and not node.east.visited) then
            self:dijkstra(node.east, target)
        end

        if(not self.foundPath 
                and node.west ~= nil
                and not node.west.visited) then
            self:dijkstra(node.west, target)
        end
        
        if(self.foundPath) then
            node.path = true
        end
    end
    
    -- Process the maptop.txt file.
    -- For each connection, get or create the two ambients from
    -- rooms table and connect them as needed.
    for line in io.lines(maptop) do
        local roomName = line:sub(1,2)
        local room = this:getRoomOrCreate(roomName)
        
        local childName = line:sub(5,6)
        local child = this:getRoomOrCreate(childName)
        
        dir = line:sub(8,8)
        if(dir == 'N') then
            room.north = child
        elseif(dir == 'S') then
            room.south = child
        elseif(dir == 'L') then
            room.east = child
        elseif(dir == 'O') then
            room.west = child
        end
    end
    
    -- Process the missao.txt file.
    l={}
    i = 1;
    for line in io.lines(mission) do
        table.insert(l, line:sub(1,-2))
    end
    this.from = this.rooms[l[1]]
    this.actual_room = this.from -- I am where I start
    this.to = this.rooms[l[2]]
    this.target    = {}
    this.target.x = l[3]:sub(1,l[3]:find(' ')-1):gsub('%.',',')
    this.target.x = tonumber(this.target.x)
    this.target.y = l[3]:sub(l[3]:find(' ')+1):gsub('%.',',')
    --this.target.y = tonumber(this.target.y)
    
    -- Call the dijkstra algorithm
    this.foundPath = false
    this:dijkstra(this.from, this.to.name)
    
    for k,v in pairs(this.rooms) do
        v.visited = false
    end
    
    -- Log the path found with dijkstra.
    -- If use this, clean the visited parameters again.
    -- These are used during robot navigation.
    this.writePath = function(self, from, to)
        from.visited = true
        if(from.name == to) then return end
        if(from.north ~= nil and from.north.path 
                and not from.north.visited) then
            self:writePath(from.north, to)
        elseif(from.south ~= nil and from.south.path 
                and not from.south.visited) then
            self:writePath(from.south, to)
        elseif(from.east ~= nil and from.east.path 
                and not from.east.visited) then
            self:writePath(from.east, to)
        elseif(from.west ~= nil and from.west.path 
                and not from.west.visited) then
            self:writePath(from.west, to)
        end
    end
    
    -- Uncomment this to log the path.
    -- this:writePath(this.from, this.to.name)
    
    -- Calculates the reference angle
    this.findDir = function (self)
        if(self.actual_room.north ~= nil 
                and not self.actual_room.north.visited) then
            self.dir = 270
        elseif(self.actual_room.south ~= nil 
                and not self.actual_room.south.visited) then
            self.dir = 90
        elseif(self.actual_room.east ~= nil 
                and not self.actual_room.east.visited) then
            self.dir = 180
        elseif(self.actual_room.west ~= nil 
                and not self.actual_room.west.visited) then
            self.dir = 0
        end
    end
    
    this.actual_room.visited = true
    this:findDir()

    this.nextRoom = function(self)
        if(self.actual_room.north ~= nil 
                and self.actual_room.north.path 
                and not self.actual_room.north.visited) then
            self.actual_room = self.actual_room.north
            self.actual_room.visited = true
        elseif(self.actual_room.south ~= nil 
                and self.actual_room.south.path 
                and not self.actual_room.south.visited) then
            self.actual_room = self.actual_room.south
            self.actual_room.visited = true
        elseif(self.actual_room.east ~= nil 
                and self.actual_room.east.path 
                and not self.actual_room.east.visited) then
            self.actual_room = self.actual_room.east
            self.actual_room.visited = true
        elseif(self.actual_room.west ~= nil 
                and self.actual_room.west.path 
                and not self.actual_room.west.visited) then
            self.actual_room = self.actual_room.west
            self.actual_room.visited = true
        end
        self:findDir()
    end

    this.getNextRoom = function(self)
        if(self.actual_room.north ~= nil 
                and self.actual_room.north.path 
                and not self.actual_room.north.visited) then
            return self.actual_room.north
        elseif(self.actual_room.south ~= nil 
                and self.actual_room.south.path 
                and not self.actual_room.south.visited) then
            return self.actual_room.south
        elseif(self.actual_room.east ~= nil 
                and self.actual_room.east.path 
                and not self.actual_room.east.visited) then
            return self.actual_room.east
        elseif(self.actual_room.west ~= nil 
                and self.actual_room.west.path 
                and not self.actual_room.west.visited) then
            return self.actual_room.west
        end
    end
    
    return this
end

function new_door(b, bIndex, e, eIndex)
    local door = {}
    door.b = b
    door.bIndex = bIndex
    door.e = e
    door.eIndex = eIndex
    door.center = {
            x = 0.5 * (b.x + e.x),
            y = 0.5 * (b.y + e.y)
        }

    return door
end

function readLaser()
    local data = simGetStringSignal("measuredDataAtThisTime")
    if data ~= nil then
        data = simUnpackFloats(data)
        laser = {}
        for i=1,#data-12,3 do
            table.insert(laser, {x = data[i], y = data[i+1]})
        end
    end
end

-- Bussola algorithm released by fosorio.
function bussola() 
    piohand=simGetObjectHandle("Pioneer_p3dx")
    AngEuler=simGetObjectOrientation(piohand,-1)
    AngDiscr= math.floor((AngEuler[3]*180)/3.1415926535)
    if (AngDiscr < 0) then AngDiscr=(180-math.abs(AngDiscr))+180 end
    return AngDiscr
end

function robotVector()
    local r0 = {x = 0.0, y = 0.0}
    for i=2,#gps do
        r0.x = r0.x + gps[i].x
        r0.y = r0.y + gps[i].y
    end
    r0.x = r0.x / (#gps - 1)
    r0.y = r0.y / (#gps - 1)

    return {x = gps[1].x - r0.x, -- gps[10].x,
            y = gps[1].y - r0.y} -- gps[10].y}
end

function billVector()
    local r0 = {x = 0.0, y = 0.0}
    for i=2,#gps do
        r0.x = r0.x + gps[i].x
        r0.y = r0.y + gps[i].y
    end
    r0.x = r0.x / (#gps - 1)
    r0.y = r0.y / (#gps - 1)

    return {x = map.target.x - r0.x, -- gps[10].x,
            y = map.target.y - r0.y} -- gps[10].y}
end

function readGps()
    local data = simGetStringSignal('gps')
    if data ~= nil then
        data = simUnpackFloats(data)

        for i=2,#gps do
            gps[i] = gps[i-1]
        end
        gps[1] = {x = data[1], y = data[2]}
    end
end

function radius(point)
    return math.sqrt(point.x * point.x + point.y * point.y)
end

function distance(from, to)
    return radius({x = to.x - from.x, y = to.y - from.y})
end

function slope(from, to)
    return (to.y - from.y)/(to.x - from.x)
end

function findDoors()
    found_door = false
    local d = {}
    local todraw = {}
    local i = #laser - 2

    robotdir = bussola()
    if math.abs(robotdir - map.dir) > 180 then
        robotdir = robotdir - 360
    end
    angleIndex = #laser/2 - (map.dir - robotdir) * 2 * math.pi / 180

    if angleIndex > #laser then
        angleIndex = #laser
    elseif angleIndex < 1 then
        angleIndex = 1
    end

    while i > 3 do
    -- for i=#laser-2,2,-1 do
        if distance(laser[i], laser[i-1]) > threshold.doorSize then
            local min = {}
            for j=i-1,3,-1 do
                table.insert(min,
                    {dist = distance(laser[i], laser[j]), index = j})
            end
            if #min > 0 then
                table.sort(min, function(a, b)
                    return a.dist < b.dist
                end)
                if distance(laser[i], laser[min[1].index]) < threshold.maxDoorSize then
                    table.insert(d, new_door(laser[i], i,
                                 laser[min[1].index], min[1].index))
                    i = min[1].index
                end
            end
        end
        i = i - 1
    end

    if #d > 0 then
        table.sort(d, function(a, b)
            return radius(a.center) < radius(b.center)
        end)
        found_door = true
        doors = d
    end

    if #doors > 0 then
        table.insert(todraw,doors[1].b.x)
        table.insert(todraw,doors[1].b.y)
        table.insert(todraw,0)

        table.insert(todraw,doors[1].center.x)
        table.insert(todraw,doors[1].center.y)
        table.insert(todraw,0)

        table.insert(todraw,doors[1].e.x)
        table.insert(todraw,doors[1].e.y)
        table.insert(todraw,0)
    end

    table.insert(todraw,5)
    table.insert(todraw,-1)
    table.insert(todraw,0)

    table.insert(todraw,-5)
    table.insert(todraw,-1)
    table.insert(todraw,0)

    table.insert(todraw,-5)
    table.insert(todraw,6)
    table.insert(todraw,0)

    table.insert(todraw,5)
    table.insert(todraw,6)
    table.insert(todraw,0)

    local data=simPackFloats(todraw)
    simSetStringSignal("doors",data)
end

function goToDoorControl(v, wheel)
    return v * math.exp(wheel * (k.doordistance * -doors[1].center.x
        - k.doorslope * slope(doors[1].b, doors[1].e) / threshold.doorslope))
end

function goToDoor()
    if not found_door and map.actual_room.type == 'outdoor' then
        map:nextRoom()
        state = FIND_DIRECTION
    elseif radius(doors[1].center) < threshold.doorproximity then
        state = PASS_DOOR
    end

    if #doors > 0 then
        vLeft = goToDoorControl(vLeft, leftWheel)
        vRight = goToDoorControl(vRight, rightWheel)
    end
end

function isOnRightDirection()
    return (map.dir == 0 and ((bussola() > 355 and bussola() <= 359) 
                or (bussola() >= 0 and bussola() < 5 )))
                or (map.dir == 90  and bussola() > 85  and bussola() < 95)
                or (map.dir == 270 and bussola() > 265 and bussola() < 275)
                or (map.dir == 180 and bussola() > 175 and bussola() < 185)
end

function findDirectionControl(v, wheel)
    robotdir = bussola()
    if math.abs(robotdir - map.dir) > 180 then
        robotdir = robotdir - 360;
    end
    return v * math.exp(wheel * k.direction * (map.dir - robotdir))
end

function findDirection()
    if isOnRightDirection() then
        state = GO_TO_DOOR
    end

    vLeft = findDirectionControl(vLeft, leftWheel)
    vRight = findDirectionControl(vRight, rightWheel)
end

function passDoorControl(v, wheel)
    return v * math.exp(wheel * k.doordistance * -doors[1].center.x)
    -- return v * (0.8 * math.exp(wheel * k.passDoor
    --     * (radius(doors[1].b) - radius(doors[1].e))) + 0.2)
end

function passDoor()
    if radius(doors[1].center) > 1.25 then
        map:nextRoom()
        state = FIND_DIRECTION
    end

    vLeft = passDoorControl(vLeft, leftWheel)
    vRight = passDoorControl(vRight, rightWheel)
end

function checkSideDoors()
    sideDoors[-4] = sideDoors[-3]
    sideDoors[-3] = sideDoors[-2]
    sideDoors[-2] = sideDoors[-1]
    sideDoors[-1] = sideDoors[0]
    sideDoors[0] =  math.abs(doors[1].center.y)
        < math.abs(doors[1].center.x)
        and math.abs(doors[1].center.y) < 1.5
    return sideDoors[0]
        and not sideDoors[-1]
        and not sideDoors[-2]
        and not sideDoors[-3]
        and not sideDoors[-4]
end

function walkOnCorridor()
    if checkSideDoors() then
        map:nextRoom()
        state = FIND_DIRECTION
    end
end

function goToBillControl(v, wheel)
    -- log:write("distance: ", distance(gps(), map.target), "\n")
    -- z is the k portion of vetorial product
    local a = billVector()
    local b = robotVector()
    local z = a.x*b.y - a.y*b.x

    local todraw = {}
    table.insert(todraw,a.x)
    table.insert(todraw,a.y)
    table.insert(todraw,0)

    table.insert(todraw,b.x)
    table.insert(todraw,b.y)
    table.insert(todraw,0)

    table.insert(todraw,0)
    table.insert(todraw,z)
    table.insert(todraw,0)    

    table.insert(todraw,5)
    table.insert(todraw,-1)
    table.insert(todraw,0)

    table.insert(todraw,-5)
    table.insert(todraw,-1)
    table.insert(todraw,0)

    table.insert(todraw,-5)
    table.insert(todraw,6)
    table.insert(todraw,0)

    table.insert(todraw,5)
    table.insert(todraw,6)
    table.insert(todraw,0)

    local data=simPackFloats(todraw)
    simSetStringSignal("doors",data)
    
    return v * math.exp(wheel * k.bill * -z)-- (1 - distance(gps(), map.target)))
end

function goToBill()
    if distance(gps[1], map.target) < 2 then
        state = STOP
    end

    vLeft = goToBillControl(vLeft, leftWheel)
    vRight = goToBillControl(vRight, rightWheel)
end

function roomStateMachine()
    if state == FIND_DIRECTION then
        findDirection()
    elseif state == GO_TO_DOOR then
        goToDoor()
    elseif state == PASS_DOOR then
        passDoor()
    end
end

function corridorStateMachine()
    if state == FIND_DIRECTION then
        findDirection()
    elseif state == GO_TO_DOOR then
        if map:getNextRoom().type == 'corredor' then
            walkOnCorridor()
        else
            goToDoor()
        end
    elseif state == PASS_DOOR then
        passDoor()
    end
end

function outdoorStateMachine()
    if state == FIND_DIRECTION then
        findDirection()
    elseif state == GO_TO_DOOR then
        goToDoor()
    elseif state == PASS_DOOR then
        passDoor()
    end
end

if (sim_call_type==sim_childscriptcall_initialization) then 
    motorLeft=simGetObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight=simGetObjectHandle("Pioneer_p3dx_rightMotor")
    log = io.open('pioneer.log', 'w')

    map = new_map(
        '/home/ddiorio/Documents/USP20151/robosmoveis/FindBill/maptop.txt',
        '/home/ddiorio/Documents/USP20151/robosmoveis/FindBill/missao.txt')

    v0=2

    leftWheel = -1
    rightWheel = 1

    laser = {}
    doors = {}
    found_door = false

    distanceToDoor = {}
    distanceToDoor[ 0] = 100
    distanceToDoor[-1] = 100
    distanceToDoor[-2] = 100
    distanceToDoor[-3] = 100
    distanceToDoor[-4] = 100

    sideDoors = {}
    sideDoors[ 0] = false
    sideDoors[-1] = false
    sideDoors[-2] = false
    sideDoors[-3] = false
    sideDoors[-4] = false

    gps = {}
    gps[1] = {x = 0.0, y = 0.0}
    gps[2] = {x = 0.0, y = 0.0}
    gps[3] = {x = 0.0, y = 0.0}
    gps[4] = {x = 0.0, y = 0.0}
    gps[5] = {x = 0.0, y = 0.0}
    gps[6] = {x = 0.0, y = 0.0}
    gps[7] = {x = 0.0, y = 0.0}
    gps[8] = {x = 0.0, y = 0.0}
    gps[9] = {x = 0.0, y = 0.0}
    gps[10] = {x = 0.0, y = 0.0}

    k = {}
    k.doordistance = 0.7
    k.direction = 0.007
    k.passDoor = 0.08
    k.walkOnCorridor = 0.08
    k.doorslope = 0.02
    k.bill = 1.3

    threshold = {}
    threshold.doorSize = 0.85
    threshold.maxDoorSize = 1.15
    threshold.doorslope = 0.1
    threshold.doorproximity = 0.7

    STOP = -1
    FIND_DIRECTION = 0
    GO_TO_DOOR = 1
    PASS_DOOR = 2
    state = FIND_DIRECTION
end 

if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end 

if (sim_call_type==sim_childscriptcall_actuation) then
    if map.actual_room.type == 'outdoor' then
        threshold.doorSize = 6.35
        threshold.maxDoorSize = 6.45
    else
        threshold.doorSize = 0.85
        threshold.maxDoorSize = 1.15
    end

    readGps()
    readLaser()
    findDoors()

    vLeft=v0
    vRight=v0


    if state == STOP then
        vLeft = 0
        vRight = 0
    elseif map.actual_room.name == map.to.name then
        goToBill()
    elseif map.actual_room.type == 'sala' then
        roomStateMachine()
    elseif map.actual_room.type == 'corredor' then
        corridorStateMachine()
    elseif map.actual_room.type == 'outdoor' then
        outdoorStateMachine()
    end
    
    simSetJointTargetVelocity(motorLeft,vLeft)
    simSetJointTargetVelocity(motorRight,vRight)

    r = gps[1]
    log:write(string.format("%.3f %.3f\n", r.x, r.y))
end 
