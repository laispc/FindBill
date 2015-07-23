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
    this.target.x = tonumber(l[3]:sub(1,l[3]:find(' ')-1))
    this.target.y = tonumber(l[3]:sub(l[3]:find(' ')+1))
    
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
        log:write(from.name)
        if(from.name == to) then return end
        if(from.north ~= nil and from.north.path 
                and not from.north.visited) then
            log:write("->N->")
            self:writePath(from.north, to)
        elseif(from.south ~= nil and from.south.path 
                and not from.south.visited) then
            log:write("->S->")
            self:writePath(from.south, to)
        elseif(from.east ~= nil and from.east.path 
                and not from.east.visited) then
            log:write("->L->")
            self:writePath(from.east, to)
        elseif(from.west ~= nil and from.west.path 
                and not from.west.visited) then
            log:write("->O->")
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
        log:write("Now on room ", self.actual_room.name, "\n")
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
        -- log:write("-----------------\n")
        for i=1,#data-12,3 do
            table.insert(laser, {x = data[i], y = data[i+1]})
            -- log:write(string.format("{%.4f;%.4f}\n", data[i], data[i+1]))
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
    local d = {}
    local todraw = {}
    for i=#laser-2,2,-1 do
        if distance(laser[i], laser[i-1]) > threshold.doorSize then
            local min = {}
            for j=i-1,1,-1 do
                table.insert(min,
                    {dist = distance(laser[i], laser[j]), index = j})
            end
            if #min > 0 then
                table.sort(min, function(a, b)
                    return a.dist < b.dist
                end)
                if distance(laser[i], laser[min[1].index]) < 1.2 then
                    table.insert(d, new_door(laser[i], i,
                                 laser[min[1].index], min[1].index))
                    i = min.index
                end
            end
        end
    end

    if #d > 0 then
        table.sort(d, function(a, b)
            return radius(a.center) < radius(b.center)
        end)
        doors = d
    end
    --[[
    log:write("----------\n")
    for k,v in ipairs(doors) do
        log:write("{",v.b.x,";",v.b.y,"}", 
                  "{", v.center.x, ";", v.center.y, "}", 
                  "{",v.e.x,";",v.e.y,"}\n")

        table.insert(todraw, v.b.x)
        table.insert(todraw, v.b.y)
        table.insert(todraw, 0)

        table.insert(todraw, v.center.x)
        table.insert(todraw, v.center.y)
        table.insert(todraw, 0)

        table.insert(todraw, v.e.x)
        table.insert(todraw, v.e.y)
        table.insert(todraw, 0)
    end]]

    table.insert(todraw,doors[1].b.x)
    table.insert(todraw,doors[1].b.y)
    table.insert(todraw,0)

    table.insert(todraw,doors[1].center.x)
    table.insert(todraw,doors[1].center.y)
    table.insert(todraw,0)

    table.insert(todraw,doors[1].e.x)
    table.insert(todraw,doors[1].e.y)
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
end

function goToDoorControl(v, wheel)
    return v * math.exp(wheel * k.doordistance * -doors[1].center.x)
end

function goToDoor()
    if radius(doors[1].center) < 1 then
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
    return v * math.exp(wheel * k.direction * (map.dir - bussola()))
end

function findDirection()
    if isOnRightDirection() then
        state = GO_TO_DOOR
    end

    vLeft = findDirectionControl(vLeft, leftWheel)
    vRight = findDirectionControl(vRight, rightWheel)
end

function passDoorControl(v, wheel)
    return v * (0.8 * math.exp(wheel * k.passDoor
        * (radius(doors[1].b) - radius(doors[1].e))) + 0.2)
end

function passDoor()
    if radius(doors[1].center) > 1.25 then
        map:nextRoom()
        state = FIND_DIRECTION
    end

    vLeft = passDoorControl(vLeft, leftWheel)
    vRight = passDoorControl(vRight, rightWheel)
end

function walkOnCorridorControl(v, wheel)
    return v * math.exp(wheel * k.walkOnCorridor
        * (radius(laser[1]) - radius(laser[#laser])))
end

function walkOnCorridor()
    log:write("{", doors[1].center.x, ";", doors[1].center.y, "}\n")
    if math.abs(doors[1].center.y) < math.abs(doors[1].center.x)
            and math.abs(doors[1].center.y) < 1 then
        map:nextRoom()
        state = FIND_DIRECTION
    end

    -- vLeft = walkOnCorridorControl(vLeft, leftWheel)
    -- vRight = walkOnCorridorControl(vRight, rightWheel)
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
    -- body
end

if (sim_call_type==sim_childscriptcall_initialization) then 
    motorLeft=simGetObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight=simGetObjectHandle("Pioneer_p3dx_rightMotor")
    log = io.open('pioneer.log', 'w')

    map = new_map(
        '/home/ddiorio/Documents/USP20151/robosmoveis/TrabalhoFinal/maptop.txt',
        '/home/ddiorio/Documents/USP20151/robosmoveis/TrabalhoFinal/missao.txt')

    v0=2

    leftWheel = -1
    rightWheel = 1

    laser = {}
    doors = {}

    k = {}
    k.doordistance = 0.7
    k.direction = 0.007
    k.passDoor = 0.08
    k.walkOnCorridor = 0.08

    threshold = {}
    threshold.doorSize = 0.85

    STOP = -1
    FIND_DIRECTION = 0
    GO_TO_DOOR = 1
    PASS_DOOR = 2
    state = FIND_DIRECTION
end 

if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end 

if (sim_call_type==sim_childscriptcall_actuation) then
    log:write("----------\n")
    readLaser()
    findDoors()

    vLeft=v0
    vRight=v0

    if state == STOP then
        vLeft = 0
        vRight = 0
    elseif map.actual_room.type == 'sala' then
        roomStateMachine()
    elseif map.actual_room.type == 'corredor' then
        corridorStateMachine()
    elseif map.actual_room.type == 'outdoor' then
        outdoorStateMachine()
    end
    
    simSetJointTargetVelocity(motorLeft,vLeft)
    simSetJointTargetVelocity(motorRight,vRight)
end 
