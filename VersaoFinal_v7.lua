	-- Ambient object 
function new_ambient(ambientType, ambientName)
	local this = {}
	this.type  = ambientType
	this.name  = ambientName
	this.north = nil
	this.south = nil
	this.east  = nil
	this.west  = nil
	this.here  = false
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
	this.from.here = true
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
		if(from.north ~= nil and from.north.path and not from.north.visited) then
			log:write("->N->")
			self:writePath(from.north, to)
		elseif(from.south ~= nil and from.south.path and not from.south.visited) then
			log:write("->S->")
			self:writePath(from.south, to)
		elseif(from.east ~= nil and from.east.path and not from.east.visited) then
			log:write("->L->")
			self:writePath(from.east, to)
		elseif(from.west ~= nil and from.west.path and not from.west.visited) then
			log:write("->O->")
			self:writePath(from.west, to)
		end
	end
	
	-- Uncomment this to log the path.
	-- this:writePath(this.from, this.to.name)
	
	-- Calculates the reference angle
	this.findDir = function (self)
		if(self.actual_room.north ~= nil and not self.actual_room.north.visited) then
			self.dir = 270
		elseif(self.actual_room.south ~= nil and not self.actual_room.south.visited) then
		 	self.dir = 90
		elseif(self.actual_room.east ~= nil and not self.actual_room.east.visited) then
			self.dir = 180
		elseif(self.actual_room.west ~= nil and not self.actual_room.west.visited) then
			self.dir = 0
		end
	end
	
	this.actual_room.visited = true
	this:findDir()

	this.nextRoom = function(self)
		self.actual_room.here = false
		if(self.actual_room.north ~= nil and not self.actual_room.north.visited) then
			self.actual_room = self.actual_room.north
			self.actual_room.north.here = true
			self.actual_room.north.visited = true
		elseif(self.actual_room.south ~= nil and not self.actual_room.south.visited) then
		 	self.actual_room = self.actual_room.south
		 	self.actual_room.south.here = true
		 	self.actual_room.south.visited = true
		elseif(self.actual_room.east ~= nil and not self.actual_room.east.visited) then
			self.actual_room = self.actual_room.east
			self.actual_room.east.here = true
			self.actual_room.east.visited = true
		elseif(self.actual_room.west ~= nil and not self.actual_room.west.visited) then
			self.actual_room = self.actual_room.west
			self.actual_room.west.here = true
			self.actual_room.west.visited = true
		end
		self:findDir()
	end
	
	return this
end

-- Bussola algorithm released by fosorio.
function bussola() 
	piohand=simGetObjectHandle("Pioneer_p3dx")
    AngEuler=simGetObjectOrientation(piohand,-1)
    AngDiscr= math.floor((AngEuler[3]*180)/3.1415926535) 
    if (AngDiscr < 0) then AngDiscr=(180-math.abs(AngDiscr))+180 end
    -- simSetIntegerSignal("Bussola",AngDiscr)
	return AngDiscr
end

-- Clean all reference values to default
function cleanRefs()
	ref['door'] = {0,0}
	ref[ '90']  = 10
	ref[ '45']  = 10
	ref[  '0']  = 10
	ref['-45']  = 10
	ref['-90']  = 10
	ref['critRight'] = 0
	ref['critLeft' ] = 0
	ref['minDist'  ] = 0
	ref['bussola'  ] = 0
end

-- Read data from hokuio
function readHokuio()
	local data = simTubeRead(hokuioTube)
	local temp = {}
	if (data) then
	    temp = simUnpackFloats(data)
	end

	hokuio = {}
	for i=1,#temp,3 do
		if(--temp[i] ~= nil and temp[i+1] ~= nil
			--and radius(temp[i], temp[i+1]) > threshold['avoidColision.critProximity']/3
			temp[i] ~= 0 and temp[i+1] ~= 0) then
			table.insert(hokuio, temp[  i])
			table.insert(hokuio, temp[i+1])
			table.insert(hokuio, temp[i+2])
		end
	end
	
end

function new_door()
	this = {}
	this.bound1 = {0,0}
	this.bound2 = {0,0}

	this.getCenter = function(self)
		--local center = {}
		--[[table.insert(
			center,
			self.bound1[1] + 0.5 * (self.bound2[1] - self.bound1[1]))
		table.insert(
			center,
			self.bound1[2] + 0.5 * (self.bound2[2] - self.bound1[2]) - 2.5)]]
		return self.bound1[2] + 0.5 * (self.bound2[2] - self.bound1[2]) --- 2.5--center
	end

	return this
end

-- I will change this... But it is to find doors.
function findDoor()
	local doors = {}
	local last = 0
	for i=1,#hokuio-5,3 do
		if(math.abs(hokuio[i+3] - hokuio[i]) > threshold['dLaser']) then
			if(last == 0) then
				if(hokuio[i+3] > hokuio[i]) then
					table.insert(doors, {hokuio[i], hokuio[i+1] + 0.5})
				else
					table.insert(doors, {hokuio[i+3], hokuio[i+4] - 0.5})
				end
			elseif(last == 1) then

			elseif(last == -1) then

			end


			if(hokuio[i+3] > hokuio[i]) then
				table.insert(doors, {hokuio[i], hokuio[i+1] + 0.5})
			else
				table.insert(doors, {hokuio[i+3], hokuio[i+4] - 0.5})
			end
		end
	end
	ref['door'] = max_x(doors)
end


function min_y(doors)
	if(#doors == 0) then return {0,0} end
	local door = doors[1]
	for i=2,#doors,1 do
		if(doors[i][2] < door[2]) then
			door = doors[i]
		end
	end
	return door
end

function max_y(doors)
	if(#doors == 0) then return new_door() end
	local door = doors[1]
	for i=2,#doors,1 do
		if(doors[i]:getCenter()--[[2]] > door:getCenter()--[[2]]) then
			door = doors[i]
		end
	end
	return door
end

function max_x(doors)
	if(#doors == 0) then return {0,0} end
	local door = doors[1]
	for i=2,#doors,1 do
		if(doors[i][1] > door[1]) then
			door = doors[i]
		end
	end
	return door
end

-- Recognize all doors hokuio can reach
-- and select the right door.
--[[function findDoor()
	local next_i = 0
	local r1 = 0
	local r2 = 0
	local min = 0
	local doors = {}
	local door = nil

	-- Recognize all doors hokuio can reach
	for i=1,#hokuio-3,3 do
		r1 = radius(hokuio[  i], hokuio[i+1])
		r2 = radius(hokuio[i+3], hokuio[i+4])
		--log:write(math.abs(r1 - r2), " ", threshold['door'], "\n")
		-- If the distance between two points is large enough to pioneer pass.
		if(door == nil and math.abs(r1 - r2) > threshold['door']) then
			door = new_door()
			door.bound1 = {hokuio[  i], hokuio[i+1]}
			door.bound2 = {hokuio[i+3], hokuio[i+4]}
			min = math.abs(r1 - r2)

			-- Find the next point closest to bound1
			for j=i+6,#hokuio-3,3 do
				r2 = radius(hokuio[j], hokuio[j+1])
				if(math.abs(r1 - r2) < min) then
					next_i = j+3
					min = math.abs(r1 - r2)
					door.bound2 = {hokuio[  j],hokuio[j+1]}
				end
			end

			-- Put the found door in doors collection.
			log:write("door found!")
			table.insert(doors, door)
			door = nil
			i = next_i
		end
	end



	-- Find the desired door
	if(state == ALIGN_TO_DOOR) then
		door = max_x(doors)
	else
		door = new_door()
	end

	-- Update the reference table
	ref['door'] = door
end]]


function findAngles()
	local data = simTubeRead(anglesTube)
	local temp = {}
	if (data) then
	    temp = simUnpackFloats(data)
	end
	ref[ '90'] = temp[1]
	ref[ '45'] = temp[2]
	ref[  '0'] = temp[3]
	ref['-45'] = temp[4]
	ref['-90'] = temp[5]
	ref['bussola'] = bussola()
	if(ref['90'] == nil) then
			ref['90'] = 10
		end
	 
		if(ref['45'] == nil) then
			ref['45'] = 10
		end
	 
		if(ref['0'] == nil) then
			ref['0'] = 10
		end
	 
		if(ref['-45'] == nil) then
			ref['-45'] = 10
		end
	 
		if(ref['-90'] == nil) then
			ref['-90'] = 10
		end
	 
		log:write(tostring(ref['90']), "\n",tostring(ref['45']), "\n",tostring(ref['0']), "\n",tostring(ref['-45']), "\n",tostring(ref['-90']), "\n")
	end


	-- Find the distance measured by the laser beams of
	-- 60º, 30º, 0º(not working), -30º, -60º. 
--[[function findAngles()
	ref['bussola'] = bussola()

	local found = {}
	found[ '90']   = 10
	found[ '60']   = false
	found[ '30']   = false
	found[  '0']   = 10
	found['-30']   = false
	found['-60']   = false
	found['-90']   = 10

	for i=1,#hokuio,3 do
		if(hokuio[i+1] < 0) then
			if(math.abs(hokuio[i]) < found['-90']) then
				found['-90'] = math.abs(hokuio[i])
				ref['-90'] = radius(hokuio[i], hokuio[i+1])
			end
		else
			if(math.abs(hokuio[i]) < found['90']) then
				found['90'] = math.abs(hokuio[i])
				ref['90'] = radius(hokuio[i], hokuio[i+1])
			end
		end

		if(math.abs(hokuio[i+1]) < found['0']) then
			found['0'] = math.abs(hokuio[i+1])
			ref['0'] = radius(hokuio[i], hokuio[i+1])
		end

		if(hokuio[i] > 0 and
		   hokuio[i+1] > 0) then
			-- Check if the actual laser beam has approximately
			-- the desired angle tangent. The found table avoid
			-- keep trying to find an angle already found.
			if(not found['60']) then
				if(math.floor(100*hokuio[i]/hokuio[i+1]) == -57 ) then
					ref['60'] = radius(hokuio[i], hokuio[i+1])
					found['60'] = true
				end
			end
			if(not found['30']) then
				if(math.floor(10*hokuio[i]/hokuio[i+1]) == -17 ) then
					ref['30'] = radius(hokuio[i], hokuio[i+1])
					found['30'] = true
				end
			end
			if(not found['-30']) then
				if(math.floor(10*hokuio[i]/hokuio[i+1]) == 17 ) then
					ref['-30'] = radius(hokuio[i], hokuio[i+1])
					found['-30'] = true
				end
			end
			if(not found['-60']) then
				if(math.floor(100*hokuio[i]/hokuio[i+1]) == 57 ) then
					ref['-60'] = radius(hokuio[i], hokuio[i+1])
					found['-60'] = true
				end
			end
		   
		end
	end
end]]

function findMinDist()
	local r = 0
	local halfHokuio = math.floor(#hokuio/2)
	local minLeft  = 0
	local minRight = 0
	if(#hokuio > 0) then
		minLeft = 1
		ref['minDist.left'] = radius(hokuio[1], hokuio[2])
	else
		ref['minDist'] = 10
		ref['minDist.left'] = 10
		ref['minDist.right'] = 10
		return
	end

	for i=1,halfHokuio,3 do
		r = radius(hokuio[i], hokuio[i+1])
		if(r < ref['minDist.left']) then
			minLeft = i
			ref['minDist.left'] = r
		end
		if(r < ref['minDist'] and hokuio[i] > 0) then
			ref['minDist'] = r
		end
	end

	minRight = i
	ref['minDist.right'] = radius(hokuio[i], hokuio[i+1])

	for i=i,#hokuio,3 do
		r = radius(hokuio[i], hokuio[i+1])
		if(r < ref['minDist.right']) then
			minRight = i
			ref['minDist.right'] = r
		end
		if(r < ref['minDist'] and hokuio[i] > 0) then
			ref['minDist'] = r
		end
	end

	if(ref['minDist.left'] < threshold['avoidColision.critProximity']) then
		ref['critLeft' ] = 1
	elseif(ref['minDist.right'] < threshold['avoidColision.critProximity']) then
		ref['critRight'] = 1
	end
	
	if(ref['minDist.left'] < ref['minDist.right']) then
		--i = #hokuio - (minLeft - 1) - 2
		--ref['minDist.right'] = radius(hokuio[i], hokuio[i+1])
		ref['minDist'] = ref['minDist.left']
	else
		--i = #hokuio - (minRight - 1) - 2
		--ref['minDist.left'] = radius(hokuio[i], hokuio[i+1])
		ref['minDist'] = ref['minDist.right']
	end
end

function isAlignedOnRightDirection()
	return (map.dir == 0 and ((ref['bussola'] > 355 and ref['bussola'] <= 359) or (ref['bussola'] >= 0 and ref['bussola'] < 5 )))
				or (map.dir == 90  and ref['bussola'] > 85  and ref['bussola'] < 95)
				or (map.dir == 270 and ref['bussola'] > 265 and ref['bussola'] < 275)
				or (map.dir == 180 and ref['bussola'] > 175 and ref['bussola'] < 185)
end

function onRightDirection()
	--log:write(map.dir, " ", ref['bussola'])
	return (map.dir == 0 and ((ref['bussola'] > 315 and ref['bussola'] <= 359) or (ref['bussola'] >= 0 and ref['bussola'] <= 45 )))
				or (map.dir == 90  and ref['bussola'] > 45  and ref['bussola'] <= 135)
				or (map.dir == 270 and ref['bussola'] > 225 and ref['bussola'] <= 315)
				or (map.dir == 180 and ref['bussola'] > 135 and ref['bussola'] <= 225)
end

-- Calculates the wheel speed necessary to align the robot with the door.
function alignToDoorControl(v, wheel)
	return v * math.exp(wheel * k['alignToDoor'] * ref['door'][2])
end

-- Calls correctly the alignToDoorControl function.
function alignToDoor()
	vLeft  = alignToDoorControl(vLeft , leftWheel )
	vRight = alignToDoorControl(vRight, rightWheel)
end

-- Calculates the wheel speed necessary to minimally put the door under laser vision.
function nextDoorDirectionControl(v, wheel, refdir)
	log:write(tostring(wheel * k['nextDoor'] * (refdir - ref['bussola'])))
	return v * math.exp(wheel * k['nextDoor'] * (refdir - ref['bussola']))
end

-- Calls correctly the nextDoorDirectionControl
function nextDoorDirection(rightDirection)
	vLeft  = nextDoorDirectionControl(vLeft , leftWheel , map.dir)
	vRight = nextDoorDirectionControl(vRight, rightWheel, map.dir)
end

-- Calculate the modulus of the vector (x,y).
function radius(x, y)
	if(x ~= nil and y ~= nil) then
		lastradius = math.sqrt(x*x + y*y)
	end
	return lastradius
end

-- Calculates the wheel speed necessary to avoid colision.
function avoidColisionControl(v, wheel, sixty, thirty, zero)
	local proximity = 0
	if(state == PASS_DOOR) then
		proximity = threshold['avoidColision.doorproximity']
	else
		proximity = threshold['avoidColision.usualproximity']
	end
	return v * math.exp( wheel * (
		k['avoidColision.sixty'] * proximity - sixty
		+ k['avoidColision.thirty'] * proximity / thirty
	))
end

-- Calls correctly the avoidColisionControl function.
--[[function avoidColision()
	if(ref['45'] <  ref['-45']) then
		vLeft  = avoidColisionControl(vLeft , leftWheel , ref['60'], ref['30'], 0)
		vRight = avoidColisionControl(vRight, rightWheel, ref['60'], ref['30'], 0)
	else
		vLeft  = avoidColisionControl(vLeft , leftWheel , ref['-60'], ref['-30'], 0)
		vRight = avoidColisionControl(vRight, rightWheel, ref['-60'], ref['-30'], 0)
	end
end]]

--[[function vFar(ref, front, ortho, diag)
	return v0
		* math.exp(- kl*(ref - ortho)
		- kdd * (ref - diag / (2 * khip)) 
		- kdr * ref / diag / khip
		- kf * ref / front)
end]]


function followWallControl(v, wheel, ortho, diag)
	--[[local detour = 0
	if(ref['follow'] == right) then
		detour = -1
	else
		detour =  1
	end
	log:write(normal, " ", ref['0'])]]
	return v * math.exp( (k['follow'] * wheel * (threshold['avoidColision.doorproximity'] - ortho)
						  + k['avoidColision.front'] * wheel * (threshold['avoidColision.usualproximity'] / ref['0']) 
						  + k['avoidColision.diag1'] * wheel * (threshold['avoidColision.usualproximity'] - diag) / math.sqrt(2)
						  + k['avoidColision.diag2'] * wheel * threshold['avoidColision.usualproximity'] / diag / math.sqrt(2) ))
end

function followWall()
	--log:write("follow ", ref['follow'])
	if(ref['follow'] == 'right') then
		vLeft  = followWallControl(vLeft , leftWheel , ref[ '90'], ref[ '45'])
		vRight = followWallControl(vRight, rightWheel, ref[ '90'], ref[ '45'])
	elseif(ref['follow'] == 'left') then
		vLeft  = followWallControl(vLeft , leftWheel , ref['-90'], ref['-45'])
		vRight = followWallControl(vRight, rightWheel, ref['-90'], ref['-45'])
	end
end

function avoidColisionControl(v, wheel)
	return v * math.exp( k['avoidColision.ortho'] * wheel * threshold['avoidColision.doorproximity'] / ref['90']
		- k['avoidColision.ortho'] * wheel * threshold['avoidColision.doorproximity'] / ref['-90']
		+ k['avoidColision.diag' ] * wheel * threshold['avoidColision.doorproximity'] / ref[ '45']
		- k['avoidColision.diag' ] * wheel * threshold['avoidColision.doorproximity'] / ref['-45']
		+ k['avoidColision.front'] * wheel * threshold['avoidColision.doorproximity'] / ref[  '0'] )
end

function avoidColision()
	vLeft  = avoidColisionControl(vLeft , leftWheel )
	vRight = avoidColisionControl(vRight, rightWheel)
end

-- Check and actuates if the robot is about to colide with something.
function checkCritDistance()
	if(ref['critLeft'] == 1) then
		if(ref['critRight'] == 1) then
			vLeft  = 0
			vRight = 0
		else
			vLeft  = -1.3
			vRight = -0.6
		end
		return false
	elseif(ref['critRight'] == 1) then
		vLeft  = -0.6
		vRight = -1.3
		return false
	else
		return true
	end
end

function corredor()
	if(state == LOOK_TO_TARGET) then
		nextDoorDirection()
		if(isAlignedOnRightDirection()) then
			log:write("\ncorridor, going to state aligntodoor")
			state = ALIGN_TO_DOOR
		end
	end
end

function sala()
	--log:write(tostring(ref['90']), " ", tostring(ref['-90']), " ", tostring(ref['0']))
	if(state == ALIGN_TO_DOOR) then
		alignToDoor()
		if(math.abs(ref['door'][2]) < 0.25) then
			transitionMinDist = ref['minDist']
			log:write("\ngoing to state gotodoor")
			state = GO_TO_DOOR
		elseif(ref['minDist'] < 0.8) then
			log:write("\ngoing to state passdoor")
			minEnclosure = ref['minDist.left']  + ref['minDist.right']
			state = PASS_DOOR

			--log:write(ref['90'], " ", ref['-90'])
			if(ref['90'] < ref['-90']) then
				ref['follow'] = 'right'
			else
				ref['follow'] = 'left'
			end
		end
	elseif(state == GO_TO_DOOR) then
		if(ref['minDist'] < 1) then
			log:write("\ngoing to state passdoor")
			minEnclosure = ref['minDist.left']  + ref['minDist.right']
			state = PASS_DOOR

			if(ref['90'] < ref['-90']) then
				ref['follow'] = 'right'
			else
				ref['follow'] = 'left'
			end
		elseif(ref['minDist'] > 1 and ref['minDist'] < 0.7 * transitionMinDist) then
			log:write("\ngoing to state aligntodoor")
			state = ALIGN_TO_DOOR
		end
	elseif(state == PASS_DOOR) then
		-- TODO Try a wall following using +/-90º laser beam as reference.
		avoidColision()
		local mdSum = ref['minDist.left']  + ref['minDist.right']
		if(mdSum < minEnclosure) then
			minEnclosure = mdSum
		elseif(mdSum > 1.5 * minEnclosure) then
			possibleDoor = false
			minEnclosure = 0
			map:nextRoom()
			state = LOOK_TO_TARGET
			ref['follow'] = 'none'
			log:write("\ngoing to state looktotarget")
		end
	elseif(state == LOOK_TO_TARGET) then
		--log:write(tostring(onRightDirection()))
		nextDoorDirection(onRightDirection())
		if(onRightDirection()) then
			log:write("\ngoing to state aligntodoor")
			state = ALIGN_TO_DOOR
		end
	end
end

if (sim_call_type==sim_childscriptcall_initialization) then
	log = io.open('pioneer.log', 'w')
	
 	hokuioTube = simTubeOpen(0,'_HOKUYO',1)
 	anglesTube = simTubeOpen(0,'_ANGLES',1)
	hokuio  = {}
	
	motorLeft=simGetObjectHandle("Pioneer_p3dx_leftMotor")
	motorRight=simGetObjectHandle("Pioneer_p3dx_rightMotor")
	v0 = 2
	vLeft=v0
	vRight=v0

	ALIGN_TO_DOOR = 0
	GO_TO_DOOR = 1
	PASS_DOOR = 2
	LOOK_TO_TARGET = 3

	state = LOOK_TO_TARGET
	
	--map = new_map('/home/ddiorio/Documents/USP20151/robosmoveis/TrabalhoFinal/maptop.txt', '/home/ddiorio/Documents/USP20151/robosmoveis/TrabalhoFinal/missao.txt')
	map = new_map('/home/laispc/FindBill/maptop.txt', '/home/laispc/FindBill/missao.txt')

	rightWheel = 1
	leftWheel = -1
	k = {}
	k['follow'] = 0.8
	k['nextDoor'] = 0.01
	k['alignToDoor'] = 0.5
	k['avoidColision.ortho'] = 0.1
	k['avoidColision.front'] = 0.1
	k['avoidColision.diag' ] = 0.1
	
	ref = {}
	ref['door'] = {0,0}
	ref[ '90']  = 10
	ref[ '45']  = 10
	ref[  '0']  = 10
	ref['-45']  = 10
	ref['-90']  = 10
	ref['follow']    = 'none'
	ref['critRight'] = 0
	ref['critLeft' ] = 0
	ref['minDist'  ] = 0
	ref['bussola'  ] = 0
	
	threshold = {}
	threshold['door'] = 0.7
	threshold['dLaser'] = 0.15
	threshold['avoidColision.usualproximity'] = 0.7
	threshold['avoidColision.doorproximity' ] = 0.31
	threshold['avoidColision.critProximity' ] = 0.29

	transitionMinDist = 0
	possibleDoor = false
	minEnclosure = 0
	lastradius = 10
end 

if (sim_call_type==sim_childscriptcall_cleanup) then 
	log:close();
	simTubeClose(hokuioTube)
	simTubeClose(anglesTube)
end 

if (sim_call_type==sim_childscriptcall_actuation) then
	readHokuio()
	findDoor()
	findAngles()
	findMinDist()

	vLeft=v0
	vRight=v0

	if(checkCritDistance()) then -- If no critical distances.
		if(map.actual_room.type == 'sala') then
			sala()
		elseif(map.actual_room.type == 'corredor') then
			corredor()
		end
	end
	
	simSetJointTargetVelocity(motorLeft ,vLeft )
	simSetJointTargetVelocity(motorRight,vRight)
	
	cleanRefs()
	log:write('\n')
end