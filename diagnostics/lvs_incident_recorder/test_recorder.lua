-- Run with SOURCE containing the recorder source, under an isolated Lua environment.
-- All entity, hook, command, cvar and disk APIs below are fakes; creates no game entities.
local now, tickNumber, population = 0, 0, 1
local enabled, files, hooks, commands = true, {}, {}, {}
local unix = os.time()
local entities = {}
local function valid(x) return istable(x) and x.valid == true end
local function entity(index, pos, class)
	local e = {valid = true, index = index, pos = pos, data = {}, class = class or "fixture"}
	function e:EntIndex() return self.index end
	function e:GetCreationID() return 1000 + self.index end
	function e:GetClass() return self.class end
	function e:GetTable() return self.data end
	function e:GetPos() return self.pos end
	function e:GetAngles() return Angle(0, 0, 0) end
	function e:IsPlayerHolding() return false end
	function e:GetForward() return Vector(1, 0, 0) end
	function e:GetRight() return Vector(1, 0, 0) end
	function e:WorldToLocal(p) return p - self.pos end
	function e:GetHP() return 100 end
	function e:GetNWDamaged() return false end
	function e:GetDestroyed() return false end
	function e:GetRPM() return 0 end
	function e:GetRotationAxis() return Vector(1, 0, 0) end
	local p = {valid = true, motion = true, velocity = Vector(0, 0, 0)}
	function p:GetPos() return e.pos end
	function p:GetAngles() return e:GetAngles() end
	function p:GetVelocity() return self.velocity end
	function p:GetAngleVelocity() return Vector(0, 0, 0) end
	function p:GetMass() return 100 end
	function p:GetInertia() return Vector(10, 8, 10) end
	function p:GetMassCenter() return Vector(0, 0, 0) end
	function p:IsMotionEnabled() return self.motion end
	function p:IsAsleep() return false end
	function p:IsCollisionEnabled() return true end
	function p:GetMaterial() return "jeeptire" end
	function p:LocalToWorld(localPos) return e.pos + localPos end
	function e:GetPhysicsObject() return p end
	function e:GetPhysicsObjectNum() return p end
	entities[index] = e
	return e
end
local vehicle = entity(1, Vector(0, 0, 0), "sw_brdm2")
local engineActive = false
function vehicle:GetEngineActive() return engineActive end
function vehicle:GetThrottle() return 0 end
function vehicle:GetBrake() return 0 end
function vehicle:GetSteer() return 0 end
function vehicle:GetReverse() return false end
function vehicle:GetDriver() return nil end
vehicle.data._WheelEnts = {}
for i = 1, 4 do
	local wheel = entity(10 + i, Vector(0, i * 10, 0))
	wheel.data._Master = entity(20 + i, wheel.pos)
	wheel.data.Constraints = {}
	for j = 1, 2 do
		local c = {valid = true}
		function c:EntIndex() return 0 end -- Multiple distinct HolyLib virtual constraints.
		local data = {Type = "Rope", Ent1 = vehicle, Ent2 = wheel, Bone1 = 0, Bone2 = 0,
			LPos1 = Vector(25 * (j == 1 and 1 or -1), i * 10, 0), LPos2 = Vector(0, 0, 0), length = 25, rigid = true}
		function c:GetTable() return data end
		wheel.data.Constraints[j] = c
	end
	vehicle.data._WheelEnts[i] = wheel
end
local cvar = {GetBool = function() return enabled end}
local env = setmetatable({SERVER = true, NCG_LVS_RECORDER = false,
	CreateConVar = function() return cvar end, GetConVar = function() return nil end,
	SysTime = function() return now end, CurTime = function() return now end,
	os = {time = function(t) return t and unix + 7200 or unix + math.floor(now) end,
		date = function() return "fixture" end},
	engine = {TickCount = function() return tickNumber end, TickInterval = function() return 1 / 22 end},
	player = {GetCount = function() return population end}, game = {GetMap = function() return "fixture" end},
	IsValid = valid, Entity = function(id) return entities[id] end,
	ents = {FindByClass = function() return {vehicle} end},
	hook = {Add = function(kind, _, fn) hooks[kind] = fn end, Remove = function(kind) hooks[kind] = nil end},
	concommand = {Add = function(name, fn) commands[name] = fn end}, print = function() end,
	file = {CreateDir = function() end, Find = function() return {} end,
		Size = function(path) return files[path] and #files[path] or -1 end,
		Write = function(path, data) files[path] = data end}}, {__index = getfenv(1)})
local compiled = CompileString(SOURCE, "isolated LVS observer fixture", false)
assert(isfunction(compiled), tostring(compiled))
setfenv(compiled, env)
compiled()
local api = env.NCG_LVS_RECORDER
local function step(n)
	for _ = 1, n do
		now, tickNumber = now + 1 / 22, tickNumber + 1
		assert(hooks.Tick, "recorder stopped unexpectedly")
		hooks.Tick()
		assert(not api.Status().error, api.Status().error)
	end
end
local function findCapture(suffix)
	for path, contents in pairs(files) do
		if string.EndsWith(path, suffix .. ".json") then return util.JSONToTable(contents) end
	end
end
CHECK(api.Status().target.index == 1, "auto-attaches to one existing BRDM")
step(270)
EQ(api.Status().samples, 256, "rolling buffer is bounded")
local baseline = findCapture("baseline")
CHECK(baseline and #baseline.samples > 0, "baseline is durable in fake storage")
EQ(table.Count(baseline.constraints), 8, "EntIndex 0 constraints retain distinct identities")
CHECK(not api.Status().captures.anomaly, "healthy geometry does not trigger")
engineActive = true
step(1)
EQ(api.Status().captures.engine, 1, "engine start preserves preceding samples")
now = now + 5
step(1)
CHECK(findCapture("engine_1_post") ~= nil, "engine event includes bounded post-event capture")
local checkpoint = findCapture("checkpoint_1")
for i = 2, #checkpoint.samples do
	CHECK(checkpoint.samples[i].tick > checkpoint.samples[i - 1].tick, "ring chronological " .. i)
end
local wheel = vehicle.data._WheelEnts[1]
wheel.pos = wheel.pos + Vector(0, 0, 30)
now = now + 31
step(8)
for _ = 1, 3 do now = now + 31 step(1) end
EQ(api.Status().captures.anomaly, 1, "persistent low-population error consumes only one tier slot")
population = 101
now = now + 31
step(1)
EQ(api.Status().captures.anomaly, 2, "new population tier preserves new evidence")
local incident = findCapture("anomaly_2_pre")
CHECK(incident.samples[#incident.samples].max_rod_error > 3, "authored rod error measured from physics-local anchors")
EQ(incident.samples[#incident.samples].players, 101, "incident population retained")
wheel:GetPhysicsObject().velocity = Vector(0 / 0, 0, 0)
population = 126
now = now + 31
step(1)
local nonfinite = findCapture("anomaly_3_pre")
CHECK(nonfinite and nonfinite.samples[#nonfinite.samples].nonfinite, "nonfinite velocity detected and JSON-safe")
local writtenBefore = api.Status().writes
vehicle.valid = false
step(1)
CHECK(not api.Status().target, "removed vehicle detaches")
CHECK(api.Status().writes > writtenBefore and findCapture("final") ~= nil, "last ring survives vehicle removal")
vehicle.valid = true
enabled = false
CHECK(not api.Watch(vehicle), "disabled recorder will not attach")
enabled = true
CHECK(api.Watch(vehicle), "explicit observer-only watch can reattach")
now = 7201
step(1)
CHECK(api.Status().expired and not api.Status().target, "dated canary expires and detaches")
CHECK(not api.Watch(vehicle), "expired canary stays inert")
return {checks = "complete", fake_disk_bytes = api.Status().disk_bytes, real_entities_created = 0,
	real_hooks_installed = 0, real_files_written = 0}
