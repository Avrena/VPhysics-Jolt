-- Optional SERVER-ONLY observer. Does not spawn, drive, repair, freeze or edit entities.
-- Disabled by default. An explicitly enabled session expires two hours after load.
if not SERVER then return end
if VJOLT_LVS_RECORDER then return end -- Auto-refresh must not discard a live ring.

local ENABLED = CreateConVar("vjolt_lvs_recorder_enabled", "0", FCVAR_ARCHIVE,
	"Observe one BRDM without changing physics; 0 stops the recorder.")
local EXPIRES = os.time() + 2 * 60 * 60
local DIR, HOOK = "vjolt_lvs_incidents", "VJOLT_LVSIncidentRecorder"
local MAX_FRAMES, MAX_WHEELS, MAX_CONSTRAINTS = 256, 8, 24
local MAX_FILE, MAX_DISK = 4 * 1024 * 1024, 64 * 1024 * 1024
local session = os.date("!%Y%m%dT%H%M%SZ") .. "_" .. engine.TickCount()
local target, watch, lastError, lastFile
local watchNumber, stopped = 0, false
local diskBytes, writes, sampleSeconds, sampleCount, sampleMax = 0, 0, 0, 0, 0
local api = {}
VJOLT_LVS_RECORDER = api
file.CreateDir(DIR)
for _, name in ipairs(file.Find(DIR .. "/*.json", "DATA")) do
	diskBytes = diskBytes + math.max(file.Size(DIR .. "/" .. name, "DATA"), 0)
end

local function num(n)
	if not isnumber(n) then return n end
	if n ~= n then return "nan" end
	if n == math.huge then return "+inf" end
	if n == -math.huge then return "-inf" end
	return n
end

local function xyz(v)
	if not isvector(v) then return nil end
	return {num(v.x), num(v.y), num(v.z)}
end

local function angles(a) return {num(a.p), num(a.y), num(a.r)} end
local function finite3(a)
	return not a or (isnumber(a[1]) and isnumber(a[2]) and isnumber(a[3]))
end
local function identity(e)
	if not IsValid(e) then return false end
	return {index = e:EntIndex(), creation = e:GetCreationID()}
end

local function getter(e, name)
	local fn = e[name]
	if isfunction(fn) then return num(fn(e)) end
end

local function body(e)
	if not IsValid(e) then return {valid = false} end
	local p = e:GetPhysicsObject()
	local out = {entity = identity(e), pos = xyz(e:GetPos()), ang = angles(e:GetAngles()),
		held = e:IsPlayerHolding(), valid = IsValid(p)}
	if not out.valid then return out end
	out.physics = tostring(p) -- Physical-object replacement, not a Lua entity index.
	out.ppos, out.pang = xyz(p:GetPos()), angles(p:GetAngles())
	out.vel, out.avel_local = xyz(p:GetVelocity()), xyz(p:GetAngleVelocity())
	out.mass, out.inertia, out.com_local = num(p:GetMass()), xyz(p:GetInertia()), xyz(p:GetMassCenter())
	out.motion, out.asleep, out.collisions = p:IsMotionEnabled(), p:IsAsleep(), p:IsCollisionEnabled()
	out.material = p:GetMaterial()
	out.nonfinite = not (finite3(out.pos) and finite3(out.ang) and finite3(out.ppos)
		and finite3(out.pang) and finite3(out.vel) and finite3(out.avel_local))
	return out
end

local fields = {"Type", "Bone1", "Bone2", "length", "addlength", "rigid", "onlyrotation",
	"xmin", "ymin", "zmin", "xmax", "ymax", "zmax", "xfric", "yfric", "zfric",
	"constant", "damping", "rdamping", "stretchonly", "nocollide", "forcelimit", "torquelimit"}
local function constraints(wheel, frame)
	local result, seen = {}, {}
	local count = 0
	-- Do not call constraint.GetTable/GetWheels: those helpers can clean up live tables.
	for _, c in pairs(wheel:GetTable().Constraints or {}) do
		count = count + 1
		if count > MAX_CONSTRAINTS then frame.truncated = true break end
		if IsValid(c) and not seen[c] then
			seen[c] = true
			local d = c:GetTable()
			local def = watch.constraintIDs[c]
			if not def then
				watch.nextConstraint = watch.nextConstraint + 1
				def = {uid = watch.nextConstraint, ent_index = c:EntIndex(),
					ent1 = identity(d.Ent1), ent2 = identity(d.Ent2),
					lpos1 = xyz(d.LPos1), lpos2 = xyz(d.LPos2)}
				for _, k in ipairs(fields) do def[k] = num(d[k]) end
				-- HolyLib virtual constraints can all have EntIndex 0. Key by object.
				watch.constraintIDs[c] = def
			end
			frame.definitions[def.uid] = def
			local item = {uid = def.uid}
			if (d.Type == "Rope" or d.Type == "Elastic") and IsValid(d.Ent1) and IsValid(d.Ent2)
				and isvector(d.LPos1) and isvector(d.LPos2) then
				local p1 = d.Ent1:GetPhysicsObjectNum(d.Bone1 or 0)
				local p2 = d.Ent2:GetPhysicsObjectNum(d.Bone2 or 0)
				if IsValid(p1) and IsValid(p2) then
					local distance = p1:LocalToWorld(d.LPos1):Distance(p2:LocalToWorld(d.LPos2))
					item.distance = num(distance)
					if d.Type == "Rope" and isnumber(d.length) then
						local delta = distance - d.length - (d.addlength or 0)
						local error = (d.rigid == true or d.rigid == 1) and math.abs(delta) or math.max(delta, 0)
						item.error = num(error)
						if num(error) ~= error then frame.nonfinite = true
						else frame.max_rod_error = math.max(frame.max_rod_error, error) end
					end
				end
			end
			result[#result + 1] = item
		end
	end
	return result
end

local function snapshot(now)
	local v = target
	local vt = v:GetTable()
	local f = {tick = engine.TickCount(), curtime = CurTime(), realtime = now,
		wall_delta = watch.lastSample and now - watch.lastSample or 0, unix = os.time(),
		players = player.GetCount(), engine = getter(v, "GetEngineActive"),
		throttle = getter(v, "GetThrottle"), brake = getter(v, "GetBrake"), steer = getter(v, "GetSteer"),
		reverse = getter(v, "GetReverse"), hp = getter(v, "GetHP"), driver = IsValid(getter(v, "GetDriver")),
		chassis = body(v), wheels = {}, definitions = {}, max_rod_error = 0, min_axis_dot = 1,
		shared_vector_origin = xyz(vector_origin),
		deferred_nocollide = {vt._DeferredRootWheelNoCollideScheduled, vt._DeferredRootWheelNoCollideCreated,
			vt._DeferredRootWheelNoCollidePending, vt._DeferredRootWheelNoCollideFailed}}
	local count = 0
	for _, w in pairs(vt._WheelEnts or {}) do
		count = count + 1
		if count > MAX_WHEELS then f.truncated = true break end
		if IsValid(w) then
			local wt = w:GetTable()
			local m = wt._Master
			local state = body(w)
			state.local_pos = xyz(v:WorldToLocal(w:GetPos()))
			state.hp, state.damaged, state.destroyed = getter(w, "GetHP"), getter(w, "GetNWDamaged"), getter(w, "GetDestroyed")
			state.rpm, state.axle = getter(w, "GetRPM"), wt.axle
			state.lock, state.damage_lock = IsValid(wt.bsLock), IsValid(wt.bsLockDMG)
			state.handbrake, state.lock_until = wt._handbrakeActive == true, num(wt._RotationLockTime)
			state.original_mass, state.suspension_disabled = num(wt._OriginalMass), wt._IsSuspensionDisabled == true
			state.old_tire_material, state.leaking = wt._OldTirePhysProp, wt._IsLeakingAir == true
			state.suspension_height, state.suspension_stiffness = num(wt._SuspensionHeightMultiplier), num(wt._SuspensionStiffnessMultiplier)
			-- LVS names these backwards: Force is angular, ForceAng is linear.
			state.command_angular_local, state.command_linear = xyz(wt.Force), xyz(wt.ForceAng)
			state.rotation_axis_local = xyz(getter(w, "GetRotationAxis"))
			state.simulate, state.next_simulate = wt.Simulate, num(wt._NextSimulate)
			state.master = body(m)
			if state.nonfinite or state.master.nonfinite or not finite3(state.command_angular_local)
				or not finite3(state.command_linear) or not finite3(state.rotation_axis_local) then f.nonfinite = true end
			if IsValid(m) then
				local dot = w:GetForward():Dot(m:GetRight())
				state.axis_dot = num(dot)
				if num(dot) ~= dot then f.nonfinite = true else f.min_axis_dot = math.min(f.min_axis_dot, dot) end
			end
			state.constraints = constraints(w, f)
			f.wheels[#f.wheels + 1] = state
		end
	end
	if f.chassis.nonfinite then f.nonfinite = true end
	return f
end

local function ringFrames()
	local frames = {}
	for n = 1, watch.count do
		frames[n] = watch.ring[(watch.cursor - watch.count + n - 1) % MAX_FRAMES + 1]
	end
	return frames
end

local function save(suffix, reason)
	if not watch then return false end
	local frames, defs = {}, {}
	for _, f in ipairs(ringFrames()) do
		local copy = {}
		for k, value in pairs(f) do if k ~= "definitions" then copy[k] = value end end
		for uid, def in pairs(f.definitions) do defs[tostring(uid)] = def end
		frames[#frames + 1] = copy
	end
	local path = DIR .. "/" .. watch.key .. "_" .. suffix .. ".json"
	local payload = util.TableToJSON({schema = 1, reason = reason, metadata = watch.metadata,
		constraints = defs, samples = frames}, false)
	if not payload or #payload > MAX_FILE then lastError = "capture exceeds per-file cap" return false end
	local oldSize = math.max(file.Size(path, "DATA"), 0)
	if diskBytes - oldSize + #payload > MAX_DISK then lastError = "64 MiB disk cap reached; copy out evidence" return false end
	file.Write(path, payload)
	if file.Size(path, "DATA") ~= #payload then lastError = "capture write failed" return false end
	diskBytes, writes, lastFile = diskBytes - oldSize + #payload, writes + 1, path
	return true
end

local function capture(reason, category)
	if not watch then return false end
	local now = SysTime()
	category = category or "manual"
	if watch.pending or now < watch.nextCapture then return false end
	local count = watch.captures[category] or 0
	if count >= (category == "anomaly" and 6 or 8) then return false end
	local label = category .. "_" .. (count + 1)
	if not save(label .. "_pre", reason) then return false end
	watch.captures[category] = count + 1
	watch.pending = {label = label, reason = reason, untilTime = now + 5}
	watch.nextCapture = now + 30
	print("[LVS recorder] Captured " .. reason .. ": " .. lastFile)
	return true
end

local function detach(reason)
	if watch then
		if watch.pending then save(watch.pending.label .. "_post", watch.pending.reason .. "; " .. reason) end
		save("final", reason)
	end
	target, watch = nil, nil
end

function api.Watch(v)
	if stopped then return false, "recorder stopped on error; inspect Status before reloading" end
	if not ENABLED:GetBool() or os.time() >= EXPIRES then return false, "disabled or expired" end
	if not IsValid(v) or v:GetClass() ~= "sw_brdm2" then return false, "select a valid sw_brdm2" end
	if v == target then return true end
	detach("watch target changed")
	target = v
	watchNumber = watchNumber + 1
	local cvars = {}
	for _, name in ipairs({"vjolt_substeps", "vjolt_velocity_steps", "vjolt_position_steps",
		"vjolt_onlyrot_recapture_ticks", "vjolt_constraint_position_substeps", "vjolt_baumgarte_factor", "gmod_physiterations"}) do
		local cv = GetConVar(name)
		if cv then cvars[name] = cv:GetString() end
	end
	watch = {key = session .. "_" .. v:GetCreationID() .. "_" .. watchNumber, ring = {}, count = 0, cursor = 0,
		started = SysTime(), nextCheckpoint = SysTime() + 5, checkpoint = 0,
		constraintIDs = setmetatable({}, {__mode = "k"}), nextConstraint = 0,
		captures = {}, engineTiers = {}, anomalyTiers = {}, nextCapture = 0,
		metadata = {session = session, map = game.GetMap(), vehicle = identity(v), class = v:GetClass(),
			tick_interval = engine.TickInterval(), cvars = cvars, expires_unix = EXPIRES,
			units = "Source inches; entity Euler degrees; physics angular velocity and angular commands LOCAL; master translation is intentionally static",
			constraint_note = "Lua authored definitions, not native solver frames; uid distinguishes virtual constraints with EntIndex 0"}}
	print("[LVS recorder] Watching BRDM " .. v:EntIndex() .. " (creation " .. v:GetCreationID() .. ")")
	return true
end

function api.Status()
	return {version = 1, enabled = ENABLED:GetBool(), expires_unix = EXPIRES, expired = os.time() >= EXPIRES, stopped = stopped,
		target = identity(target), session = session, samples = watch and watch.count or 0,
		last_tick = watch and watch.lastFrame and watch.lastFrame.tick, last_file = lastFile,
		last_players = watch and watch.lastFrame and watch.lastFrame.players,
		last_engine = watch and watch.lastFrame and watch.lastFrame.engine,
		captures = watch and watch.captures, disk_bytes = diskBytes, writes = writes, error = lastError,
		sample_mean_ms = sampleCount > 0 and sampleSeconds / sampleCount * 1000 or 0,
		sample_max_ms = sampleMax * 1000}
end

function api.Mark(reason) return capture(string.sub(tostring(reason or "manual observation"), 1, 120), "manual") end

local function tick()
	if not watch then return end
	if not ENABLED:GetBool() or os.time() >= EXPIRES then detach("disabled or expired") return end
	if not IsValid(target) then detach("vehicle removed") return end
	local now = SysTime()
	-- Sample every tick at 22 Hz, without exceeding 25 Hz on faster configurations.
	if watch.lastSample and now - watch.lastSample < 0.039 then return end
	local f = snapshot(now)
	watch.lastSample = now
	watch.cursor = watch.cursor % MAX_FRAMES + 1
	watch.ring[watch.cursor] = f
	watch.count = math.min(watch.count + 1, MAX_FRAMES)
	local prev = watch.lastFrame
	watch.lastFrame = f
	if not watch.initialWheels and #f.wheels > 0 then watch.initialWheels = #f.wheels end
	if watch.pending and now >= watch.pending.untilTime then
		save(watch.pending.label .. "_post", watch.pending.reason)
		watch.pending = nil
	end
	local reason
	local tier = math.floor(f.players / 25)
	if f.nonfinite then reason = "nonfinite geometry"
	elseif now - watch.started > 2 then
		if #f.wheels < (watch.initialWheels or 0) then reason = "wheel missing"
		elseif f.max_rod_error > 3 then reason = "suspension constraint error > 3 inches"
		elseif f.min_axis_dot < 0.996 then reason = "wheel/master axis misalignment"
		else
			for _, w in ipairs(f.wheels) do
				if not w.valid or not w.master.valid or w.motion == false then reason = "wheel/master physics invalid or wheel frozen" break end
			end
		end
	end
	if reason then
		watch.badSince = watch.badSince or now
		-- Reserve captures for population growth; an early persistent skew must not
		-- consume every incident slot hours before the public test.
		if not watch.anomalyTiers[tier] and (f.nonfinite or now - watch.badSince >= 0.25) then
			if capture(reason, "anomaly") then watch.anomalyTiers[tier] = true end
		end
	else watch.badSince = nil end
	if prev and f.engine and not prev.engine and not watch.engineTiers[tier] then
		if capture("engine started at population tier " .. tier, "engine") then watch.engineTiers[tier] = true end
	end
	if now >= watch.nextCheckpoint then
		-- Alternating bounded checkpoints retain earlier evidence if a write/server is interrupted.
		watch.checkpoint = 1 - watch.checkpoint
		save("checkpoint_" .. watch.checkpoint, "rolling checkpoint")
		if not watch.baseline then watch.baseline = save("baseline", "initial observation; health not assumed") end
		watch.nextCheckpoint = now + 15
	end
end

hook.Add("Tick", HOOK, function()
	local started = SysTime()
	local ok, err = pcall(tick)
	local elapsed = SysTime() - started
	if watch then sampleCount, sampleSeconds, sampleMax = sampleCount + 1, sampleSeconds + elapsed, math.max(sampleMax, elapsed) end
	if not ok then
		lastError = tostring(err)
		stopped = true
		pcall(detach, "recorder stopped on error")
		-- Fail closed: no repeated errors each tick and no corrective physics actions.
		hook.Remove("Tick", HOOK)
		print("[LVS recorder] Stopped on error: " .. lastError)
	end
end)
hook.Add("OnEntityCreated", HOOK, function(e)
	if not watch and IsValid(e) and e:GetClass() == "sw_brdm2" then api.Watch(e) end
end)
hook.Add("ShutDown", HOOK, function() if watch then pcall(detach, "server shutdown") end end)
hook.Add("PreCleanupMap", HOOK, function() if watch then pcall(detach, "map cleanup") end end)

local function allowed(ply) return not IsValid(ply) or ply:IsSuperAdmin() end
concommand.Add("vjolt_lvs_recorder_status", function(ply)
	if allowed(ply) then print(util.TableToJSON(api.Status(), true)) end
end)
concommand.Add("vjolt_lvs_recorder_watch", function(ply, _, args)
	if allowed(ply) then print(api.Watch(Entity(tonumber(args[1]) or -1))) end
end)
concommand.Add("vjolt_lvs_recorder_mark", function(ply, _, _, text)
	if allowed(ply) then print(api.Mark(text)) end
end)

-- One startup lookup only. Refuse to guess when multiple existing BRDMs exist.
local existing = ents.FindByClass("sw_brdm2")
if #existing == 1 then api.Watch(existing[1]) end
