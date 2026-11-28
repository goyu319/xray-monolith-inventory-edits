#pragma once
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace PainterFireSpread
{
struct V3
{
	float x, y, z;
};
enum Kind : uint8_t { GROUND = 0, WALL = 1, CEILING = 2 };
// what a surface is, to a fire
enum Fuel : uint8_t
{
	NOTHING = 0, // rock, asphalt, bare earth, concrete in the open, metal, water: never catches
	GRASS,       // grass and low plants: catches at once, burns out fast, runs with the wind
	BRUSH,       // bushes, tree crowns
	WOOD,        // boards, logs, trunks: slow to catch, burns long
	CLOTH,
	CONTENTS,    // a room whose surfaces themselves do not burn: what stands, hangs and lies in it
	FUEL_COUNT
};
enum State : uint8_t { COLD = 0, CATCHING, BURNING, DYING, BURNT };

struct Site // one answer of the level
{
	V3 p, n;      // the point on the surface, and the way the surface faces
	Fuel fuel;
	float amount; // 0..1: how much of that fuel is there (grass: how thick it grows)
	bool roofed;  // something is overhead: rain and most of the wind do not reach it
	bool trunk;   // a tree's trunk (its fire is a column around it, not a fire on a wall)
	float lid = 0.f; // how high what is overhead lies (a height in the level); 0 = open sky, or the level did not say
	int woodFace = -1; // originating static collision face for wood residue
};
struct World
{
	// the ground at a point: looked for from `up` above it to `down` below it
	virtual bool ground(const V3& at, float up, float down, Site& out) = 0;
	// the first upright surface from `from` towards `dir` (level, unit length) within `reach`
	virtual bool wall(const V3& from, const V3& dir, float reach, Site& out) = 0;
	// the first surface overhead within `reach`
	virtual bool ceiling(const V3& from, float reach, Site& out) = 0;
	// a tree's trunk within `radius` (level) of a point, between `low` and `high` above it: the nearest place on it
	virtual bool trunk(const V3& at, float radius, float low, float high, Site& out) { return false; }
	// firesides: the other side of a thin wooden wall in the open. 1 = its back face (out is that face); 2 = a board with
	// one face only and open air behind it (out.p / out.n = its back; the caller gives it the front's fuel); 0 = none
	virtual int thinBack(const V3& p, const V3& n, Site& out) { return 0; }
	virtual ~World() {}
};
struct Weather
{
	float windX = 0.f, windZ = 0.f; // m/s, the way the wind blows to
	float wet = 0.f;                // 0 dry .. 1 soaked: builds up while it rains, dries afterwards
	float rain = 0.f;               // 0 none .. 1 pouring, right now
};
struct Rule
{
	float catchTime; // seconds for a cell to catch from one fully burning cell next to it, calm and dry
	float burnTime;  // seconds it burns
	float dieTime;   // seconds it dies down
	float size;      // 0..1: how big its flames get
	float wind;      // 0..1: how much the wind drives it
	float wetStop;   // in the open: the wetness at which it no longer catches
};
inline const Rule& rule(Fuel f)
{
	static const Rule rules[FUEL_COUNT] = {
		{0.f, 0.f, 3.f, 0.f, 0.f, 0.f},      // nothing
		// firerework: every fuel dies down slower (was 2.5 / 5 / 10 / 4 / 8 s): a fire shrinks away instead of vanishing
		{1.6f, 5.5f, 5.f, .65f, 1.f, .75f},  // grass: a thin front that runs, black ground behind it
		{4.0f, 22.f, 9.f, 1.f, .8f, .85f},   // brush
		{9.0f, 70.f, 18.f, .85f, .35f, .95f}, // wood
		{3.0f, 14.f, 7.f, .6f, .5f, .8f},    // cloth
		{6.0f, 35.f, 14.f, .7f, .2f, 2.f},   // the contents of a room
	};
	return rules[f < FUEL_COUNT ? f : NOTHING];
}

struct Link
{
	uint32_t to;
	float k;      // how well the heat gets there (distance, up or down)
	float dx, dz; // the level direction to it (unit), or 0 0 straight up or down
};
struct Cell
{
	V3 p = {0.f, 0.f, 0.f}, n = {0.f, 1.f, 0.f};
	Kind kind = GROUND;
	Fuel fuel = NOTHING;
	State state = COLD;
	bool roofed = false;
	bool trunk = false;
	bool probed = false;   // what it touches is known
	bool source = false;   // burning fuel was thrown here: it burns whatever the surface is
	bool warm = false;     // in the list of cells that are being heated
	float lid = 0.f;       // how high what is overhead lies, 0 = open sky (told for ground and ceiling cells): no fire goes through it
	float amount = 0.f;
	float heat = 0.f;      // 0..1: how close to catching
	float t = 0.f;         // seconds in this state
	float burnFor = 0.f;
	float sourceFor = 0.f; // seconds of thrown fuel left
	float until = 0.f;     // its fire may spread until then (the field's clock)
	float ox = 0.f, oz = 0.f; // where its fire started
	float size = 0.f;      // 0..1: how big its flames are right now
	float crowd = 0.f;     // how much burns around it: a cell in the middle of a fire burns taller than one alone
	float fedAt = 0.f;     // when it was last heated (the field's clock)
	int32_t kx = 0, ky = 0, kz = 0;
	int woodFace = -1;
	std::vector<Link> links;
};

class Field
{
public:
	float cellSize = 1.6f;
	float speed = 1.f;         // all catching times are divided by this
	float burnK = 1.f;         // how long a patch burns, times its fuel's own time
	float spreadFor = 180.f;
	float reach = 80.f;        // no cell catches farther than this from where its fire started
	uint32_t maxBurning = 2000; // more than this never burn at once: the rest waits (the renderer shows fewer, see there)
	uint32_t maxCells = 60000;
	std::vector<Cell> cells;
	std::vector<uint32_t> changed; // the cells whose state changed in the last update or ignite
	float clock = 0.f;
	uint32_t burning = 0;          // catching + burning + dying
	uint32_t dying = 0;            // firerework: of those, dying (they do not count against maxBurning)
	uint32_t burnt = 0;

	void clear()
	{
		cells.clear();
		changed.clear();
		index.clear();
		active.clear();
		warm.clear();
		toProbe.clear();
		probeAt = 0;
		clock = 0.f;
		burning = burnt = dying = 0;
	}
	bool idle() const { return active.empty() && warm.empty(); }
	// what burnt there has grown back: it can catch again (the renderer's painter_fire_burnt_minutes)
	void regrow(uint32_t i)
	{
		if (i >= cells.size()) return;
		Cell& c = cells[i];
		if (c.state != BURNT) return;
		c.state = COLD;
		c.heat = 0.f;
		c.t = 0.f;
		c.size = 0.f;
		c.source = false;
		c.sourceFor = 0.f;
		c.burnFor = 0.f;
		if (burnt) --burnt;
	}
	const std::vector<uint32_t>& burningCells() const { return active; }
	// the ground cell at a place (its own storey, the one below, the one above), or NO_CELL
	enum : uint32_t { NO_CELL = 0xffffffffu };
	uint32_t groundAt(float x, float y, float z) const
	{
		const int kx = lattice(x), kz = lattice(z), ky = storey(y);
		for (int d = 0; d < 3; ++d)
		{
			const auto found = index.find(key(GROUND, kx, ky + (d == 0 ? 0 : d == 1 ? -1 : 1), kz, 0));
			if (found != index.end()) return found->second;
		}
		return NO_CELL;
	}

	// fireheat (2026-11-24): the cells around a place - ground cells `reach` lattice cells around it (its storey, the one
	// below and above), wall cells one lattice cell around it from 0.6 m under to 2 m over it (every face, every slot)
	void cellsNear(float x, float y, float z, int reach, std::vector<uint32_t>& out) const
	{
		out.clear();
		const int kx = lattice(x), kz = lattice(z), ks = storey(y);
		const int wy0 = lattice(y - .6f), wy1 = lattice(y + 2.f);
		for (int dz = -reach; dz <= reach; ++dz)
			for (int dx = -reach; dx <= reach; ++dx)
			{
				for (int d = -1; d <= 1; ++d)
				{
					const auto f = index.find(key(GROUND, kx + dx, ks + d, kz + dz, 0));
					if (f != index.end()) out.push_back(f->second);
				}
				if (dx < -1 || dx > 1 || dz < -1 || dz > 1) continue;
				for (int ky = wy0; ky <= wy1; ++ky)
					for (int face = 0; face < 4; ++face)
					{
						const uint64_t base = key(WALL, kx + dx, ky, kz + dz, face);
						for (uint32_t slot = 0; slot < 256; ++slot)
						{
							const auto it = index.find(base | (uint64_t(slot) << 55));
							if (it == index.end()) break;
							out.push_back(it->second);
						}
					}
			}
	}
	// the wall cell at a lattice place with a wall's facing, or NO_CELL
	uint32_t wallAt(int kx, int ky, int kz, const V3& n,const Cell* source=nullptr) const
	{
		const int face = std::fabs(n.x) > std::fabs(n.z) ? (n.x > 0.f ? 0 : 1) : (n.z > 0.f ? 2 : 3);
        const uint64_t base=key(WALL,kx,ky,kz,face);
        float best=1e9f;uint32_t nearest=NO_CELL;
        for(uint32_t slot=0;slot<256;++slot){
            const auto it=index.find(base|(uint64_t(slot)<<55));
            if(it==index.end())break;
            if(!source)return it->second;
            const auto& c=cells[it->second];
            if(c.trunk!=source->trunk||c.fuel!=source->fuel)continue;
            const float dx=c.p.x-source->p.x,dz=c.p.z-source->p.z;
            const float d=dx*dx+dz*dz;
            if(d<best){best=d;nearest=it->second;}
        }
        return nearest;
	}

    bool igniteWall(const Site& site,float seconds)
    {
        const uint32_t i=wallCell(site);
        if(i==NONE)return false;
        igniteSource(i,site.p,seconds);
        return true;
    }
	// burning fuel lands at p: the ground within `radius` burns for `seconds`, whatever it is made of
	bool ignite(World& w, const V3& p, float radius, float seconds, const Site* splash = nullptr)
	{
		const float S = cellSize;
		const int cx = lattice(p.x), cz = lattice(p.z), r = int(std::ceil(radius / S));
		bool any = false;
		for (int dz = -r; dz <= r; ++dz)
			for (int dx = -r; dx <= r; ++dx)
			{
				const float x = float(cx + dx) * S, z = float(cz + dz) * S;
				const float d = std::sqrt((x - p.x) * (x - p.x) + (z - p.z) * (z - p.z));
				if ((dx || dz) && d > radius) continue;
				Site s;
                if (splash && !splash->trunk && (x-splash->p.x)*splash->n.x+(z-splash->p.z)*splash->n.z<-.03f) continue;
                const V3 at = {x, p.y, z};
                if (!w.ground(at, splash ? .20f : 1.3f, 1.6f, s)) continue;
				const uint32_t i = groundCell(s, cx + dx, cz + dz);
				if (i == NONE) continue;
				igniteSource(i, p, seconds);
				any = true;
			}
		return any;
	}

	// A small flame is held to the ground at p (a match, a lighter): the nearest cell there that can burn catches, unless
	// it is too wet. Nothing is thrown: bare ground stays bare. 0 = nothing here burns, 1 = too wet, 2 = it catches (or
	// burns already).
	int light(World& w, const V3& p, const Weather& wx)
	{
		const float S = cellSize;
		const int cx = lattice(p.x), cz = lattice(p.z);
		int answer = 0;
		uint32_t pick = NONE;
		float best = 1e9f;
		for (int dz = -1; dz <= 1; ++dz)
			for (int dx = -1; dx <= 1; ++dx)
			{
				const float x = float(cx + dx) * S, z = float(cz + dz) * S;
				const float d = (x - p.x) * (x - p.x) + (z - p.z) * (z - p.z);
				if (d > 1.5f * 1.5f || d >= best) continue;
				Site s;
				const V3 at = {x, p.y, z};
				if (!w.ground(at, 1.3f, 1.6f, s)) continue;
				const uint32_t i = groundCell(s, cx + dx, cz + dz);
				if (i == NONE) continue;
				const Cell& c = cells[i];
				if (c.state == CATCHING || c.state == BURNING) return 2;
				if (c.state != COLD || c.fuel == NOTHING) continue;
				if (!c.roofed && (wx.wet >= rule(c.fuel).wetStop || wx.rain > .6f))
				{
					answer = 1;
					continue;
				}
				best = d;
				pick = i;
			}
		if (pick == NONE) return answer;
		Cell& c = cells[pick];
		c.state = CATCHING;
		c.t = 0.f;
		c.heat = 1.f;
		c.burnFor = rule(c.fuel).burnTime * (.5f + .5f * c.amount) * burnK;
		c.until = clock + spreadFor;
		c.ox = p.x;
		c.oz = p.z;
		++burning;
		active.push_back(pick);
		if (!c.probed) toProbe.push_back(pick);
		changed.push_back(pick);
		return 2;
	}

	// one step: `probes` = how many cells may ask the level what they touch in this step (each asks up to 20 questions)
	void update(float dt, World& w, const Weather& wx, int probes)
	{
		changed.clear();
		if (idle()) return;
		clock += dt;
		while (probes > 0 && probeAt < toProbe.size())
		{
			const uint32_t i = toProbe[probeAt++];
			if (cells[i].probed) continue;
			probe(w, i);
			--probes;
		}
		if (probeAt == toProbe.size())
		{
			toProbe.clear();
			probeAt = 0;
		}
		// burning cells heat what they touch
		for (size_t a = 0; a < active.size(); ++a)
		{
			const uint32_t i = active[a];
			const float out = power(cells[i], wx);
			if (out <= 0.f || !cells[i].probed || cells[i].until <= clock) continue;
			const float until = cells[i].until, ox = cells[i].ox, oz = cells[i].oz;
			const size_t count = cells[i].links.size();
			for (size_t l = 0; l < count; ++l)
			{
				const Link link = cells[i].links[l];
				Cell& n = cells[link.to];
				if (n.state != COLD)
				{
					if (n.state == BURNING || n.state == CATCHING) n.crowd += out * dt; // they feed each other
					continue;
				}
				if (n.fuel == NOTHING) continue;
				const Rule& r = rule(n.fuel);
				// no two spots catch alike: the front of a fire is a ragged line, never a ruled one
				const uint32_t h = (link.to * 2654435761u) >> 16;
				float k = link.k * (.35f + .65f * n.amount) * (.55f + .9f * float(h & 0xffu) / 255.f);
				if (link.dx != 0.f || link.dz != 0.f)
				{
					// with the wind the fire runs, against it it creeps
					const float along = (wx.windX * link.dx + wx.windZ * link.dz) * (n.roofed ? .2f : 1.f) * r.wind;
					// (at most 3.5 times: at 10 a front in a strong wind crossed a cell in a sixth of a second, which read as the
					// fire jumping ahead instead of burning its way there)
					float f = std::exp(.33f * along);
					k *= f < .2f ? .2f : f > 3.5f ? 3.5f : f;
				}
				if (!n.roofed)
				{
					const float dry = 1.f - wx.wet / r.wetStop;
					if (dry <= 0.f) continue; // too wet to catch
					k *= dry * (1.f - .5f * wx.rain);
				}
				n.heat += dt * speed * out * k / r.catchTime;
				n.fedAt = clock;
				if (n.until < until)
				{
					n.until = until;
					n.ox = ox;
					n.oz = oz;
				}
				if (!n.warm)
				{
					n.warm = true;
					warm.push_back(link.to);
				}
			}
		}
		// cold cells catch or cool
		for (size_t a = 0; a < warm.size();)
		{
			const uint32_t i = warm[a];
			Cell& c = cells[i];
			bool keep = c.state == COLD;
			if (keep && c.heat >= 1.f)
			{
				const float dx = c.p.x - c.ox, dz = c.p.z - c.oz;
				if (burning - dying < maxBurning && dx * dx + dz * dz <= reach * reach) // firerework: the dying do not count
				{
					const Rule& r = rule(c.fuel);
					c.state = CATCHING;
					c.t = 0.f;
					c.burnFor = r.burnTime * (.5f + .5f * c.amount) * burnK;
					++burning;
					active.push_back(i);
					if (!c.probed) toProbe.push_back(i);
					changed.push_back(i);
					keep = false;
				}
				else
					c.heat = .95f; // too much burns already, or too far from where its fire started: it waits, and cools if nothing feeds it
			}
			else if (keep && clock - c.fedAt > 1.f)
			{
				c.heat -= dt * .05f; // nothing feeds it any more: it cools
				if (c.heat <= 0.f)
				{
					c.heat = 0.f;
					keep = false;
				}
			}
			if (keep)
				++a;
			else
			{
				c.warm = false;
				warm[a] = warm.back();
				warm.pop_back();
			}
		}
		// what burns, burns down
		for (size_t a = 0; a < active.size();)
		{
			const uint32_t i = active[a];
			Cell& c = cells[i];
			const Rule& r = rule(c.fuel);
			const float rain = c.roofed ? 0.f : wx.rain;
			const float ramp = catchRamp(c.fuel);
			const float crowd = c.crowd > 4.f ? 4.f : c.crowd;
			c.crowd *= 1.f - (dt > 1.f ? 1.f : dt); // forgets in about a second
			const float full = (c.fuel == NOTHING ? .3f : r.size * (.45f + .55f * c.amount)) * (.7f + .15f * crowd) * (1.f - .35f * rain);
			bool gone = false;
			switch (c.state)
			{
			case CATCHING:
				c.t += dt;
				c.size = full * (c.t / ramp);
				if (c.t >= ramp)
				{
					c.state = BURNING;
					c.t = 0.f;
					changed.push_back(i);
				}
				break;
			case BURNING:
				c.t += dt * (1.f + 1.5f * rain); // rain beats it down sooner
				c.size += (full - c.size) * (dt * 1.5f > 1.f ? 1.f : dt * 1.5f);
				if (c.source)
				{
					c.sourceFor -= dt;
					if (c.sourceFor <= 0.f)
					{
						// the thrown fuel is spent: what is under it burns on, if it can
						c.source = false;
						c.t = 0.f;
						c.burnFor = c.fuel == NOTHING ? 0.f : r.burnTime * (.5f + .5f * c.amount) * burnK;
					}
				}
				else if (c.t >= c.burnFor)
				{
					c.state = DYING;
					++dying; // firerework
					c.t = 0.f;
					changed.push_back(i);
				}
				break;
			case DYING:
				c.t += dt;
				c.size = full * (1.f - c.t / r.dieTime);
				if (c.size < 0.f) c.size = 0.f;
				if (c.t >= r.dieTime)
				{
					c.state = BURNT;
					c.size = 0.f;
					--burning;
					if (dying) --dying; // firerework
					++burnt;
					changed.push_back(i);
					gone = true;
				}
				break;
			default:
				gone = true;
				break;
			}
			if (gone)
			{
				active[a] = active.back();
				active.pop_back();
			}
			else
				++a;
		}
	}

	static float catchRamp(Fuel f)
	{
		const float t = .45f + .22f * rule(f).catchTime;
		return t > 2.f ? 2.f : t;
	}
	// how much heat a cell gives off, 0..1
	static float power(const Cell& c, const Weather& wx)
	{
		const float rain = c.roofed ? 1.f : 1.f - .4f * wx.rain;
		switch (c.state)
		{
		case CATCHING: return rain * (.25f + .35f * c.t / catchRamp(c.fuel));
		case BURNING: return rain * (c.source ? 1.f : .5f + .5f * c.amount);
		case DYING:
		{
			// firerework: dying lasts longer, its heat falls off squared - about as much heat in all as before
			const float k = 1.f - c.t / rule(c.fuel).dieTime;
			return .5f * rain * (k > 0.f ? k * k : 0.f);
		}
		default: return 0.f;
		}
	}

private:
	enum : uint32_t { NONE = 0xffffffffu };
	std::unordered_map<uint64_t, uint32_t> index;
	std::vector<uint32_t> active, warm, toProbe;
	size_t probeAt = 0;

	int lattice(float v) const { return int(std::floor(v / cellSize + .5f)); }
	static int storey(float y) { return int(std::floor(y / 1.2f + .5f)); }
	static uint64_t key(Kind kind, int kx, int ky, int kz, int face)
	{
		return uint64_t(uint32_t(kx + 0x20000) & 0x3ffffu) | (uint64_t(uint32_t(ky + 0x2000) & 0x3fffu) << 18) |
			(uint64_t(uint32_t(kz + 0x20000) & 0x3ffffu) << 32) | (uint64_t(kind) << 50) | (uint64_t(face & 7) << 52);
	}
	void igniteSource(uint32_t i,const V3& origin,float seconds)
	{
		Cell& c = cells[i];
		if (c.state == COLD || c.state == BURNT)
		{
			if (c.state == BURNT) --burnt;
			++burning;
			active.push_back(i);
		}
		if (c.state == DYING && dying) --dying; // firerework
		c.state = BURNING;
		c.source = true;
		c.sourceFor = c.sourceFor > seconds ? c.sourceFor : seconds;
		c.heat = 1.f;
		c.t = 0.f;
		c.burnFor = 0.f;
		c.until = clock + spreadFor;
		c.ox = origin.x;
		c.oz = origin.z;
		if (!c.probed) toProbe.push_back(i);
		changed.push_back(i);
	}
	uint32_t make(uint64_t k, const Site& s, Kind kind, int kx, int ky, int kz)
	{
		const auto found = index.find(k);
		if (found != index.end()) return found->second;
		if (cells.size() >= maxCells) return NONE;
		Cell c;
		c.p = s.p;
		c.n = s.n;
		c.kind = kind;
		c.fuel = s.fuel;
		c.amount = s.amount < 0.f ? 0.f : s.amount > 1.f ? 1.f : s.amount;
		c.roofed = s.roofed;
		c.trunk = s.trunk;
		c.lid = s.lid;
		c.woodFace = s.woodFace;
		c.kx = kx;
		c.ky = ky;
		c.kz = kz;
		cells.push_back(c);
		const uint32_t i = uint32_t(cells.size() - 1);
		index[k] = i;
		return i;
	}
	// ground and ceiling cells stand on the lattice; wall cells are where the wall was met
	uint32_t groundCell(Site s, int kx, int kz)
	{
        // The lattice indexes the cell; the volume stays at the actual support contact.
		return make(key(GROUND, kx, storey(s.p.y), kz, 0), s, GROUND, kx, storey(s.p.y), kz);
	}
	uint32_t ceilingCell(Site s, int kx, int kz)
	{
		s.p.x = float(kx) * cellSize;
		s.p.z = float(kz) * cellSize;
		return make(key(CEILING, kx, storey(s.p.y), kz, 0), s, CEILING, kx, storey(s.p.y), kz);
	}
	uint32_t wallCell(const Site& s)
	{
		const int face = std::fabs(s.n.x) > std::fabs(s.n.z) ? (s.n.x > 0.f ? 0 : 1) : (s.n.z > 0.f ? 2 : 3);
		const int kx = lattice(s.p.x), ky = lattice(s.p.y), kz = lattice(s.p.z);
        const uint64_t base=key(WALL,kx,ky,kz,face);
        for(uint32_t slot=0;slot<256;++slot){
            const uint64_t id=base|(uint64_t(slot)<<55);
            const auto it=index.find(id);
            if(it==index.end())return make(id,s,WALL,kx,ky,kz);
            const auto& c=cells[it->second];
            if(c.woodFace==s.woodFace && c.woodFace>=0)return it->second;
            if(c.trunk!=s.trunk || c.fuel!=s.fuel)continue;
            const float normalDot=c.n.x*s.n.x+c.n.z*s.n.z;
            const float depth=(c.p.x-s.p.x)*s.n.x+(c.p.z-s.p.z)*s.n.z;
            if(normalDot>=.65f && std::fabs(depth)<=(s.trunk?.09f:.018f))return it->second;
        }
        return NONE;
	}
	void link(uint32_t from, uint32_t to, float k, float dx, float dz,bool directWood=false)
	{
		if (to == NONE || to == from) return;
		for (Link& l : cells[from].links)
            if (l.to == to){
                if(directWood && k>l.k){l.k=k;l.dx=dx;l.dz=dz;}
                return;
            }
		const Link l = {to, k, dx, dz};
		cells[from].links.push_back(l);
	}
	void probe(World& w, uint32_t i)
	{
		switch (cells[i].kind)
		{
		case GROUND: probeGround(w, i); break;
		case WALL: probeWall(w, i); break;
		default: probeCeiling(w, i); break;
		}
		cells[i].probed = true;
	}
	void probeGround(World& w, uint32_t i)
	{
		static const int D[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
		const float S = cellSize;
		const V3 p = cells[i].p;
		const int kx = cells[i].kx, kz = cells[i].kz;
		Site s;
		for (int d = 0; d < 8; ++d)
		{
			const float len = S * std::sqrt(float(D[d][0] * D[d][0] + D[d][1] * D[d][1]));
			const V3 dir = {float(D[d][0]) * S / len, 0.f, float(D[d][1]) * S / len};
			const V3 from = {p.x, p.y + .6f, p.z};
			if (w.wall(from, dir, len, s))
			{
				link(i, wallCell(s), 1.f, dir.x, dir.z); // the flames stand against it; a fire does not go through a wall
				continue;
			}
			const V3 at = {float(kx + D[d][0]) * S, p.y, float(kz + D[d][1]) * S};
			if (!w.ground(at, 1.3f, 1.6f, s)) continue; // a drop, or nothing there
			const float rise = (s.p.y - p.y) / len;   // uphill a fire is faster, downhill slower
			float slope = 1.f + 1.5f * rise;
			slope = slope < .5f ? .5f : slope > 2.5f ? 2.5f : slope;
			link(i, groundCell(s, kx + D[d][0], kz + D[d][1]), slope * S / len, dir.x, dir.z);
		}
		// a tree's trunk in or beside this cell: the eight rays above miss a thin trunk, so the level is asked for one
		// (never a tree that stands on the ground over a room: its foot can reach down past the room's ceiling)
		if (w.trunk(p, S * .88f, .40f * S, .80f * S, s) && !(cells[i].lid != 0.f && s.p.y > cells[i].lid - .1f)){
            const float dx=s.p.x-p.x,dz=s.p.z-p.z;
            const float gap=std::sqrt(dx*dx+dz*dz)/S;
            link(i,wallCell(s),3.8f-1.4f*gap,0.f,0.f,true);
        }
		const V3 from = {p.x, p.y + .3f, p.z};
		if (w.ceiling(from, 3.4f, s))
		{
			float k = 1.6f - .45f * (s.p.y - p.y); // a low ceiling catches from the flames under it
			link(i, ceilingCell(s, kx, kz), k < .15f ? .15f : k > 1.2f ? 1.2f : k, 0.f, 0.f);
		}
	}
	void probeWall(World& w, uint32_t i)
	{
		const float S = cellSize;
		const V3 p = cells[i].p, n = cells[i].n;
		const bool isTrunk = cells[i].trunk;
		const V3 t = {-n.z, 0.f, n.x}; // along the wall
		const V3 into = {-n.x, 0.f, -n.z};
		// up, up and aside, aside, down: a fire climbs much faster than it creeps along or down
		static const float O[6][3] = {{0.f, 1.f, 3.f}, {1.f, 1.f, 1.2f}, {-1.f, 1.f, 1.2f}, {1.f, 0.f, 1.f}, {-1.f, 0.f, 1.f}, {0.f, -1.f, .35f}};
		Site s;
		// A wall's fire stays in its room. With a ceiling over it within a cell's height - the room's own ceiling, the floor
		// of the storey above, the ground over a cellar - it does not climb on: the wall above that is another room's, or
		// stands in the open on the ground overhead. With a floor right under it, it does not creep down into the room
		// underneath. (The ceiling is looked for over the air in front of the wall, where its flames stand, a little to
		// either side: in a corner one of the two places lies in the neighbouring wall. A ledge that juts out less than
		// that is none, and a tree's crown or a branch is none. A wall in the open, a fence, a house's front, a trunk burn
		// up as before.)
		const V3 foot = {p.x + n.x * .5f, p.y, p.z + n.z * .5f};
		Site below;
		const bool stands = w.ground(foot, .2f, 1.4f, below);
		bool lid = false;
		if (!isTrunk)
			for (int side = -1; side <= 1 && !lid; side += 2)
			{
				const V3 air = {p.x + n.x * .45f + t.x * .3f * float(side), p.y, p.z + n.z * .45f + t.z * .3f * float(side)};
				lid = w.ceiling(air, S + .1f, s) && s.fuel != BRUSH && !s.trunk;
			}
        auto connectedWood=[&](const Site& target){
            if(cells[i].fuel!=WOOD||target.fuel!=WOOD||cells[i].woodFace==target.woodFace)return true;
            const float dy=target.p.y-p.y;
            if(std::fabs(dy)<S*.5f)return true;
            for(int j=1;j<=3;++j){
                const float t=.25f*float(j);
                const V3 mid={p.x+(target.p.x-p.x)*t+n.x*.32f,p.y+dy*t,p.z+(target.p.z-p.z)*t+n.z*.32f};
                Site witness;
                if(!w.wall(mid,into,.68f,witness)||witness.fuel!=WOOD){
                    const V3 axis={mid.x-n.x*.32f,mid.y,mid.z-n.z*.32f};
                    if(!w.trunk(axis,.32f,-.035f,.035f,witness))return false;
                }
            }
            return true;
        };
		for (int o = 0; o < 6; ++o)
		{
			const V3 from = {p.x + n.x * .45f + t.x * O[o][0] * S, p.y + O[o][1] * S, p.z + n.z * .45f + t.z * O[o][0] * S};
			const bool shut = !isTrunk && (O[o][1] > 0.f ? lid : O[o][1] < 0.f && stands);
			if (!shut && w.wall(from, into, .95f, s)){
                if(connectedWood(s))link(i,wallCell(s),O[o][2],t.x*O[o][0],t.z*O[o][0]);
            }
			else if (o == 0)
			{
				// the top of the wall or an opening: whatever is overhead in front of it catches (the eave, the ceiling)
				const V3 f2 = {p.x + n.x * .45f, p.y, p.z + n.z * .45f};
				if (w.ceiling(f2, S + 1.2f, s)) link(i, ceilingCell(s, lattice(f2.x), lattice(f2.z)), 2.f, 0.f, 0.f);
			}
		}
		// firesides (2026-11-24): a thin wooden wall in the open burns through - its other face catches from this one
		// (a wall cell is one face; a fence burnt on one side only). Stone, thick walls, trunks, walls under a roof: as before.
		if (!isTrunk && cells[i].fuel == WOOD)
		{
			Site back;
			back.fuel = NOTHING; back.amount = 0.f; back.roofed = false; back.trunk = false;
			const int found = w.thinBack(p, n, back);
			if (found == 2)
			{
				// a board with one face: its back is the same board
				back.fuel = cells[i].fuel; back.amount = cells[i].amount; back.roofed = cells[i].roofed;
				back.lid = cells[i].lid; back.trunk = false; back.woodFace = cells[i].woodFace;
			}
			if (found) link(i, wallCell(back), 4.f, 0.f, 0.f);
		}
		if (isTrunk)
		{
			// A trunk leans and thins, and a ray from in front of it misses it a cell higher: the fire follows the trunk
			// itself up, and what grows overhead - the crown - catches from it.
			const V3 higher = {p.x - n.x * .2f, p.y + S * .75f, p.z - n.z * .2f};
			if (w.trunk(higher, 1.1f, 0.f, S * .6f, s) && connectedWood(s)) link(i, wallCell(s), 3.f, 0.f, 0.f);
			const V3 under = {p.x + n.x * .45f, p.y, p.z + n.z * .45f};
			if (w.ceiling(under, 5.f, s)) link(i, ceilingCell(s, lattice(under.x), lattice(under.z)), 1.5f, 0.f, 0.f);
		}
		// burning bits fall to its foot
		if (stands)
		{
			const int kx = lattice(foot.x), kz = lattice(foot.z);
			// only a ground cell that stands in front of this wall, never one behind it
			if ((float(kx) * S - p.x) * n.x + (float(kz) * S - p.z) * n.z > .05f) link(i, groundCell(below, kx, kz), .5f, 0.f, 0.f);
		}
	}
	void probeCeiling(World& w, uint32_t i)
	{
		static const int D[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
		const float S = cellSize;
		const V3 p = cells[i].p;
		const int kx = cells[i].kx, kz = cells[i].kz;
		const V3 under = {p.x, p.y - .45f, p.z};
		Site s;
		for (int d = 0; d < 8; ++d)
		{
			const float len = S * std::sqrt(float(D[d][0] * D[d][0] + D[d][1] * D[d][1]));
			const V3 dir = {float(D[d][0]) * S / len, 0.f, float(D[d][1]) * S / len};
			if (w.wall(under, dir, len, s))
			{
				link(i, wallCell(s), .6f, dir.x, dir.z); // down a wall it goes slowly
				continue;
			}
			const V3 at = {float(kx + D[d][0]) * S, under.y, float(kz + D[d][1]) * S};
			if (w.ceiling(at, 1.3f, s)) link(i, ceilingCell(s, kx + D[d][0], kz + D[d][1]), 1.5f * S / len, dir.x, dir.z); // flames roll under it
		}
		if (w.ground(under, 0.f, 6.f, s)) link(i, groundCell(s, kx, kz), .3f, 0.f, 0.f); // burning bits fall
	}
};
} // namespace PainterFireSpread
