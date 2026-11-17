// Included by script_wallmarks_manager.cpp so existing project variants share
// this implementation. Do not also add it as a separate compiled source.
#include "BloodOverlap.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <stdexcept>
#include <vector>
#include "../3rd party/luajit-2/src/lua.hpp"

// The Lua geometry uses binary64 operations in source order. In particular,
// contractions/reassociation can change a tangency or the first witness.
#if defined(_MSC_VER)
#pragma float_control(precise, on, push)
#pragma fp_contract(off)
#endif

namespace blood_overlap_detail
{
constexpr double plane = .004;
constexpr const char* shape_metatable = "blood_overlap.shape.v1";
struct Point { double x, y, z; };
struct Disk { double x, y, z, r; };
struct Shape
{
    Point position{}, normal{}, minimum{}, maximum{};
    std::vector<Disk> disks;
};
struct ShapeHandle
{
    Shape* shape = nullptr;
    alignas(Shape) unsigned char storage[sizeof(Shape)];
};
struct Anchor { Point position{}, normal{}; double radius = 0; };

double dot(const Point& a, const Point& b)
{
    return a.x*b.x+a.y*b.y+a.z*b.z;
}
bool basis(const Point& n, Point& right, Point& up)
{
    right = std::abs(n.y)>.99 ? Point{0,n.z,-n.y} : Point{-n.z,0,n.x};
    const double length = std::sqrt(dot(right,right));
    if (!std::isfinite(length) || std::abs(length)>=1e8 || length<=1e-8)
        return false;
    right = {right.x/length,right.y/length,right.z/length};
    up = {right.y*n.z-right.z*n.y,right.z*n.x-right.x*n.z,right.x*n.y-right.y*n.x};
    return true;
}
bool bounds_contain(const Shape& m, const Point& p)
{
    return p.x>=m.minimum.x && p.x<=m.maximum.x
        && p.y>=m.minimum.y && p.y<=m.maximum.y
        && p.z>=m.minimum.z && p.z<=m.maximum.z;
}
bool bounds_overlap(const Shape& a, const Shape& b)
{
    return a.minimum.x<=b.maximum.x && b.minimum.x<=a.maximum.x
        && a.minimum.y<=b.maximum.y && b.minimum.y<=a.maximum.y
        && a.minimum.z<=b.maximum.z && b.minimum.z<=a.maximum.z;
}
double contains(const Shape& m, const Point& p)
{
    if (!bounds_contain(m,p)) return 0;
    const Point& n = m.normal;
    if (std::abs((p.x-m.position.x)*n.x+(p.y-m.position.y)*n.y+(p.z-m.position.z)*n.z)>plane)
        return 0;
    double room = 0;
    for (const Disk& d : m.disks)
    {
        const double x=p.x-d.x, y=p.y-d.y, z=p.z-d.z;
        const double depth=x*n.x+y*n.y+z*n.z;
        const double distance=std::sqrt((std::max)(0.0,x*x+y*y+z*z-depth*depth));
        const double r=d.r-distance;
        if (r>0 && r>room) room=r;
    }
    return room;
}
bool overlaps(const Shape& a, const Shape& b)
{
    if (!bounds_overlap(a,b)) return false;
    for (const Disk& p : a.disks) for (const Disk& q : b.disks)
    {
        const double x=p.x-q.x, y=p.y-q.y, z=p.z-q.z;
        if (x*x+y*y+z*z<std::pow(p.r+q.r,2.0)) return true;
    }
    return false;
}
bool common(const std::vector<const Shape*>& records, double& work, Anchor& anchor)
{
    if (records.empty()) return false;
    const Shape& first=*records[0];
    Point right{},up{};
    if (!basis(first.normal,right,up)) return false;
    // Every witness still debits the complete immutable disk count, including
    // rejected witnesses and the second sign after the first exhausts work.
    double cost=0;
    for (const Shape* m : records) cost+=static_cast<double>(m->disks.size());
    const auto witness = [&](const Point& point)
    {
        work=work-cost;
        if (work<0) return false;
        double radius=std::numeric_limits<double>::infinity();
        for (const Shape* m : records)
        {
            const double room=contains(*m,point);
            if (room<=.00001) return false;
            radius=(std::min)(radius,room);
        }
        anchor={point,first.normal,radius*.8};
        return true;
    };
    for (const Shape* m : records) for (const Disk& d : m->disks)
    {
        if (work<0) return false;
        if (witness({d.x,d.y,d.z})) return true;
    }
    for (size_t i=0;i+1<records.size();++i) for (size_t j=i+1;j<records.size();++j)
    {
        for (const Disk& a : records[i]->disks) for (const Disk& b : records[j]->disks)
        {
            work=work-1;
            if (work<0) return false;
            const Point delta{b.x-a.x,b.y-a.y,b.z-a.z};
            const double x=dot(delta,right), y=dot(delta,up);
            const double distance=std::sqrt(x*x+y*y);
            const double ar=(std::max)(0.0,a.r-.00002), br=(std::max)(0.0,b.r-.00002);
            if (distance>1e-8 && distance<ar+br && distance>std::abs(ar-br))
            {
                const double along=(ar*ar-br*br+distance*distance)/(2*distance);
                const double height=std::sqrt((std::max)(0.0,ar*ar-along*along));
                for (int sign=-1;sign<=1;sign+=2)
                {
                    const double u=along*x/distance-sign*height*y/distance;
                    const double v=along*y/distance+sign*height*x/distance;
                    if (witness({a.x+right.x*u+up.x*v,a.y+right.y*u+up.y*v,a.z+right.z*u+up.z*v}))
                        return true;
                }
            }
        }
    }
    return false;
}

// Raw field reads never invoke a caller's metamethod. Userdata has its GC
// finalizer before any subsequent Lua allocation; even a Lua longjmp leaves
// an owned object rather than a leaked vector. No Lua references are retained.
int absolute(lua_State* L,int index) { return index>0 ? index : lua_gettop(L)+index+1; }
void field(lua_State* L,int index,const char* key)
{
    index=absolute(L,index);lua_pushstring(L,key);lua_rawget(L,index);
}
bool number_field(lua_State* L,int index,const char* key,double& value)
{
    field(L,index,key);
    const bool numeric=lua_type(L,-1)==LUA_TNUMBER;
    value=numeric ? lua_tonumber(L,-1) : 0;
    lua_pop(L,1);
    return numeric && std::isfinite(value);
}
bool point_field(lua_State* L,int index,const char* key,Point& point)
{
    field(L,index,key);
    const bool valid=lua_istable(L,-1) && number_field(L,-1,"x",point.x)
        && number_field(L,-1,"y",point.y) && number_field(L,-1,"z",point.z);
    lua_pop(L,1);return valid;
}
Shape* checked_shape(lua_State* L,int index)
{
    auto* handle=static_cast<ShapeHandle*>(luaL_checkudata(L,index,shape_metatable));
    if (!handle->shape) luaL_error(L,"blood_overlap: released shape");
    return handle->shape;
}
int collect(lua_State* L)
{
    auto* handle=static_cast<ShapeHandle*>(luaL_checkudata(L,1,shape_metatable));
    if (handle->shape) { handle->shape->~Shape();handle->shape=nullptr; }
    return 0;
}
int build(lua_State* L)
{
    if (!lua_istable(L,1)) { lua_pushnil(L);return 1; }
    void* memory=lua_newuserdata(L,sizeof(ShapeHandle));
    auto* handle=new(memory) ShapeHandle;
    luaL_getmetatable(L,shape_metatable);lua_setmetatable(L,-2);
    Shape* m=new(handle->storage) Shape;
    handle->shape=m;
    bool valid=point_field(L,1,"position",m->position) && point_field(L,1,"normal",m->normal)
        && number_field(L,1,"minx",m->minimum.x) && number_field(L,1,"miny",m->minimum.y)
        && number_field(L,1,"minz",m->minimum.z) && number_field(L,1,"maxx",m->maximum.x)
        && number_field(L,1,"maxy",m->maximum.y) && number_field(L,1,"maxz",m->maximum.z);
    if (valid)
    {
        field(L,1,"disks");
        valid=lua_istable(L,-1);
        if (valid)
        {
            try
            {
                const size_t count=lua_objlen(L,-1);
                m->disks.reserve(count);
                for (size_t i=0;valid && i<count;++i)
                {
                    lua_rawgeti(L,-1,static_cast<int>(i+1));
                    Disk d{};
                    valid=lua_istable(L,-1) && number_field(L,-1,"x",d.x)
                        && number_field(L,-1,"y",d.y) && number_field(L,-1,"z",d.z)
                        && number_field(L,-1,"r",d.r);
                    lua_pop(L,1);
                    if (valid) m->disks.push_back(d);
                }
            }
            catch (const std::bad_alloc&) { valid=false; }
            catch (const std::length_error&) { valid=false; }
        }
        lua_pop(L,1);
    }
    if (!valid || m->disks.empty()) { lua_pushnil(L);return 1; }
    return 1;
}
int contains_lua(lua_State* L)
{
    const Shape& m=*checked_shape(L,1);
    const Point p{luaL_checknumber(L,2),luaL_checknumber(L,3),luaL_checknumber(L,4)};
    const double room=contains(m,p);
    if (room>0) lua_pushnumber(L,room);else lua_pushnil(L);
    return 1;
}
int overlaps_lua(lua_State* L)
{
    lua_pushboolean(L,overlaps(*checked_shape(L,1),*checked_shape(L,2)));return 1;
}
void set_number(lua_State* L,const char* name,double value)
{
    lua_pushnumber(L,value);lua_setfield(L,-2,name);
}
int common_lua(lua_State* L)
{
    luaL_checktype(L,1,LUA_TTABLE);
    double work=luaL_checknumber(L,2);
    // Validate all userdata before a C++ owning temporary exists: invalid
    // callers may raise a Lua error, whose longjmp does not unwind C++ locals.
    const size_t count=lua_objlen(L,1);
    for (size_t i=0;i<count;++i)
    {
        lua_rawgeti(L,1,static_cast<int>(i+1));checked_shape(L,-1);lua_pop(L,1);
    }
    Anchor anchor{};bool found=false;
    try
    {
        std::vector<const Shape*> records;
        records.reserve(count);
        for (size_t i=0;i<count;++i)
        {
            lua_rawgeti(L,1,static_cast<int>(i+1));
            records.push_back(static_cast<const ShapeHandle*>(lua_touserdata(L,-1))->shape);lua_pop(L,1);
        }
        found=common(records,work,anchor);
    }
    catch (const std::bad_alloc&) { lua_pushnil(L);return 1; }
    catch (const std::length_error&) { lua_pushnil(L);return 1; }
    lua_pushnumber(L,work);
    if (!found) lua_pushnil(L);
    else
    {
        lua_createtable(L,0,7);
        set_number(L,"x",anchor.position.x);set_number(L,"y",anchor.position.y);set_number(L,"z",anchor.position.z);
        set_number(L,"radius",anchor.radius);
        set_number(L,"nx",anchor.normal.x);set_number(L,"ny",anchor.normal.y);set_number(L,"nz",anchor.normal.z);
    }
    return 2;
}
}

void RegisterBloodOverlap(lua_State* L)
{
    using namespace blood_overlap_detail;
    if (luaL_newmetatable(L,shape_metatable))
    {
        lua_pushcfunction(L,collect);lua_setfield(L,-2,"__gc");
        lua_pushliteral(L,"immutable blood overlap geometry");lua_setfield(L,-2,"__metatable");
    }
    lua_pop(L,1);
    lua_createtable(L,0,5);
    set_number(L,"version",1);
    lua_pushcfunction(L,build);lua_setfield(L,-2,"build");
    lua_pushcfunction(L,contains_lua);lua_setfield(L,-2,"contains");
    lua_pushcfunction(L,overlaps_lua);lua_setfield(L,-2,"overlaps");
    lua_pushcfunction(L,common_lua);lua_setfield(L,-2,"common");
    lua_setglobal(L,"blood_overlap");
}

#if defined(_MSC_VER)
#pragma float_control(pop)
#endif
