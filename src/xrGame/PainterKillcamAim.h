#pragma once
class CActor;
class CWeapon;
class CGameObject;
template<class T> struct _vector3;
typedef _vector3<float> Fvector;
namespace painter_killcam_aim {
bool renew(CActor&, CWeapon&, CGameObject&);
void clear(CActor&);
void retire(const CGameObject&);
bool origin(CActor&, CWeapon&, Fvector&);
bool apply(CWeapon&, Fvector&, Fvector&);
}
