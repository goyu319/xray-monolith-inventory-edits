#pragma once
class CPhysicsShell;
class CEntityAlive;
class CPHIsland;
struct SHit;
extern int painter_balance_test_enabled;
extern int painter_balance_test_active;
extern int painter_balance_test_inferences;
bool painter_balance_test_configure(bool enabled);
class PainterBalanceTest {
public:
    PainterBalanceTest();
    ~PainterBalanceTest();
    bool start(CPhysicsShell* shell,CEntityAlive* owner);
    void stop();
    void hit(const SHit& hit);
    void step(CPHIsland* island,bool after,bool exact);
private:
    struct Impl;
    Impl* p_;
};
