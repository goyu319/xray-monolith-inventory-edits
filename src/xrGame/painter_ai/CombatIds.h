#pragma once
#include <cstdint>

namespace painter { namespace combat { namespace ids {
// The native owning operator in the root CStalkerPlanner, pinned through painter_forced_action; its eligibility
// evaluator has the same id (CStalkerPlanner::painter_forced_update gates on it and on nothing else).
constexpr std::uint32_t action = 1900300701u;
constexpr std::uint32_t eligibility = action;
// 1900300702 - 1900300799: reserved for further native properties and operators. Allocate here, with a purpose:
//   (none yet)
constexpr std::uint32_t reserved_first = 1900300702u;
constexpr std::uint32_t reserved_last = 1900300799u;
}}} // namespace painter::combat::ids

namespace painter { namespace combat { namespace roles {
// Order::role (u16). The names are the Lua reference's roles (src/35_orders.lua, the room planner), so replay parity
// compares role ids one to one. Allocate here only; never renumber.
constexpr std::uint16_t none = 0;
constexpr std::uint16_t individual = 1;        // the first response / solo tree: his own fight
constexpr std::uint16_t base = 2;              // field pair: the one who holds and fires
constexpr std::uint16_t mover = 3;             // field pair: the one who bounds
constexpr std::uint16_t point = 10;            // stack slot 1
constexpr std::uint16_t hook = 11;             // slot 2, the opposite corner
constexpr std::uint16_t deep = 12;             // slot 3
constexpr std::uint16_t rear = 13;             // slot 4+, rear security
constexpr std::uint16_t breacher = 14;         // shotgun / frag carrier at the door
constexpr std::uint16_t entry_check_doors = 15;
constexpr std::uint16_t door_fight = 20;       // fighting a shooter through a doorway
constexpr std::uint16_t threat_overwatch = 21; // rifle held on a known threat's spot
constexpr std::uint16_t overwatch = 22;        // eyes on an uncleared doorway
constexpr std::uint16_t lane = 23;             // a sector that fights only what it sees
constexpr std::uint16_t portal_watch = 24;
constexpr std::uint16_t assess_pie = 30;
constexpr std::uint16_t assess_hold = 31;
constexpr std::uint16_t casualty_guard = 40;
constexpr std::uint16_t aid_casualty = 41;
constexpr std::uint16_t care_security = 42;
constexpr std::uint16_t cover_noise = 50;      // turned onto a sound nobody else covers
constexpr std::uint16_t hold = 60;
constexpr std::uint16_t search = 61;
constexpr std::uint16_t finisher = 62;         // finishing a wounded enemy from cleared space
// the Lua reference's recorded role vocabulary (replay-v2.1 roles.json; the recorder maps "base[_qualifier]" by the
// longest canonical base at a token boundary, so these are the bases):
constexpr std::uint16_t barricade_guard = 70;  // a defender holding his barricade sector
constexpr std::uint16_t door_stack = 71;       // a stack slot at a door
constexpr std::uint16_t door_stack_point = 72; // the stack's point man
constexpr std::uint16_t entry_tail = 73;       // last man of an entry
constexpr std::uint16_t entry_wait = 74;       // waiting his turn through the door
constexpr std::uint16_t entry_hook_hard_corner = 75;
constexpr std::uint16_t entry_deep = 76;
constexpr std::uint16_t entry_point_clear_side = 77;
constexpr std::uint16_t entry_point_cross = 78;
constexpr std::uint16_t door_post = 79;        // posted beside a doorway, rifle on it
constexpr std::uint16_t door_post_ready = 80;
constexpr std::uint16_t door_slice = 81;       // slicing the pie at a door
constexpr std::uint16_t door_fight_second = 82;
constexpr std::uint16_t base_of_fire = 83;     // field: the firing half of a pair
constexpr std::uint16_t seek_cover = 84;
constexpr std::uint16_t hold_cover = 85;
constexpr std::uint16_t cover_hit_dir = 86;    // cover from the direction of a hit
constexpr std::uint16_t under_fire_cover = 87;
constexpr std::uint16_t retreat_cover = 88;
constexpr std::uint16_t reload_cover = 89;
constexpr std::uint16_t rear_security = 90;
constexpr std::uint16_t dogfight = 91;         // his own close fight
constexpr std::uint16_t shoot = 92;
constexpr std::uint16_t travel = 93;
constexpr std::uint16_t buddy_answer = 94;     // answering a teammate's shooter
constexpr std::uint16_t suppress_buddy = 95;
constexpr std::uint16_t threat_overwatch_high = 96;
constexpr std::uint16_t search_red = 97;       // searching the red (unseen) space
constexpr std::uint16_t set = 98;              // a sector / portal / keyed set slot: qualifier "C_3", "P:L2_L4_a_1", "K:b6_1"
constexpr std::uint16_t search_sector = 99;    // "search_<sector>": qualifier = the sector (FR, FL, NL, FL2 ...)
// the flank layer's roles (rounds 9u / 9v), seen in the 19-defender killhouse recordings (replay-v2.1 killhouse19):
constexpr std::uint16_t flank_stack = 100;     // stacked at the second door for a joint entry
constexpr std::uint16_t scout_pass = 101;      // the scout moving to the flank door
constexpr std::uint16_t scout_pie = 102;       // the scout pieing the flank door
constexpr std::uint16_t two_up = 103;          // two men up on the same door for the entry
constexpr std::uint16_t suppression_push = 104; // squad-authorized advance under another member's actual suppression
constexpr std::uint16_t reserved_first = 110;  // 110..999 for roles allocated later, with a purpose
}}} // namespace painter::combat::roles

namespace painter { namespace combat { namespace events {
// Decision::Event::kind (u16): diagnostics only, mirrors the Lua reference's event names for the logs and the replay.
constexpr std::uint16_t none = 0, first_response = 1, cover_pick = 2, stack_formed = 3, entry_released = 4,
    room_cleared = 5, mission_handover = 6, squad_released = 7, lean = 8, suppress = 9, casualty = 10, hand_back = 11,
    search_loss_priced = 12, search_viewpoint = 13, reserved_first = 100;
}}} // namespace painter::combat::events

namespace painter { namespace combat { namespace compat {
// v2.21: the compatibility push model's fact domains (the `domain` argument of level.painter_combat_compat_fact and
// _invalidate, as these names; the engine maps the string to the code). A fact is a script-owned state about one of OUR men,
// never enemy knowledge. 1-5 mirror the Flag bits (the stable flags keep their set_flag path; a compat_fact for the same
// name updates the same stored flag); 6-9 are the dynamic domains the poll covered.
enum Domain : std::uint8_t {
    story = 1, companion = 2, script_combat = 3, dont_shoot = 4, not_for_help = 5,
    scripted_target = 6,       // a script gave him a target (axr_companions, squad task / mission writers)
    legacy_controller = 7,     // a Lua controller holds him (painter_ai_tactical / painter_ai_world, a third-party controller)
    help_claim = 8,            // the stock help-wounded selection / claim (selected_id, wounded_already_selected)
    grenade_request = 9,       // a legacy Lua grenade request owns his hands
    domain_reserved_first = 32,
};
constexpr const char* domain_names[] = { "", "story", "companion", "script_combat", "dont_shoot", "not_for_help",
    "scripted_target", "legacy_controller", "help_claim", "grenade_request" };
}}} // namespace painter::combat::compat
