#pragma once
// Explicit source-child ownership for each native damaged body piece.
#include "RuntimePackedMesh.h"
#include <cstdint>
#include <array>
#include <stdexcept>
#include <vector>

namespace part_child_ownership {

// Caller-reviewed metadata for every ORIGINAL child. A coat spanning multiple
// body regions must be SurfaceRequiresPartition, not WholeAccessory.
enum class Role : std::uint8_t { SurfaceRequiresPartition, WholeAccessory };
enum class Action : std::uint8_t { ReplaceWithPacked, RetainOriginalAccessory, RemoveOriginal };
struct Decision { std::uint16_t original_child; Action action; };

inline void require(bool condition, const char* reason) {
    if (!condition) throw std::invalid_argument(reason);
}

// Engine runtime builds without exception unwinding. Validate every intentional
// refusal before calling the value-building helper below; only allocation can
// then fail, following the engine's ordinary allocation policy.
inline const char* validate_part_packets(
    const std::vector<gfy_packed::Mesh>& packed_parts,
    const std::vector<Role>& original_roles,
    const std::vector<std::vector<Decision>>& decisions_by_part) noexcept {
    const auto count=original_roles.size();
    if(!count||count>=gfy_packed::max_child_slots)return "original_child_count";
    if(packed_parts.empty()||packed_parts.size()!=decisions_by_part.size())
        return "part_plan_count";
    std::array<unsigned,gfy_packed::max_child_slots> accessory_owners{};
    for(auto role:original_roles)
        if(role!=Role::SurfaceRequiresPartition&&role!=Role::WholeAccessory)
            return "invalid_child_role";
    for(std::size_t part=0;part<packed_parts.size();++part){
        std::array<const gfy_packed::Child*,gfy_packed::max_child_slots> original{};
        std::array<bool,gfy_packed::max_child_slots> occupied{},seen{};
        std::size_t highest=count-1;
        for(const auto& child:packed_parts[part].children){
            const auto slot=child.child_index;
            if(slot>=gfy_packed::max_child_slots)return "packed_slot_range";
            if(child.material_source_child>=count)return "material_source_range";
            if(occupied[slot])return "duplicate_packed_slot";
            occupied[slot]=true;
            if(slot<count){
                if(child.material!=0||child.material_source_child!=slot)
                    return "original_slot_provenance";
                if(child.vertices.empty()&&child.indices.empty()&&
                   !child.original_faces.empty())return "empty_removal_provenance";
                original[slot]=&child;
            }else{
                if(child.vertices.empty()||child.indices.empty())
                    return "empty_appended_slot";
                if(slot>highest)highest=slot;
            }
        }
        for(std::size_t slot=count;slot<=highest;++slot)
            if(!occupied[slot])return "appended_slot_gap";
        const auto& plan=decisions_by_part[part];
        if(plan.size()!=count)return "missing_original_decision";
        for(const auto& decision:plan){
            const auto slot=decision.original_child;
            if(slot>=count)return "decision_slot_range";
            if(seen[slot])return "duplicate_original_decision";
            seen[slot]=true;
            const auto* child=original[slot];
            switch(decision.action){
            case Action::ReplaceWithPacked:
                if(!child||child->vertices.empty()||child->indices.empty())
                    return "replacement_geometry_missing";
                if(original_roles[slot]==Role::WholeAccessory&&
                   ++accessory_owners[slot]>1)return "accessory_owned_twice";
                break;
            case Action::RetainOriginalAccessory:
                if(original_roles[slot]!=Role::WholeAccessory)
                    return "surface_requires_partition";
                if(child)return "retained_child_also_packed";
                if(++accessory_owners[slot]>1)return "accessory_owned_twice";
                break;
            case Action::RemoveOriginal:
                if(child&&(!child->vertices.empty()||!child->indices.empty()))
                    return "removed_child_has_geometry";
                break;
            default:return "invalid_child_action";
            }
        }
        for(std::size_t slot=0;slot<count;++slot)
            if(!seen[slot])return "missing_original_decision";
    }
    return nullptr;
}

// Each packed input is already partitioned for ONE part. Every original child
// gets an explicit action in every part. Existing packed entries, including
// appended cavity slots, are copied byte-for-byte as value objects. Remove
// actions append an empty original Child, which RuntimePackedBridge converts to
// remove_surface=true. A whole accessory may have only one owner, whether
// intact or replaced by packed geometry. On refusal inputs remain untouched.
// piececommit: the output starts as the parts (copied; or, handed over, each mesh moved - the caller's vector keeps its size).
inline std::vector<gfy_packed::Mesh> part_packets_start(const std::vector<gfy_packed::Mesh>& parts) {return parts;}
inline std::vector<gfy_packed::Mesh> part_packets_start(std::vector<gfy_packed::Mesh>&& parts) {
    std::vector<gfy_packed::Mesh> out;out.reserve(parts.size());
    for (auto& mesh : parts) out.push_back(std::move(mesh));
    return out;
}
template <class Parts>
inline std::vector<gfy_packed::Mesh> build_part_packets_from(
    Parts&& packed_parts,
    const std::vector<Role>& original_roles,
    const std::vector<std::vector<Decision>>& decisions_by_part)
{
    const auto original_count = original_roles.size();
    require(original_count > 0 && original_count < gfy_packed::max_child_slots,
            "original_child_count");
    for (auto role : original_roles)
        require(role == Role::SurfaceRequiresPartition || role == Role::WholeAccessory,
                "invalid_child_role");
    require(!packed_parts.empty() && packed_parts.size() == decisions_by_part.size(),
            "part_plan_count");
    std::vector<unsigned> accessory_owners(original_count, 0);
    std::vector<std::vector<bool>> add_removal;
    add_removal.reserve(packed_parts.size());

    // Validate every part and cross-part ownership before constructing output.
    for (std::size_t part = 0; part < packed_parts.size(); ++part) {
        std::vector<const gfy_packed::Child*> original(original_count, nullptr);
        std::vector<bool> occupied(gfy_packed::max_child_slots, false);
        std::size_t highest = original_count - 1;
        for (const auto& child : packed_parts[part].children) {
            require(child.child_index < gfy_packed::max_child_slots, "packed_slot_range");
            require(child.material_source_child < original_count, "material_source_range");
            require(!occupied[child.child_index], "duplicate_packed_slot");
            occupied[child.child_index] = true;
            if (child.child_index < original_count) {
                require(child.material == 0 && child.material_source_child == child.child_index,
                        "original_slot_provenance");
                if (child.vertices.empty() && child.indices.empty())
                    require(child.original_faces.empty(), "empty_removal_provenance");
                original[child.child_index] = &child;
            } else {
                require(!child.vertices.empty() && !child.indices.empty(),
                        "empty_appended_slot");
                if (child.child_index > highest) highest = child.child_index;
            }
        }
        for (std::size_t slot = original_count; slot <= highest; ++slot)
            require(occupied[slot], "appended_slot_gap");

        const auto& plan = decisions_by_part[part];
        require(plan.size() == original_count, "missing_original_decision");
        std::vector<bool> seen(original_count, false), removal(original_count, false);
        for (const auto& decision : plan) {
            require(decision.original_child < original_count, "decision_slot_range");
            const auto slot = decision.original_child;
            require(!seen[slot], "duplicate_original_decision");
            seen[slot] = true;
            const auto* child = original[slot];
            switch (decision.action) {
            case Action::ReplaceWithPacked:
                require(child && !child->vertices.empty() && !child->indices.empty(),
                        "replacement_geometry_missing");
                if (original_roles[slot] == Role::WholeAccessory)
                    require(++accessory_owners[slot] <= 1, "accessory_owned_twice");
                break;
            case Action::RetainOriginalAccessory:
                require(original_roles[slot] == Role::WholeAccessory,
                        "surface_requires_partition");
                require(!child, "retained_child_also_packed");
                require(++accessory_owners[slot] <= 1, "accessory_owned_twice");
                break;
            case Action::RemoveOriginal:
                require(!child || (child->vertices.empty() && child->indices.empty()),
                        "removed_child_has_geometry");
                removal[slot] = !child;
                break;
            default:
                require(false, "invalid_child_action");
            }
        }
        for (bool present : seen) require(present, "missing_original_decision");
        add_removal.push_back(std::move(removal));
    }

    std::vector<gfy_packed::Mesh> output = part_packets_start(std::forward<Parts>(packed_parts)); // piececommit
    for (std::size_t part = 0; part < output.size(); ++part)
        for (std::size_t slot = 0; slot < original_count; ++slot)
            if (add_removal[part][slot]) {
                gfy_packed::Child empty;
                empty.child_index = static_cast<std::uint16_t>(slot);
                empty.material_source_child = static_cast<std::uint16_t>(slot);
                output[part].children.push_back(std::move(empty));
            }
    return output;
}
inline std::vector<gfy_packed::Mesh> build_part_packets(
    const std::vector<gfy_packed::Mesh>& packed_parts,
    const std::vector<Role>& original_roles,
    const std::vector<std::vector<Decision>>& decisions_by_part)
{return build_part_packets_from(packed_parts, original_roles, decisions_by_part);}
inline std::vector<gfy_packed::Mesh> build_part_packets( // piececommit: parts handed over
    std::vector<gfy_packed::Mesh>&& packed_parts,
    const std::vector<Role>& original_roles,
    const std::vector<std::vector<Decision>>& decisions_by_part)
{return build_part_packets_from(std::move(packed_parts), original_roles, decisions_by_part);}
} // namespace part_child_ownership
