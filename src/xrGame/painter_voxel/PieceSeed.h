#pragma once
struct PieceSeed {
 GoreControls gore; // Frozen policy of the committed cut that created this owner.
 std::vector<std::uint32_t> components; // Exact raw components carried by a composite semantic owner; empty means ordinary single component.
 std::shared_ptr<const Asset> asset;
 Stamp parent;                                   // accepted snapshot the piece was separated from
 std::shared_ptr<const Grid::FloorTiles> grenadeFloor; // transferred to this piece's initial grid
 std::shared_ptr<const Grid> grid;               // that snapshot's damage, cropped to the piece, with its ownership
 std::shared_ptr<const ::gfy_connectivity::Topology> topology; // exact remaining tissue at birth, including disconnected carried pristine islands
 std::shared_ptr<const ResolvedCutHistory> history;           // cuts at birth: their walls are in the birth mesh
 std::shared_ptr<const SeverState> sever;         // stations already torn
 // Other pieces' cells beside this piece's triangles (removed in its render field, so a triangle crossing a break
 // shows only this piece's side).
 std::vector<std::pair<Cell,core::Tile>> marks;
 std::vector<char> faces;                         // source faces this piece's skin shows (hit mapping)
 std::array<std::uint16_t,64> render_bone{};      // bone a new wound's skin weight uses on this piece
 std::uint16_t root_bone{65535};
 // Folded physical ownership (LiveSeparationWorker_impl.h admitNativeShell step 1): an element whose gram-scale remnant
 // got no rigid body of its own -> the adjacent element (adopter) that poses its tissue. Flattened along the chain and
 // inherited by every later seed of the lineage. A later separation of this piece treats the folded element's cells
 // and bone bindings as the adopter's (foldedMap / foldedFrames / foldedBindings), so the element never comes back as a
 // part the piece's shell has no body for. 65535 = not folded.
 std::array<std::uint16_t,64> folded=[]{std::array<std::uint16_t,64> a{};a.fill(65535);return a;}();
 // bodyjitter3: what this piece's shell holds. fitted_elements: the bones whose element is a cut fit (not a native shape).
 // trims: the cut-backs of its native elements ({bone, {n.x,n.y,n.z,d}}, bone frame). A later split of the piece keeps every
 // element as it is and adds its own cut-back to an element it divides.
 std::uint64_t fitted_elements{};
 std::vector<std::pair<std::uint16_t,std::array<double,4>>> trims;
};
