## TDALT — FIFO Compliance Audit

**Date:** 2026-07-14  
**Scope:** Time-Dependent ALT (`TimeDependentBidirALT`) on `/route` with `date_time.type` = `current`, `depart_at`, or `arrive_by`.

TDALT runs a **forward** search on the time-dependent graph `G` (live `TimeInfo`, full `EdgeCost`) and a **backward** search on the lower-bound graph `G_λ` (static `λ`, `TimeInfo::invalid()`). The three-phase meet–bound–forward procedure is correct only when edge costs satisfy the **FIFO property**:

```
c(u,v,τ) + τ ≤ c(u,v,τ′) + τ′   for all τ < τ′
```

Equivalently, for an allowed edge, travel time `c(u,v,τ)` must be **non-decreasing** in departure time `τ`. Without FIFO the time-dependent shortest-path problem is NP-hard and bidirectional pruning is unsound.

`G_λ` uses `LowerBoundCost::seconds()` — `λ = length / max(edge speed)` — ignoring traffic, time restrictions, and `truck_ban` penalties. TDALT requires `λ(u,v) ≤ c(u,v,τ)` for all `τ` (verified by unit tests) **and** FIFO on the forward `c(u,v,τ)` used during search.

---

### Traffic profiles

`GraphTile::GetSpeed()` resolves speed at query time (`valhalla/baldr/graphtile.h`):

| Source | When used | FIFO note |
|--------|-----------|-----------|
| **Current (live)** | `flow_mask` includes `current`; valid live tile data | Speed is fixed for a given query instant; along-route fade blends live into historical over one hour (`live_traffic_multiplier`). |
| **Predicted** | `flow_mask` includes `predicted`; `TimeInfo.second_of_week` set | Weekly profile in 5-minute buckets (`predictedspeeds_.speed`). Constant within a bucket → constant travel time. |
| **Constrained / free-flow** | Fallback when predicted unavailable; daytime 07:00–19:00 vs night | Step change at day/night boundary when both sources apply. |
| **Base speed** | Final fallback (`DirectedEdge::speed()`) | Static — FIFO trivially holds. |

For **auto**, **truck**, and **truck_ban**, `EdgeCost()` computes `secs = length / speed` (plus static penalties independent of `τ`). When an edge is **allowed** at both departure times, FIFO reduces to **speed non-increasing in `τ`**. Predicted profiles are historical averages; constrained/free-flow are single scalar values per edge. Live traffic is point-in-time. Discontinuities can occur at 5-minute bucket boundaries and at the constrained/free-flow day/night switch — the same model used by existing `TimeDepForward` / `TimeDepReverse`.

---

### Time restrictions

Complex access restrictions (`kTimedAllowed`, `kTimedDenied`, `kDestinationAllowed`) are evaluated in `DynamicCost::EvaluateRestrictions()` via `IsConditionalActive()` when `current_time ≠ 0`. Outside an active window the edge is **disallowed** (`Allowed()` returns false), not assigned a finite penalty in `EdgeCost()`.

Valhalla does **not** model waiting at nodes: a later departure can change whether an edge is traversable. That is the same access-gating behaviour as unidirectional time-dependent routing today. For edges **allowed** at both `τ` and `τ′`, traversal time depends only on traffic speed, not on the restriction schedule.

---

### `truck_ban` windows

`Costing::truck_ban` (`TruckCost` with `enforce_eu_truck_bans_`) calls `truck_ban::IsTraverseAllowed()` when `current_time ≠ 0`, checking both endpoint nodes against country rules in `truck_ban_rules.cc`:

- **Germany (DE):** Sunday 00:00–22:00 and public holidays (all roads); July–August Saturday 07:00–20:00 on motorways/trunk.
- **Austria (AT):** Saturday 15:00–24:00, Sunday 00:00–22:00, public holidays, and nightly 22:00–05:00 (all roads).

Bans gate **access** (edge excluded from expansion), not `EdgeCost()` seconds. **Forward tree:** live `TimeInfo`, full ban checks. **Backward `G_λ`:** `λ` excludes bans — a safe lower bound because `λ ≤ c` when forward cost is effectively infinite. Ban window boundaries change legality by departure/arrival time, analogous to timed access restrictions.

---

### Conclusion

FIFO **holds for TDALT-supported configurations**:

| Dimension | Supported |
|-----------|-----------|
| Costing modes | `auto`, `truck`, `truck_ban` |
| `date_time` types | `current`, `depart_at`, `arrive_by` |

Forward traversal time on **allowed** edges is non-decreasing in departure time under the same traffic and costing model as unidirectional TD. `λ` on `G_λ` lower-bounds `c(τ)` for all `τ`. TDALT reuses that cost stack; correctness is aligned with `TimeDepForward` / `TimeDepReverse`.

---

### Exclusions

| Item | Status |
|------|--------|
| **`invariant` (`date_time.type = 3`)** | **Rejected** at request parse (`worker.cc`, HTTP 400). Not routed by TDALT or any TD algorithm. |
| **Other costing modes** | `pedestrian`, `bicycle`, `motorcycle`, `transit`, multimodal — outside TDALT v1 scope; not audited here. |
| **Access/ban window boundaries** | Edges can become allowed when departure time shifts into an open window. Formal FIFO over disallowed→allowed transitions does not hold; mitigated by the same no-waiting TD model Valhalla already uses. |
| **Traffic bucket / day-night switches** | Step changes in `GetSpeed()` at 5-minute and 07:00/19:00 boundaries; same as existing TD routing. |

No additional edge-class exclusions are required for TDALT rollout beyond the `invariant` rejection and the non-vehicle modes listed above.
