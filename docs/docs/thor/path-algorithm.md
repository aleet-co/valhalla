## Thor - Determining the Best Path

Two core components of the Valhalla open source routing engine are **Thor** and **Sif**. These 2 companions (in Norse mythology Thor and Sif are husband and wife) form the basis of Valhalla's path generation algorithm. Thor contains the path computation algorithms and traverses the routing tiles, while Sif performs **costing** that is central to forming the best path. Rather than baking costs into the routing graph data, Valhalla uses dynamic, run-time costing to generate costs based on a rich set of attributes stored in the routing graph tiles. This allows run-time generation of different types of routes (or routes with different characteristics) simply by using different costing methods and options within those methods.

#### Path Algorithm Introduction

Routing from one location to another is solved by a class of algorithms known as **shortest path algorithms**. This is somewhat misleading, as often one is interested in a route that is shortest time or one that makes fewer turns. A better term for shortest path algorithms is **least cost algorithms** - this properly indicates that the method is minimizing cost, be it distance, time, or some other metric. Naive assignment of cost to edges of the routing graph will lead to poor routing solutions. Simple costing based solely on distance or on time (based solely on speed) can lead to poor route paths with excessive turns and stops. Considerations such as turn types, classifications of roads at intersections along the route, road surface type, elevation change, road curvature, and a host of other considerations can be important.  It is also important to note that different costing considerations are needed for bicycle routing than pedestrian routing or automobile routing. Dynamic costing is described [here](../sif/dynamic-costing.md).

Valhalla uses several levels of road hierarchies to enhance performance. The lowest level hierarchy is called the local level. The local level includes all roads and paths that are routable (using various access methods). The next hierarchy level is called arterial. This level drops out all paths and residential roads. The highest level is called the highway level. This level includes just motorways, trunk roads, and primary roads - these are roads needed for long routes. By transitioning to the higher hierarchy levels as the route path moves away from the origin or destination the path finding algorithm considers less roads - improving performance. Also, shortcut edges are formed on the arterial and highway hierarchies. These edges bypass intersections that only connect to lower hierarchy edges. This allows several edges to be combined into one longer edge, which also improves performance. 

Thor uses several different algorithms to compute the least cost path. These algorithms are described below.

#### A\*

The basic algorithm provided within Thor is an A\* algorithm. This algorithm searches in one direction - from the origin towards the destination. The A\* heuristic is added to the cost to help guide the search more rapidly towards the destination. The A\* method has been superseded for most cases by the bidirectional A\* algorithm which has better performance. Also, the A\* algorithm does not work as well with transitions to upper hierarchy levels as the path approaches the destination.

#### Bidirectional A\*

The primary algorithm used for most types of routes is a bidirectional A\* method. This algorithm searches for the lowest cost path in two directions: one from the origin towards the destination and the other "backwards" from the destination towards the origin. This algorithm has better performance then the A\* algorithm since it more effectively cuts the search space. However, there are some complexities added to handle the backwards progression from the destination to the origin. Turn restrictions and transition costing is more complicated. Also, the determination of the connection point between the two searches (determination of route completion) is more complex. Another strength of the bidirectional A* method is that hierarchy transitions near the destination are simplified. For requests **without** a `date_time`, bidirectional A\* is the default algorithm. It does not advance a departure clock during search; time-dependent edge costs are not evaluated per edge in the search trees. When a `/route` request includes a supported `date_time` (`current`, `depart_at`, or `arrive_by`), Thor selects TDALT, TimeDepForward, or TimeDepReverse instead (see [Time-dependent routing](#time-dependent-routing) above).

Pedestrian and bicycle routes use just the local graph hierarchy. They never transition to the arterial or highway levels and thus never use shortcut edges.

The bidirectional A\* algorithm makes use of edge markings that enter regions where no through path exists. These are areas of the routing graph that represent cul-de-sas, dead-end roads, and even larger communities where there is only one entrance. The search paths can exclude ever entering an edge that is marked as "not-though".

#### Multi-modal

Multi-modal routes use an A\* method that is enahanced to allow time-dependency and mode changes. Public transit information includes schedule information that find the next departure along directed edges between transit stops. Unique pairs of transit stops and routes create separate graph edges with a unique *line-id* to which departure schedules can be associated.

#### Time-dependent routing

When a `/route` request includes a `date_time`, Thor selects a time-dependent path algorithm. Static requests (no `date_time` on any leg) continue to use **bidirectional A\*** with hierarchy pruning and shortcuts unchanged.

**Algorithm selection**

| Request | Algorithm | Notes |
| :------ | :-------- | :---- |
| No `date_time` | `bidir_astar` | Default static routing |
| `date_time.type` = `current` (0) or `depart_at` (1) | **TDALT** | When `thor.tdalt.enabled` is true and landmark data is available |
| Same, but TDALT unavailable | `TimeDepForward` | When `thor.tdalt.fallback_to_unidirectional` is true |
| Same, but fallback disabled | `bidir_astar` | Warning 499; ETA recosted for time-dependent accuracy |
| `date_time.type` = `arrive_by` (2) | `TimeDepReverse` | **Not TDALT** — project choice for arrival-time queries |
| `date_time.type` = `invariant` (3) | — | **Rejected** at request parse (HTTP 400, error 169) |

Multimodal, transit, and bikeshare costing modes use their own specialized algorithms regardless of `date_time`.

#### TDALT (Time-Dependent ALT)

**TDALT** (*Time-Dependent bidirectional A\* with ALT landmarks*) is the primary time-dependent algorithm for **`current`** and **`depart_at`** `/route` requests when enabled. It implements the bidirectional time-dependent search from Nannicini et al. using precomputed ALT landmark potentials on a lower-bound graph `G_λ`.

| Search tree | Graph | Costs | Time tracking |
| :---------- | :---- | :---- | :-------------- |
| Forward | `G` (routing graph) | Time-dependent edge costs `c(u,v,τ)` | Live `TimeInfo` at each expansion |
| Backward | `G_λ` (lower-bound graph) | Static lower bounds `λ(u,v)` | None |

The search runs in three phases: bidirectional meet, backward bounding, then forward-only completion restricted to nodes discovered by the backward search. This yields correct time-dependent paths (traffic, time restrictions, and costing rules such as `truck_ban` are evaluated on the forward tree during search, not only after the path is found).

**Requirements**

- Landmark sidecar built at tile-build time: `valhalla_build_tdalt_landmarks` writes `tdalt_landmarks.bin` (default 16 landmarks on `G_λ`).
- Runtime flag `thor.tdalt.enabled: true` (defaults to **false**).
- Landmark file path configured under `mjolnir.tdalt.landmarks_file`.

If landmarks are missing or TDALT is disabled, `current` / `depart_at` requests fall back per the table above.

**Configuration**

| Key | Default | Purpose |
| :-- | :------ | :------ |
| `mjolnir.tdalt.landmark_count` | 16 | Landmarks selected during preprocessing |
| `mjolnir.tdalt.landmarks_file` | — | Path to `tdalt_landmarks.bin` sidecar |
| `thor.tdalt.enabled` | `false` | Enable TDALT for `current` / `depart_at` |
| `thor.tdalt.fallback_to_unidirectional` | `false` | Fall back to `TimeDepForward` when TDALT unavailable |
| `thor.tdalt.approximation_factor` | 1.0 | Phase-2 approximation bound (`1.0` = exact) |
| `thor.tdalt.checkpoint_count` | 10 | Checkpoints for tightened backward potential |
| `thor.tdalt.max_reserved_labels_count` | (internal default) | Pre-allocated edge label pool size |

See also [TDALT FIFO audit](tdalt-fifo-audit.md) for correctness assumptions.

#### TimeDepForward and TimeDepReverse

These unidirectional A\* algorithms remain in Thor for fallback and for request types TDALT does not handle:

- **TimeDepForward** — forward search with live time-dependent costing. Used as fallback for `current` / `depart_at` when TDALT is unavailable and `thor.tdalt.fallback_to_unidirectional` is true, and for trivial same-edge cases when TDALT is disabled.
- **TimeDepReverse** — reverse search from the destination. Used for all **`arrive_by`** `/route` requests within `max_timedep_distance` (not TDALT in this deployment).

Both skip hierarchy shortcuts and are slower on long routes than bidirectional static search, which is why TDALT is preferred for departure-time queries when landmarks are available.

#### Deprecated / unsupported for time-dependent `/route`

| Feature | Status |
| :------ | :----- |
| `prioritize_bidirectional` | **Deprecated** for time-dependent routing — does not apply when TDALT or unidirectional TD is selected |
| `date_time.type` = `invariant` (3) | **Unsupported** — rejected with HTTP 400, error 169 |
| `reverse_time_tracking` | Obsolete for TD routing; only affects non-TD bidirectional A\* |

#### A* Heuristic

A simple class within Thor handles the A\* heuristic computation. At the beginning of PathAlgorithm::GetBestPath the A\* heuristic is initialized with the latitude, longitude of the destination and a costing factor to multiply distance estimates with. This factor needs to be tied to the costing model to multiply distance that will underestimate the cost to the destination, but keep close to a reasonable true cost so that performance is kept high. For example, in automobile costing the factor is based on the highest speed expected - thus any straight line distance estimate from a specific location will undersestimate the true cost on any path on real roads to get to the destination. Distance estimates are computed using a distance approximation method that computes a Euclidean distance using meters per degree of latitude and an estimate of meters per degree of longitude based on the destination latitude. This produces a close approximation of the arc distance along the surface of the earth while providing a distance measure that is locally stable (nearby locations will get consistent and close distance approximations).

#### Edge Labeling

Thor marks directed edges in the routing graph rather than nodes. This allows a node to be traversed multiple times in a route with different directed edges. This allows turn restrictions to be incorporated into the data and the path algorithm. This is demonstrated in the following example where a left turn is not allowed at an intersection. Rather the route must take a separate turn lane to the right and loop back through the intersection. The least cost path to the intersection node is to proceed straight. If the node were marked it would prevent traversing the node after using the turn lane since that path is higher cost. 

Each directed edge in the routing graph can have three states:

- **Not Visited** - An edge that is not visited has not yet been encountered within the PathAlgorithm graph traversal.
- **Temporary** - An edge that has been visited or encountered but there could still be a lower cost path to this edge. This edge will be "adjacent" or connected to an edge that is permanently labeled. Temporary edges are noted in the Adjacency List and are sorted such that they are "expanded" in order of lowest cost.
- **Permanent** - Lowest cost path to this edge has been found.

Edges that have been visited are stored in a vector with an EdgeLabel structure that contains information about the path up to this edge. In particular the predecessor edge is stored. This allows the shortest path of directed edges to be constructed by using each edges predecessor information to walk the path backwards. Additional information about the path to get to the directed edge is also kept. This information includes:

- **Edge Id** - Graph Id of the edge.
- **Opposing edge Id** - Graph Id of the opposing edge (for bidirectional A*).
- **End node** - GraphId of the end node of the edge. This allows the expansion to occur by reading the node and not having to re-read the directed edge to find its end node.
- **Cost** - Cost and elapsed time in seconds along the path to this edge.
- **Sort cost** - Cost including includes A* heuristic. 
- **Distance** - An estimate of the straight line distance to the destination.
- **Predecessor edge** - Index to the predecessor edge label information within the EdgeLabels list.

Several other pieces of information about the prior edge are also kept to avoid having to re-read an edge. In addition, several transit specific attributes are added for multi-modal routes.

#### Edge Status

An unordered map (hash map) is used to identify the state of directed edges. The map contains tile id as key and array of EdgeStatusInfo which contains 
index of the edge in the EdgeLabels vector and the current edge label state: kUnreachedOrReset, temporary or permanent.
Whenever a new tile (new edge in previously unvisited tile) is encountered a new value in the map is inserted with key as tile id and EdgeStatusInfo array of length equal to number of directed edges in the tile as value, all directed edges in the new array are initialized with kUnreachedOrReset status.

The index of edge in EdgeStatusInfo array is equal to it's id in the tile

EdgeStatus is constructed given an initial size of the edge status map. To avoid rehashing the initial size should be large enough.

- **Set** - Sets the status of a directed edge given its GraphId.
- **Update** - Updates the status of a directed edge given its GraphId.
- **Get** - Gets the status info of a directed edge given its GraphId.

#### Adjacency List

The AdjacencyList class provides a sorting order to the edge labels that are marked as temporary and are adjacent to edges that have lowest cost path found. The adjacency list uses a bucket sort implementation for performance. An "overflow" bucket is maintained to allow reduced memory use - costs outside the current bucket range get placed into the overflow bucket and are moved into the low-level buckets as needed. The adjacency list stores indexes into a list (vector) of labels where complete cost and predecessor information are stored. The adjacency list simply provides a fast sorting method. Benchmarks show a marked improvement over using an STL priority_queue, even in cases where the overflow bucket is utilized.

An AdjacencyList is constructed using a minimum cost (based on the A* heuristic distance from the origin location to the destination location), a range of costs held within the bucket sort, and a bucket size. All costs above mincost + range are stored in an "overflow" bucket. The following methods are provided in the AdjacencyList class:

- **Add** - Adds a label index to the sorted list. Adds it to the appropriate bucket given the sort cost. If the sortcost is greater than maxcost_ the label is placed in the overflow bucket. If the sortcost is < the current bucket cost then the label is placed at the front of the current bucket (this prevents underflow).
- **DecreaseCost** - The specified label index now has a smaller cost.  Reorders it in the sorted bucket list.
- **Clear** - Clear all labels from from the adjacency list. Called at the start of the path finding algorithm,
- **Remove** - Removes the lowest cost label index from the sorted list.
- **EmptyOverflow** - Empties the overflow bucket by placing the label indexes into the low level buckets. This method is private and is called from the Remove method when needed.
