# Truck region grid (`valhalla_build_region_grid`)

Offline tool that partitions the truck subgraph into country-clipped H3 regions,
picks a network medoid per region, and assigns every truck-graph node to a region
via in-country network Voronoi ownership.

This artifact is the snap layer for later representative time-dependent matrices.
It does **not** compute ban safe-ranges or TD fill.

| Map region | Soft `--target-regions` | Density cutoffs |
|------------|-------------------------|-----------------|
| Europe tiles | **6000** | `dense_max_h3_res=5`, `dense_density_factor=1.5` |
| `central-eu` tiles (PL+DE+SK+CZ+AT) | **800** | **same** |

## Prerequisites

- Built Valhalla tiles for the map region (`mjolnir.tile_dir` / extract in config)
- CMake build with tools enabled (vendored Uber H3 in `third_party/h3`)
- Recommended host: `r6i.8xlarge` (32 vCPU, 256 GB) for full Europe; central-eu is lighter

On the **EC2 task image**, `valhalla_build_region_grid` is installed under `/usr/local/bin`
(compiled from `valhalla-fork` during `./buildEC2TaskImage.sh`). Tiles and optional
`cch_truck.bin` come from the base image; you can run the grid tool in the running task
container without an extra compile step.

## Build (local / fork tree)

```bash
cd valhalla-fork
cmake --build build --target valhalla_build_region_grid -j
```

## Run — Europe (task container)

```bash
valhalla_build_region_grid -c /custom_files/valhalla.json \
  --out-dir /custom_files/region_grid \
  --target-regions 6000 \
  --dense-max-h3-res 5 \
  --dense-density-factor 1.5 \
  --exclude-countries RU,BY \
  --levels 0,1 \
  --max-class 6 \
  --hgv-only \
  --write-geojson
```

## Run — central-eu (task container)

Same density cutoffs; smaller soft target for the five-country tile set:

```bash
valhalla_build_region_grid -c /custom_files/valhalla.json \
  --out-dir /data/region_grid_central_eu \
  --target-regions 800 \
  --dense-max-h3-res 5 \
  --dense-density-factor 1.5 \
  --exclude-countries RU,BY \
  --levels 0,1 \
  --max-class 6 \
  --hgv-only \
  --write-geojson
```

Then scaffold rep_matrix with `--central-eu` (see aleet
`private/modules/valhalla/scripts/README_rep_matrix.md`).

## Run (local binary)

```bash
./build/valhalla_build_region_grid -c /custom_files/valhalla.json \
  --out-dir /data/region_grid \
  --target-regions 6000 \
  --levels 0,1 \
  --max-class 6 \
  --hgv-only \
  --write-geojson
```

### Useful flags

| Flag | Default | Meaning |
|------|---------|---------|
| `--out-dir` | `/custom_files/region_grid` | Output directory |
| `--levels` | `0,1` | Hierarchy levels (highway + arterial) |
| `--max-class` | `6` | Max OSM road class kept |
| `--hgv-only` | `true` | Keep only `kTruckAccess` edges |
| `--truck-weight` | `40` | Metric tons for maxweight/… filters (match CostMatrix `weight`) |
| `--target-regions` | `6000` | Soft target cell count after merge/split |
| `--base-h3-res` | `5` | Seed H3 resolution |
| `--max-h3-res` | `6` | Hard cap for any cell |
| `--dense-max-h3-res` | `5` | Cap refinement for dense (high weight/km²) cells |
| `--dense-density-factor` | `1.5` | Dense if weight/km² ≥ factor × country mean |
| `--exclude-countries` | `RU,BY` | ISO2 codes dropped from the grid (empty = keep all) |
| `--concurrency` / `-j` | hardware | Tile load threads |
| `--write-geojson` | `false` | Also emit `regions.geojson` for QGIS |

## Outputs

| File | Contents |
|------|----------|
| `regions.csv` | `region_id,country,h3,rep_graph_id,rep_lat,rep_lon,node_count` |
| `node_regions.csv` | `graph_id,region_id,time_to_rep_s,lat,lon` for every assigned truck node |
| `region_grid_meta.json` | `tile_build_hash`, levels, counts, exclude list, tool version |
| `regions.geojson` | Optional H3 cell polygons |

Pin downstream jobs to `tile_build_hash` in the meta file so a tile rebuild forces a grid rebuild.

## Visualize

From the module root (needs matplotlib; for CSV→polygons also `h3>=4`):

```bash
poetry run python scripts/visualize_region_grid.py --dir /path/to/region_grid
# writes regions_map.png + medoids_map.png next to regions.csv (or --out-dir …)
```

If `regions.geojson` is absent, polygons are reconstructed from the `h3` column in `regions.csv`.

## Algorithm (summary)

1. `BuildTruckGraph` (optional HGV filter) from tiles  
2. Drop excluded countries (default RU/BY); weight remaining countries by in-country truck edge time; allocate cell budgets  
3. H3 partition at base res 5, merge/split toward each country’s budget  
4. Dense-area split cap: cells with high weight/km² stop refining at `dense_max_h3_res` (sparse cells may reach `max_h3_res`)  
5. Build truck subgraph with CostMatrix-aligned filters: `kTruckAccess`, impassable
   surface drop, `destonly_hgv` drop, dimensional access restrictions at `--truck-weight`
   (default 40 t, matching rep_matrix baseline)  
6. Per-country **giant strongly connected component** (directed); cells with no nodes in
   that SCC are dropped  
7. Network medoid per remaining cell (sampled Dijkstra on cell + 1-ring halo; candidates
   restricted to the giant SCC)  
8. Multi-source Dijkstra Voronoi, **no cross-country ownership transfer**

## Re-Voronoi after pruning reps

After a rep_matrix baseline, dead reps can be dropped (`prune-null-reps`) and ownership
rebuilt without re-picking medoids:

```bash
valhalla_revoronoi_region_grid -c /custom_files/valhalla.json \
  --regions /data/region_grid_central_eu/regions.csv \
  --out-dir /data/region_grid_central_eu \
  --levels 0,1 --max-class 6 --hgv-only --truck-weight 40
```

This rewrites `node_regions.csv` / `regions.csv` node counts from the kept
`rep_graph_id`s via `ComputeNetworkVoronoi`, and regenerates `regions.geojson`
so it matches the pruned region set.

## Tests

```bash
cmake --build build --target region_grid_partition region_grid_voronoi -j
./build/test/region_grid_partition
./build/test/region_grid_voronoi
# optional tile-backed smoke (gurka):
cmake --build build --target gurka_region_grid -j
./build/test/gurka/gurka_region_grid
```
