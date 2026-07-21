# Truck region grid (`valhalla_build_region_grid`)

Offline tool that partitions the Europe truck subgraph into ~10 000 country-clipped
H3 regions, picks a network medoid per region, and assigns every truck-graph node
to a region via in-country network Voronoi ownership.

This artifact is the snap layer for later representative time-dependent matrices.
It does **not** compute ban safe-ranges or TD fill.

## Prerequisites

- Built Valhalla Europe tiles (`mjolnir.tile_dir` / extract in config)
- CMake build with tools enabled (vendored Uber H3 in `third_party/h3`)
- Recommended host: `r6i.8xlarge` (32 vCPU, 256 GB) for full Europe

On the **EC2 task image**, `valhalla_build_region_grid` is installed under `/usr/local/bin`
(compiled from `valhalla-fork` during `./buildEC2TaskImage.sh`). Tiles and `cch_truck.bin`
come from the base image; you can run the grid tool in the running task container without
an extra compile step.

## Build (local / fork tree)

```bash
cd valhalla-fork
cmake --build build --target valhalla_build_region_grid -j
```

## Run (task container)

```bash
valhalla_build_region_grid -c /custom_files/valhalla.json \
  --out-dir /custom_files/region_grid \
  --target-regions 10000 \
  --levels 0,1 \
  --max-class 6 \
  --hgv-only \
  --write-geojson
```

## Run (local binary)

```bash
./build/valhalla_build_region_grid -c /custom_files/valhalla.json \
  --out-dir /data/region_grid \
  --target-regions 10000 \
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
| `--target-regions` | `10000` | Soft target cell count after merge/split |
| `--concurrency` / `-j` | hardware | Tile load threads |
| `--write-geojson` | `false` | Also emit `regions.geojson` for QGIS |

## Outputs

| File | Contents |
|------|----------|
| `regions.csv` | `region_id,country,h3,rep_graph_id,rep_lat,rep_lon,node_count` |
| `node_regions.csv` | `graph_id,region_id,time_to_rep_s` for every truck node |
| `region_grid_meta.json` | `tile_build_hash`, levels, counts, tool version |
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
2. Weight countries by in-country truck edge time; allocate cell budgets  
3. H3 partition at base res 5, merge/split toward each country’s budget  
4. Network medoid per cell (sampled Dijkstra on cell + 1-ring halo)  
5. Multi-source Dijkstra Voronoi, **no cross-country ownership transfer**

## Tests

```bash
cmake --build build --target region_grid_partition region_grid_voronoi -j
./build/test/region_grid_partition
./build/test/region_grid_voronoi
# optional tile-backed smoke (gurka):
cmake --build build --target gurka_region_grid -j
./build/test/gurka/gurka_region_grid
```
