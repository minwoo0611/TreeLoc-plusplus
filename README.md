<div align="center">

<h1>TreeLoc++: Robust 6-DoF LiDAR Localization in Forests</h1>

Official repository for **TreeLoc++**.

TreeLoc is the predecessor of TreeLoc++.

<a href="https://github.com/minwoo0611/TreeLoc" target="_blank">TreeLoc</a> |
<a href="https://drive.google.com/drive/folders/1O0d3Xg3oDdF0GC7BhcHZsiglNQmsUBNn?usp=sharing" target="_blank">Dataset used in TreeLoc++</a>

</div>

### Recent Updates
- [2026/06/27] Updated the README with TreeLoc predecessor context and the public TreeLoc++ dataset link.
- [2026/06/25] Initial TreeLoc++ branch with intra-session and inter-session C++ code.

### Contributions
- **TreeLoc++ performs forest localization on compact Digital Forest Inventories (DFIs)**, representing tree stems with geometric attributes rather than dense point-cloud maps.
- **TreeLoc++ improves retrieval in structurally ambiguous forests** by combining TDH retrieval with pairwise tree-layout context, triangle verification, DBH consistency, and yaw-consistent inlier selection.
- **TreeLoc++ estimates full 6-DoF corrections from tree geometry** by jointly refining height, roll, and pitch after geometric matching.
- **TreeLoc++ supports long-term and multi-session evaluation** across repeated forest traversals using the same tree-level map interface.

### Data Source and Tree Extraction

TreeLoc++ consumes tree-level observations extracted before evaluation. The repository includes two Wild-Places Venman sample sequences, while the full processed Oxford and Wild-Places release is distributed separately through [Google Drive](https://drive.google.com/drive/folders/1O0d3Xg3oDdF0GC7BhcHZsiglNQmsUBNn?usp=sharing). Following TreeLoc, tree observations can be produced with [RealtimeTrees](https://github.com/ori-drs/realtime_trees) from the corresponding forest LiDAR recordings.

The bundled sample datasets correspond to:

- `Wild_V01` and `Wild_V02`: Wild-Places Venman sequences. The raw dataset is available from the [Wild-Places project page](https://csiro-robotics.github.io/Wild-Places/) and [CSIRO Data Access Portal](https://data.csiro.au/collection/csiro%3A56372).

The full public release also includes Oxford Evo, Stein am Rein, Forest of Dean, Wytham, and multi-session Evo23/Evo25 sequences. The Oxford raw recordings are available from the [Oxford Forest Place Recognition Dataset](https://dynamic.robots.ox.ac.uk/datasets/oxford-forest/).

The evaluator expects a processed dataset root:

```text
dataset_root/
├── trajectory.txt
├── TreeManagerState_0.csv
├── TreeManagerState_1.csv
└── ...
```

`trajectory.txt` uses:

```text
timestamp x y z qx qy qz qw
```

Each `TreeManagerState_<idx>.csv` must include:

- `axis_00` ... `axis_22`
- `location_x`, `location_y`, `location_z`
- `dbh` or `dbh_approximation`

Optional columns used when available:

- `reconstructed`
- `number_clusters`
- `score` or `scores`

### Public Dataset Download

The public processed TreeLoc++ dataset is available on [Google Drive](https://drive.google.com/drive/folders/1O0d3Xg3oDdF0GC7BhcHZsiglNQmsUBNn?usp=sharing). It is organized as:

```text
TreeLoc++_Dataset/
├── Oxford/
│   ├── Evo/
│   ├── Stein am Rein/
│   ├── Forest of Dean/
│   ├── Wytham/
│   ├── Evo23_00/ ... Evo23_05/
│   └── Evo25_00/ ... Evo25_08/
└── Wild-Places/
    ├── V-01/ ... V-04/
    └── K-01/ ... K-04/
```

Each sequence folder contains:

- `PCD_downsampled_0.1.zip`: downsampled LiDAR frames under `downsampled_0.1/`.
- `tree_information.zip`: per-frame tree-level CSV files.

Single-session sequences also provide `groundtruth.txt`. Multi-session Evo23/Evo25 sequences provide `initial_slam_results.csv` and `optimized.txt` for SLAM initialization and optimized trajectory references.

The sample datasets kept under `data/` are enough to run intra-session, inter-session, pose-edge export, and graph-optimization input generation without downloading the full release. Use the Google Drive dataset for the full Oxford and Wild-Places release.

### Prerequisites

TreeLoc++ is a C++17 CMake project.

- CMake >= 3.16
- C++17 compiler
- Eigen3

Ubuntu:

```bash
sudo apt update
sudo apt install build-essential cmake libeigen3-dev
```

### Build

```bash
cmake -S . -B build
cmake --build build -j
```

This builds:

- `treelocpp_intra`
- `treelocpp_inter`
- `treelocpp_graph_opt` when GTSAM is available

### Usage

The repository keeps two processed Wild-Places sample datasets under `data/`:

```text
data/Wild_V01/
data/Wild_V02/
```

#### Intra-Session Localization

```bash
./build/treelocpp_intra config/full_v01.yaml
./build/treelocpp_intra config/full_v02.yaml
```

#### Inter-Session Localization

```bash
./build/treelocpp_inter config/inter_v01_v02.yaml
```

`config/inter_v01_v02.yaml` uses the Wild_V test-region family selected by `test_region_family` and a 5 m ground-truth radius.

#### Pose-Edge Export

Set `pose_edges.enabled: true` in a config file to export GTSAM-compatible pose edges:

```text
q_idx db_idx overlap x y z roll pitch yaw
```

Inter-session configs can also provide comma-separated `dataset.query_roots`, `dataset.map_roots`, `dataset.query_labels`, and `dataset.map_labels` for multi-session batch export.

#### Graph Optimization

When GTSAM is installed, run:

```bash
./build/treelocpp_graph_opt sessions.csv results/pose_edges results/optimized
```

The `sessions.csv` rows are `label,slam_csv` or `label,key,slam_csv`.

### Configuration

Main parameters are grouped by role in:

- `config/full_v01.yaml` for full Wild_V01 intra-session evaluation
- `config/full_v02.yaml` for full Wild_V02 intra-session evaluation
- `config/inter_v01_v02.yaml` for full Wild_V01-to-Wild_V02 inter-session evaluation

Each YAML file includes inline comments for the exposed parameters.

### Acknowledgement

TreeLoc++ builds on [TreeLoc](https://arxiv.org/abs/2602.01501), tree-level representations extracted with [RealtimeTrees](https://github.com/ori-drs/realtime_trees), and forest LiDAR recordings from the [Wild-Places](https://csiro-robotics.github.io/Wild-Places/) and [Oxford Forest Place Recognition](https://dynamic.robots.ox.ac.uk/datasets/oxford-forest/) datasets.
